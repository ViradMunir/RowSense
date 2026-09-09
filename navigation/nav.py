#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PID Heading Hold + RealSense D435 Obstacle Avoidance + Cone-Based Row-End Turn
==============================================================================

Master script: cropRowAvoid (embankment-style DRIFT_OUT/TRAVERSE/DRIFT_BACK
obstacle avoidance with all FIX-1 through FIX-10 fixes intact). On top of
that, a row-end turn sequence triggered by orange traffic cones is bolted on
unchanged from continuous_row_end_turning.py.

Behaviour summary:
  - Default: IMU PID heading hold while cruising.
  - Embankment obstacle in corridor -> DRIFT_OUT, TRAVERSE, DRIFT_BACK,
    HEADING_HOLD restored. (cropRowAvoid behaviour, untouched.)
  - Confirmed orange cone <= CONE_APPROACH_DIST_M: ditch obstacle avoidance
    and enter the row-end turn sequence (CONE_APPROACH_1 -> TURN_1 ->
    TRANSIT -> CONE_APPROACH_2 -> TURN_2). After TURN_2, resume HEADING_HOLD
    and obstacle avoidance is live again.
  - Serpentine: turn pair 0 = right-right, pair 1 = left-left, etc.

Differences vs. the two source scripts:
  - IMU heading hold logic is the cropRowAvoid version (single source of truth).
  - Cone detection / row-end turn states are taken verbatim from
    continuous_row_end_turning.py.
  - Depth video and rosbag recording are removed. A single annotated RGB
    video is recorded instead, with overlays for state, obstacle bbox,
    cone bbox, distances and lateral offset.

Camera launch requirement: realsense2_camera with align_depth.enable:=true
so /camera/camera/aligned_depth_to_color/image_raw is published at the
same resolution as the color stream.
"""

import math
import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ============================================================
# PID HEADING HOLD TUNING  (from cropRowAvoid)
# ============================================================
ROI_TOP_CLIP_FRAC = 0.35
LINEAR_SPEED_INITIAL = 0.35
LINEAR_SPEED         = 0.25
SPEED_SWITCH_DISTANCE_M = 0.45
KP = 3.0
KI = 0.5
KD = 0.5
CONTROL_RATE_HZ = 30.0
MAX_ANGULAR_CMD = 0.60
INTEGRAL_CLAMP = 0.50
IMU_TIMEOUT = 0.25
CMD_STEER_CENTER = 0.0
STEER_POWER = 1.0
IMU_SIGN = 1.0
STEER_SIGN = 1.0
CALIBRATION_TIME_SEC = 1.5
CALIBRATION_MIN_SAMPLES = 100

# ============================================================
# CROSS-TRACK
# ============================================================
LATERAL_GAIN          = 3.0
LATERAL_OFFSET_CLAMP  = 1.0
LATERAL_SIGN          = 1.0

# ============================================================
# TRACK GEOMETRY
# ============================================================
FLAT_WIDTH_M             = 0.28
EMBANKMENT_EDGE_M        = 0.60
FLAT_HALF_WIDTH_M        = FLAT_WIDTH_M / 2.0
EMBANKMENT_OUTER_HALF_M  = EMBANKMENT_EDGE_M / 2.0
EMBANKMENT_LATERAL_SETPOINT_M = (FLAT_HALF_WIDTH_M + EMBANKMENT_OUTER_HALF_M) / 2.0

CORRIDOR_HALF_WIDTH_M    = FLAT_HALF_WIDTH_M
SHOULDER_OUTER_HALF_M    = EMBANKMENT_OUTER_HALF_M
EMBANKMENT_MAX_HEIGHT_M  = 0.10

# ============================================================
# OBSTACLE
# ============================================================
DETECT_DISTANCE_M        = 4.0
OBSTACLE_DIST_CRITICAL   = 0.30
LINEAR_SPEED_AVOID       = 0.3

MIN_VALID_DEPTH          = 0.35
MAX_VALID_DEPTH          = 4.5
DEPTH_SCALE              = 0.001
DEPTH_PROCESS_RATE_HZ    = 10.0

OBSTACLE_CONFIRM_FRAMES  = 3
OBSTACLE_CLEAR_FRAMES    = 8

BRAKE_LINEAR_X         = 0.0
BRAKE_DURATION_SEC     = 0.0

# ============================================================
# DRIFT
# ============================================================
DRIFT_OUT_DISTANCE_M     = 4.0
FULL_EMBANKMENT_BY_DISTANCE_M = 2.0
DRIFT_BACK_DISTANCE_M    = 2.5
TRAVERSE_MIN_DISTANCE_M  = 1.0
LATERAL_EMBANK_TOLERANCE_M = 0.04
LATERAL_HOME_TOLERANCE_M   = 0.04
SHOULDER_BLOCK_STOP_DISTANCE_M = 0.35
DRIFT_BACK_TIMEOUT_SEC   = 1.0
DRIFT_OUT_TIMEOUT_SEC    = 1.0

# ============================================================
# CAMERA MOUNT
# ============================================================
CAMERA_HEIGHT_M  = 0.12
CAMERA_PITCH_RAD = 0.02350

GROUND_HEIGHT_THRESHOLD_M = 0.05
MIN_OBSTACLE_PIXELS       = 35
MORPH_OPEN_KERNEL_PX      = 3

# ============================================================
# CONE DETECTION
# ============================================================
COLOR_IMAGE_TOPIC   = '/camera/camera/color/image_raw'
ALIGNED_DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'

# HSV ranges for orange traffic cone.
# Tuned against daylight rover footage (May 2026 sample): in bright sun,
# saturated cone tops wrap toward red (H near 0-4) so H lower = 0; cone
# faces in shadow can drop to S~80, V~80, so S/V lowers are 70 each.
# Upper H = 28 to allow slight glint on the tip without grabbing yellow grass.
# If false positives appear on dry yellow grass / dirt, raise S_lower from
# 70 toward 100, or narrow upper H from 28 toward 22.
CONE_HSV_LOWER = np.array([ 0,  70,  70], dtype=np.uint8)
CONE_HSV_UPPER = np.array([28, 255, 255], dtype=np.uint8)

# Min contour area. With the physical-height sanity check below in place,
# we can be a bit more permissive on raw area to catch farther cones.
# At 4.0m a 30cm cone is ~28x60px ~= 1700px max but irregular contour will
# be 200-400px in practice. Floor at 200 to keep some noise rejection.
CONE_MIN_AREA_PX        = 200
CONE_MIN_ASPECT_H_OVER_W = 0.7
CONE_DETECT_RATE_HZ     = 10.0
CONE_CONFIRM_FRAMES     = 2
CONE_DETECT_MAX_DIST_M  = 4.0
# Distance at which we PREEMPT obstacle avoidance and switch to cone-approach.
# This MUST be larger than the distance at which the cone first becomes a
# corridor obstacle (it shows up in depth at ~3-4m) -- otherwise the
# obstacle pipeline triggers DRIFT_OUT or STOPPED before this trigger ever
# gets to fire. Once preempted, the rover continues in CONE_APPROACH_1
# (with depth-obstacle pipeline gated off) until the cone reaches
# CONE_TURN_TRIGGER_M and the actual 90 degree turn starts.
CONE_PREEMPT_DIST_M     = 4.0
CONE_APPROACH_DIST_M    = 1.7
CONE_TURN_TRIGGER_M     = 1.3
CONE_LOST_TIMEOUT_SEC   = 4.0

# Physical-size sanity check. A real traffic cone is roughly 0.30m tall,
# and our smallest acceptable is around 0.10m (toy / small marker cone),
# largest around 0.55m (tall highway cone). For a candidate contour with
# bounding-box height bh_px at distance Z, estimated metric height is
# bh_px * Z / fy. If outside [CONE_MIN_PHYSICAL_HEIGHT_M,
# CONE_MAX_PHYSICAL_HEIGHT_M], reject -- it's some other orange thing
# (warning sign, post, dirt patch). Without this check, tall-thin orange
# objects in the scene get treated as cones.
CONE_MIN_PHYSICAL_HEIGHT_M = 0.10
CONE_MAX_PHYSICAL_HEIGHT_M = 0.55

# ============================================================
# ROW-END TURN PARAMETERS  (from continuous_row_end_turning, taken as-is)
# ============================================================
TURN_HEADING_TOLERANCE_RAD = math.radians(5.0)
# Pivot in place during the 90 degree row-end turn: linear=0, angular is
# a dedicated constant rather than the PID output (which would be clamped
# by MAX_ANGULAR_CMD=0.60 and take ages to complete).
TURN_LINEAR_SPEED      = 0.0
TURN_ANGULAR_SPEED     = 1.0
TURN_TIMEOUT_SEC       = 8.0
TRANSIT_LINEAR_SPEED   = 0.14
TRANSIT_TIMEOUT_SEC    = 18.0
POST_TURN_COOLDOWN_SEC = 1.5

# After TURN_2 completes, the wheels are still steered from the pivot and
# the rover sits in the cross-row gap where the side embankment can look
# like a corridor obstacle. Drive straight (angular=0, no PID) at a low
# speed for a short buffer to let the chassis straighten and move clear of
# the turn region before obstacle avoidance comes back online. The depth
# pipeline stays gated off through this state.
POST_TURN_SETTLE_LINEAR_SPEED = 0.18
POST_TURN_SETTLE_DISTANCE_M   = 0.35
POST_TURN_SETTLE_TIMEOUT_SEC  = 4.0

# ============================================================
# *** ROW-END TURN MASTER SWITCH ***
# Set to False to disable cone-triggered row-end turns (for pure
# obstacle-avoidance testing). All cone code stays intact, only the
# trigger in HEADING_HOLD is gated.
# ============================================================
ROW_END_TURN_ENABLED = True

# ============================================================
# VIDEO  (single annotated RGB video, no rosbag)
# ============================================================
LOG_PERIOD_SEC = 0.5

RECORD_VIDEO = True
VIDEO_OUTPUT_DIR = os.path.expanduser('~/recordings')
VIDEO_FPS = 10.0

# ============================================================
# STATE
# ============================================================
# Obstacle-avoidance states (cropRowAvoid)
STATE_HEADING_HOLD = 'HEADING_HOLD'
STATE_DRIFT_OUT    = 'DRIFT_OUT'
STATE_TRAVERSE     = 'TRAVERSE'
STATE_DRIFT_BACK   = 'DRIFT_BACK'
STATE_STOPPED      = 'STOPPED'
# Row-end states (continuous_row_end_turning)
STATE_CONE_APPROACH_1 = 'CONE_APPROACH_1'
STATE_TURN_1          = 'TURN_1'
STATE_TRANSIT         = 'TRANSIT_BETWEEN_ROWS'
STATE_CONE_APPROACH_2 = 'CONE_APPROACH_2'
STATE_TURN_2          = 'TURN_2'
STATE_POST_TURN_SETTLE = 'POST_TURN_SETTLE'

# States considered part of the row-end turn sequence (used to gate
# obstacle-avoidance behaviour off while turning).
ROW_END_STATES = {
    STATE_CONE_APPROACH_1, STATE_TURN_1, STATE_TRANSIT,
    STATE_CONE_APPROACH_2, STATE_TURN_2, STATE_POST_TURN_SETTLE,
}


class ImuHeadingHoldObstacleNode(Node):
    def __init__(self):
        super().__init__('imu_heading_hold_obstacle_node')

        # --- Visualization persistence ---
        self.last_bbox = None
        self.persist_counter = 0
        self.PERSIST_LIMIT = 20

        # --- QoS ---
        imu_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)

        # --- Subscribers ---
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, imu_qos)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw',
            self.depth_callback, sensor_qos)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/depth/camera_info',
            self.cam_info_callback, sensor_qos)
        self.color_sub = self.create_subscription(
            Image, COLOR_IMAGE_TOPIC,
            self.color_callback, sensor_qos)
        self.aligned_depth_sub = self.create_subscription(
            Image, ALIGNED_DEPTH_TOPIC,
            self.aligned_depth_callback, sensor_qos)

        # --- Publishers ---
        self.cmd_pub   = self.create_publisher(Twist,  '/cmd_vel', 10)
        self.alert_pub = self.create_publisher(String, '/obstacle_alert', 10)

        # --- Timers ---
        self.control_timer = self.create_timer(
            1.0 / CONTROL_RATE_HZ, self.control_loop)
        self.depth_timer = self.create_timer(
            1.0 / DEPTH_PROCESS_RATE_HZ, self.depth_process_loop)
        self.cone_timer = self.create_timer(
            1.0 / CONE_DETECT_RATE_HZ, self.cone_detect_loop)

        # --- IMU state ---
        self.last_imu_time = None
        self.latest_yaw = None
        self.latest_yaw_rate = 0.0
        self.heading_error_integral = 0.0
        self.last_control_time = None

        # --- Lateral / dead reckoning state ---
        self.lateral_offset   = 0.0
        self.lateral_setpoint = 0.0
        self.last_dr_time = None
        self.last_published_linear = 0.0
        self.forward_distance_total = 0.0
        self.startup_distance_anchor = None

        # --- Avoidance bookkeeping ---
        self.avoid_start_forward = 0.0
        self.traverse_start_forward = 0.0
        self.drift_back_start_forward = 0.0
        self.drift_back_start_setpoint = 0.0
        self.pre_avoid_target_yaw = None

        # --- Calibration ---
        self.calibration_start_time = self.get_clock().now()
        self.calibration_yaws = []
        self.heading_ready = False
        self.target_yaw = 0.0
        self.heading_lock_time = None

        # --- Depth state (corridor obstacles) ---
        self.latest_depth_frame = None
        self.depth_frame_received = False
        self.depth_intrinsics = None
        self._pixel_grid = None

        # --- Color / aligned depth state (cones + RGB video) ---
        self.latest_color_frame = None
        self.latest_aligned_depth = None
        self.color_frame_received = False
        self.aligned_depth_received = False
        self.aligned_depth_warned = False

        # --- State machine ---
        self.state = STATE_HEADING_HOLD
        self.state_start_time = None
        self.obstacle_confirm_count = 0
        self.obstacle_clear_count   = 0
        self.embankment_side        = 0

        self.brake_active = False
        self.brake_end_time = None

        self.obstacle_info = {
            'detected': False, 'min_dist': float('inf'),
            'center_min_dist': float('inf'), 'height_estimate': 0.0,
            'width_meters': 0.0, 'lateral_center': 0.0,
            'left_clear': True, 'right_clear': True,
            'center_blocked': False,
            'left_min': float('inf'), 'right_min': float('inf'),
            'cluster_ok': False,
            'obs_bbox': None,
        }

        # --- Cone state ---
        self.cone_confirm_count = 0
        self.cone_info = {
            'detected':  False,
            'confirmed': False,
            'distance':  float('inf'),
            'cx': None, 'cy': None,
            'bbox': None,
            'area': 0,
        }
        self.row_turn_in_progress = False
        self.last_turn_complete_time = None
        self.pre_turn_target_yaw = None
        self.turn_pair_count = 0
        self.settle_start_forward = 0.0

        # --- Logging / video ---
        self.last_log_time = self.get_clock().now()
        self.video_writer = None
        self.video_frame_size = None
        self.video_path = None

        self.get_logger().info(
            '=== PID Heading Hold + Embankment Bypass + Cone Row-End Turn ===')
        self.get_logger().info(
            f'flat=+/-{FLAT_HALF_WIDTH_M*100:.0f}cm, '
            f'embankment_outer=+/-{EMBANKMENT_OUTER_HALF_M*100:.0f}cm, '
            f'setpoint=+/-{EMBANKMENT_LATERAL_SETPOINT_M*100:.0f}cm')
        self.get_logger().info(
            f'detect@{DETECT_DISTANCE_M:.1f}m crit@{OBSTACLE_DIST_CRITICAL:.2f}m '
            f'confirm={OBSTACLE_CONFIRM_FRAMES} pitch={CAMERA_PITCH_RAD:+.2f}rad')
        self.get_logger().info(
            f'ROW_END_TURN_ENABLED={ROW_END_TURN_ENABLED}, '
            f'cone approach@{CONE_APPROACH_DIST_M:.2f}m, '
            f'turn trigger@{CONE_TURN_TRIGGER_M:.2f}m, '
            f'HSV={CONE_HSV_LOWER.tolist()}->{CONE_HSV_UPPER.tolist()}, '
            f'min_area={CONE_MIN_AREA_PX}px')

    # =========================================================
    # UTILITY
    # =========================================================
    @staticmethod
    def wrap_angle(a): return math.atan2(math.sin(a), math.cos(a))
    @staticmethod
    def quat_to_yaw(x, y, z, w):
        return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
    @staticmethod
    def circular_mean(angles):
        return math.atan2(sum(math.sin(a) for a in angles),
                          sum(math.cos(a) for a in angles))

    def send_alert(self, m):
        msg = String(); msg.data = m
        self.alert_pub.publish(msg)
        self.get_logger().warn(f'ALERT: {m}')

    def set_state(self, ns):
        if ns != self.state:
            self.get_logger().info(f'State: {self.state} -> {ns}')
            self.state = ns
            self.state_start_time = self.get_clock().now()
            if ns == STATE_STOPPED:
                t = self.get_clock().now().nanoseconds * 1e-9
                self.brake_active = True
                self.brake_end_time = t + BRAKE_DURATION_SEC
            elif ns == STATE_TRAVERSE:
                self.traverse_start_forward = self.forward_distance_total
            elif ns == STATE_DRIFT_BACK:
                self.drift_back_start_forward = self.forward_distance_total
                self.drift_back_start_setpoint = self.lateral_setpoint
            self.obstacle_clear_count = 0
            self.obstacle_confirm_count = 0

    def state_elapsed(self):
        if self.state_start_time is None: return 0.0
        return (self.get_clock().now() - self.state_start_time).nanoseconds * 1e-9

    @staticmethod
    def clamp01(x): return max(0.0, min(1.0, float(x)))
    @staticmethod
    def smoothstep(x):
        x = max(0.0, min(1.0, float(x)))
        return x*x*(3.0 - 2.0*x)

    def get_current_speed(self):
        if self.startup_distance_anchor is None:
            return LINEAR_SPEED
        travelled_since_lock = (self.forward_distance_total -
                                self.startup_distance_anchor)
        if travelled_since_lock < SPEED_SWITCH_DISTANCE_M:
            return LINEAR_SPEED_INITIAL
        return LINEAR_SPEED

    def in_post_turn_cooldown(self) -> bool:
        if self.last_turn_complete_time is None:
            return False
        elapsed = (self.get_clock().now() -
                   self.last_turn_complete_time).nanoseconds * 1e-9
        return elapsed < POST_TURN_COOLDOWN_SEC

    def heading_error_deg(self) -> float:
        if self.latest_yaw is None:
            return 0.0
        return math.degrees(self.wrap_angle(self.latest_yaw - self.target_yaw))

    # =========================================================
    # VIDEO RECORDING (single annotated RGB video)
    # =========================================================
    def start_video_recording(self, frame_shape):
        try:
            os.makedirs(VIDEO_OUTPUT_DIR, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.video_path = os.path.join(
                VIDEO_OUTPUT_DIR, f'rgb_recording_{stamp}.avi')
            h, w = frame_shape[:2]
            self.video_frame_size = (w, h)
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(
                self.video_path, fourcc, VIDEO_FPS, self.video_frame_size)
            self.get_logger().info(f'RGB video recording started: {self.video_path}')
        except Exception as e:
            self.get_logger().error(f'Video init failed: {e}')
            self.video_writer = None

    def write_video_frame(self, bgr_img, obs):
        """Annotate the live RGB frame and write a single output video.
        Note: obstacle bbox stored in self.last_bbox is in DEPTH image
        coordinates. Color and depth are different resolutions in general,
        so we draw it scaled to the color image size."""
        if self.video_writer is None:
            return

        display_img = bgr_img.copy()
        ch, cw = display_img.shape[:2]

        # --- Obstacle bbox (depth coords -> color coords by aspect scale) ---
        if self.last_bbox is not None and self.latest_depth_frame is not None:
            x1, y1, x2, y2 = self.last_bbox
            dh, dw = self.latest_depth_frame.shape
            sx = cw / float(dw)
            sy = ch / float(dh)
            cx1, cy1 = int(x1 * sx), int(y1 * sy)
            cx2, cy2 = int(x2 * sx), int(y2 * sy)
            cv2.rectangle(display_img, (cx1, cy1), (cx2, cy2), (0, 0, 255), 3)
            dl = (f"{obs['min_dist']:.2f}m"
                  if obs['min_dist'] < 10 else "--")
            h_cm = obs['height_estimate'] * 100
            cv2.putText(display_img,
                        f"OBS d={dl}  h={h_cm:.0f}cm",
                        (max(0, cx1), max(15, cy1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

        # --- Cone bbox (already in color coords, since cone runs on color) ---
        if self.cone_info.get('bbox') is not None:
            x1, y1, x2, y2 = self.cone_info['bbox']
            cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 165, 255), 3)
            d = self.cone_info['distance']
            d_str = f"{d:.2f}m" if d < 10 else "--"
            cv2.putText(display_img,
                        f"CONE d={d_str} conf={self.cone_confirm_count}",
                        (max(0, x1), min(ch - 5, y2 + 18)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)

        # --- HUD ---
        in_row_end = self.state in ROW_END_STATES
        state_color = ((0, 165, 255) if in_row_end
                       else (0, 255, 0) if obs['min_dist'] > DETECT_DISTANCE_M
                       else (0, 165, 255))
        cv2.putText(display_img, f"STATE: {self.state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
        cv2.putText(display_img,
                    f"lat={self.lateral_offset*100:+.0f}cm "
                    f"set={self.lateral_setpoint*100:+.0f}cm",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 2)
        cv2.putText(display_img,
                    f"yaw_err={self.heading_error_deg():+.1f}deg "
                    f"pair={self.turn_pair_count}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 2)

        self.video_writer.write(display_img)

    def stop_video_recording(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
            if self.video_path:
                self.get_logger().info(f'Video saved: {self.video_path}')

    # =========================================================
    # PUBLISH HELPERS
    # =========================================================
    def publish_stop(self):
        c = Twist(); c.linear.x = 0.0; c.angular.z = CMD_STEER_CENTER
        self.cmd_pub.publish(c)

    def publish_cmd(self, lx, az):
        c = Twist()
        c.linear.x = float(lx)
        c.angular.z = float(max(-MAX_ANGULAR_CMD, min(MAX_ANGULAR_CMD, az)))
        self.cmd_pub.publish(c)
        self.last_published_linear = float(lx)

    def maybe_log(self, m):
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds * 1e-9 >= LOG_PERIOD_SEC:
            self.get_logger().info(m)
            self.last_log_time = now

    # =========================================================
    # IMU CALLBACK
    # =========================================================
    def imu_callback(self, msg):
        self.last_imu_time = self.get_clock().now()
        ry = self.quat_to_yaw(msg.orientation.x, msg.orientation.y,
                              msg.orientation.z, msg.orientation.w)
        self.latest_yaw = self.wrap_angle(IMU_SIGN * ry)
        self.latest_yaw_rate = IMU_SIGN * float(msg.angular_velocity.z)

        if not self.heading_ready:
            self.calibration_yaws.append(self.latest_yaw)
            el = (self.last_imu_time - self.calibration_start_time).nanoseconds * 1e-9
            if (el >= CALIBRATION_TIME_SEC and
                    len(self.calibration_yaws) >= CALIBRATION_MIN_SAMPLES):
                self.target_yaw = self.circular_mean(self.calibration_yaws)
                self.heading_ready = True
                self.heading_lock_time = self.get_clock().now()
                self.startup_distance_anchor = self.forward_distance_total
                self.lateral_offset = 0.0
                self.lateral_setpoint = 0.0
                self.last_dr_time = None
                self.get_logger().info(
                    f'Heading hold active. yaw={self.target_yaw:+.4f} '
                    f'({len(self.calibration_yaws)} samples)')

    # =========================================================
    # CAMERA INFO CALLBACK
    # =========================================================
    def cam_info_callback(self, msg):
        if self.depth_intrinsics is None:
            K = msg.k
            self.depth_intrinsics = (float(K[0]), float(K[4]),
                                     float(K[2]), float(K[5]))
            self.get_logger().info(
                f'Intrinsics fx={K[0]:.1f} fy={K[4]:.1f} '
                f'cx={K[2]:.1f} cy={K[5]:.1f}')

    # =========================================================
    # DEPTH / COLOR / ALIGNED-DEPTH CALLBACKS
    # =========================================================
    def depth_callback(self, msg):
        try:
            if msg.encoding not in ('16UC1', 'mono16'):
                self.get_logger().warn(f'Bad depth enc: {msg.encoding}', once=True)
                return
            frame = np.frombuffer(
                msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            # Clip top ROI fraction (sky) so those rows are ignored.
            clip_rows = int(msg.height * ROI_TOP_CLIP_FRAC)
            if clip_rows > 0:
                frame = frame.copy()
                frame[:clip_rows, :] = 0
            self.latest_depth_frame = frame
            self.depth_frame_received = True
        except Exception as e:
            self.get_logger().error(f'Depth err: {e}')

    def color_callback(self, msg):
        try:
            if msg.encoding not in ('rgb8', 'bgr8'):
                self.get_logger().warn(
                    f'Unexpected color encoding: {msg.encoding}', once=True)
                return
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                img = img.copy()
            self.latest_color_frame = img
            self.color_frame_received = True

            # Drive the single annotated RGB video off the color stream.
            if RECORD_VIDEO:
                if self.video_writer is None:
                    self.start_video_recording(img.shape)
                self.write_video_frame(img, self.obstacle_info)
        except Exception as e:
            self.get_logger().error(f'Color frame error: {e}')

    def aligned_depth_callback(self, msg):
        try:
            if msg.encoding not in ('16UC1', 'mono16'):
                self.get_logger().warn(
                    f'Unexpected aligned-depth encoding: {msg.encoding}',
                    once=True)
                return
            self.latest_aligned_depth = np.frombuffer(
                msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            self.aligned_depth_received = True
        except Exception as e:
            self.get_logger().error(f'Aligned-depth frame error: {e}')

    # =========================================================
    # DEPTH PROCESSING (corridor obstacles)
    # =========================================================
    def _get_pixel_grid(self, h, w):
        if self._pixel_grid is None or self._pixel_grid[0].shape != (h, w):
            U, V = np.meshgrid(np.arange(w, dtype=np.float32),
                               np.arange(h, dtype=np.float32))
            self._pixel_grid = (U, V)
        return self._pixel_grid

    def depth_process_loop(self):
        # Gate: depth-based obstacle detection is fully disabled while the
        # row-end turn sequence is active. The cone itself is a tall, solid,
        # close-range object and would otherwise be classified as a corridor
        # obstacle (triggering DRIFT_OUT or STOPPED) before the cone-based
        # turn ever has a chance to start. Force obstacle_info to a clean
        # "all clear" snapshot so any state still reading it gets nothing.
        if self.state in ROW_END_STATES:
            self.obstacle_info = {
                'detected': False, 'min_dist': float('inf'),
                'center_min_dist': float('inf'), 'height_estimate': 0.0,
                'width_meters': 0.0, 'lateral_center': 0.0,
                'left_clear': True, 'right_clear': True,
                'center_blocked': False,
                'left_min': float('inf'), 'right_min': float('inf'),
                'cluster_ok': False,
                'obs_bbox': None,
            }
            self.obstacle_confirm_count = 0
            self.obstacle_clear_count = 0
            self.last_bbox = None
            self.persist_counter = 0
            return

        if not self.depth_frame_received or self.latest_depth_frame is None:
            return
        if self.depth_intrinsics is None:
            self.maybe_log('Waiting for camera_info...')
            return

        fx, fy, cx, cy = self.depth_intrinsics
        frame = self.latest_depth_frame
        h, w = frame.shape

        Z = frame.astype(np.float32) * DEPTH_SCALE
        U, V = self._get_pixel_grid(h, w)
        X_cam = (U - cx) * Z / fx
        Y_cam = (V - cy) * Z / fy

        if CAMERA_PITCH_RAD != 0.0:
            cp = math.cos(CAMERA_PITCH_RAD)
            sp = math.sin(CAMERA_PITCH_RAD)
            Y_level =  Y_cam * cp + Z * sp
            Z_level = -Y_cam * sp + Z * cp
        else:
            Y_level = Y_cam
            Z_level = Z

        H_above = CAMERA_HEIGHT_M - Y_level

        if self.heading_ready and self.latest_yaw is not None:
            yaw_err = self.wrap_angle(self.latest_yaw - self.target_yaw)
        else:
            yaw_err = 0.0
        cy_y = math.cos(yaw_err); sy_y = math.sin(yaw_err)
        X_path = X_cam * cy_y - Z_level * sy_y - LATERAL_SIGN * self.lateral_offset
        Z_path = X_cam * sy_y + Z_level * cy_y

        valid_z  = (Z > MIN_VALID_DEPTH) & (Z < MAX_VALID_DEPTH)
        in_range = valid_z & (Z_path > 0.0) & (Z_path < DETECT_DISTANCE_M)

        in_corr     = np.abs(X_path) < CORRIDOR_HALF_WIDTH_M
        in_should_l = (X_path <= -CORRIDOR_HALF_WIDTH_M) & (X_path > -SHOULDER_OUTER_HALF_M)
        in_should_r = (X_path >=  CORRIDOR_HALF_WIDTH_M) & (X_path <  SHOULDER_OUTER_HALF_M)

        above_gnd = H_above > GROUND_HEIGHT_THRESHOLD_M
        below_top = H_above < 1.5

        in_embank_band = (np.abs(X_path) >= CORRIDOR_HALF_WIDTH_M) & \
                         (np.abs(X_path) <  SHOULDER_OUTER_HALF_M)
        embank_terrain = in_embank_band & (H_above < EMBANKMENT_MAX_HEIGHT_M)

        obs_layer_raw = above_gnd & below_top & in_range & ~embank_terrain

        if MORPH_OPEN_KERNEL_PX > 0:
            k = np.ones((MORPH_OPEN_KERNEL_PX, MORPH_OPEN_KERNEL_PX), np.uint8)
            obs_layer = cv2.morphologyEx(obs_layer_raw.astype(np.uint8),
                                         cv2.MORPH_OPEN, k).astype(bool)
        else:
            obs_layer = obs_layer_raw

        corr_mask  = in_corr     & obs_layer
        left_mask  = in_should_l & obs_layer
        right_mask = in_should_r & obs_layer

        n_corr_px = int(np.count_nonzero(corr_mask))
        cluster_ok = n_corr_px >= MIN_OBSTACLE_PIXELS

        if cluster_ok:
            zc = Z_path[corr_mask]; hc = H_above[corr_mask]; xc = X_path[corr_mask]
            center_min_dist = float(np.percentile(zc, 5))
            max_height      = float(np.percentile(hc, 95))
            lateral_min     = float(np.min(xc))
            lateral_max     = float(np.max(xc))
            lateral_width   = lateral_max - lateral_min
            lateral_center  = 0.5 * (lateral_min + lateral_max)
        else:
            center_min_dist = float('inf'); max_height = 0.0
            lateral_width = 0.0; lateral_center = 0.0

        if np.count_nonzero(left_mask) >= MIN_OBSTACLE_PIXELS:
            left_min = float(np.percentile(Z_path[left_mask], 5))
        else:
            left_min = float('inf')
        if np.count_nonzero(right_mask) >= MIN_OBSTACLE_PIXELS:
            right_min = float(np.percentile(Z_path[right_mask], 5))
        else:
            right_min = float('inf')

        center_blocked_now = cluster_ok and (center_min_dist < DETECT_DISTANCE_M)
        if center_blocked_now:
            self.obstacle_confirm_count = min(self.obstacle_confirm_count + 1,
                                              OBSTACLE_CONFIRM_FRAMES + 2)
        else:
            self.obstacle_confirm_count = max(self.obstacle_confirm_count - 1, 0)
        confirmed = self.obstacle_confirm_count >= OBSTACLE_CONFIRM_FRAMES

        if cluster_ok:
            ri = np.where(np.any(corr_mask, axis=1))[0]
            ci = np.where(np.any(corr_mask, axis=0))[0]
            self.last_bbox = (int(ci[0]), int(ri[0]), int(ci[-1]), int(ri[-1]))
            self.persist_counter = self.PERSIST_LIMIT
        else:
            if self.persist_counter > 0: self.persist_counter -= 1
            else: self.last_bbox = None

        self.obstacle_info = {
            'detected': confirmed, 'min_dist': center_min_dist,
            'center_min_dist': center_min_dist,
            'height_estimate': max_height, 'width_meters': lateral_width,
            'lateral_center': lateral_center,
            'left_clear':  left_min  > DETECT_DISTANCE_M,
            'right_clear': right_min > DETECT_DISTANCE_M,
            'center_blocked': center_blocked_now and confirmed,
            'left_min': left_min, 'right_min': right_min,
            'cluster_ok': cluster_ok,
            'obs_bbox': self.last_bbox,
        }

    # =========================================================
    # CONE DETECTION
    # =========================================================
    def cone_detect_loop(self):
        if self.latest_color_frame is None:
            return

        if self.latest_aligned_depth is None:
            if not self.aligned_depth_warned:
                self.get_logger().warn(
                    f'No frames on {ALIGNED_DEPTH_TOPIC} yet. Cone detection '
                    f'is OFF. Launch realsense2_camera with '
                    f'align_depth.enable:=true', once=True)
                self.aligned_depth_warned = True
            return

        bgr   = self.latest_color_frame
        depth = self.latest_aligned_depth

        if bgr.shape[:2] != depth.shape:
            if not self.aligned_depth_warned:
                self.get_logger().warn(
                    f'Color {bgr.shape[:2]} != aligned depth {depth.shape}. '
                    f'Cone detection disabled. Check '
                    f'align_depth.enable:=true.', once=True)
                self.aligned_depth_warned = True
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, CONE_HSV_LOWER, CONE_HSV_UPPER)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_score = -1.0
        dh, dw = depth.shape

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < CONE_MIN_AREA_PX:
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            if bw <= 0 or bh <= 0:
                continue
            aspect = bh / float(bw)
            if aspect < CONE_MIN_ASPECT_H_OVER_W:
                continue

            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            ucx = int(M['m10'] / M['m00'])
            ucy = int(M['m01'] / M['m00'])
            if not (0 <= ucx < dw and 0 <= ucy < dh):
                continue

            win = 5
            y0, y1 = max(0, ucy - win), min(dh, ucy + win + 1)
            x0, x1 = max(0, ucx - win), min(dw, ucx + win + 1)
            patch = depth[y0:y1, x0:x1].astype(np.float32) * DEPTH_SCALE
            valid = patch[(patch > MIN_VALID_DEPTH) &
                          (patch < CONE_DETECT_MAX_DIST_M + 1.0)]
            if valid.size < 3:
                continue
            dist = float(np.median(valid))
            if dist > CONE_DETECT_MAX_DIST_M:
                continue

            # Physical-size sanity check: reject any orange tall-thin
            # object whose computed real-world height isn't cone-sized.
            # This filters out warning signs, posts, dirt patches, etc.
            # Uses depth-camera fy as an approximation for color fy (D435
            # values are within ~5% on these models, which is fine given
            # the wide bounds [0.10m, 0.55m]).
            if self.depth_intrinsics is not None:
                fy = self.depth_intrinsics[1]
                if fy > 0:
                    physical_h_m = float(bh) * dist / fy
                    if not (CONE_MIN_PHYSICAL_HEIGHT_M
                            <= physical_h_m
                            <= CONE_MAX_PHYSICAL_HEIGHT_M):
                        continue

            score = area / max(0.2, dist)
            if score > best_score:
                best_score = score
                best = {
                    'cx': ucx, 'cy': ucy,
                    'distance': dist,
                    'area': float(area),
                    'bbox': (int(x), int(y), int(x + bw), int(y + bh)),
                    'aspect': aspect,
                }

        if best is not None:
            self.cone_confirm_count = min(
                self.cone_confirm_count + 1, CONE_CONFIRM_FRAMES + 2)
        else:
            self.cone_confirm_count = max(0, self.cone_confirm_count - 1)

        confirmed = (best is not None and
                     self.cone_confirm_count >= CONE_CONFIRM_FRAMES)

        self.cone_info = {
            'detected':  best is not None,
            'confirmed': confirmed,
            'distance':  best['distance'] if best else float('inf'),
            'cx':        best['cx']       if best else None,
            'cy':        best['cy']       if best else None,
            'bbox':      best['bbox']     if best else None,
            'area':      best['area']     if best else 0,
        }

    # =========================================================
    # LATERAL DEAD-RECKONING + PID
    # =========================================================
    def update_lateral_dr(self):
        if not self.heading_ready or self.latest_yaw is None: return
        now = self.get_clock().now()
        if self.last_dr_time is None:
            self.last_dr_time = now; return
        dt = (now - self.last_dr_time).nanoseconds * 1e-9
        self.last_dr_time = now
        if dt <= 0.0 or dt > 0.5: return
        v = self.last_published_linear
        if abs(v) < 1e-3: return
        if v > 0.0: self.forward_distance_total += v * dt
        ye = self.wrap_angle(self.latest_yaw - self.target_yaw)
        self.lateral_offset += v * math.sin(ye) * dt
        self.lateral_offset = max(-LATERAL_OFFSET_CLAMP,
                                  min(LATERAL_OFFSET_CLAMP, self.lateral_offset))

    def compute_pid_angular(self):
        he = self.wrap_angle(self.latest_yaw - self.target_yaw)
        now = self.get_clock().now()
        dt = ((now - self.last_control_time).nanoseconds * 1e-9
              if self.last_control_time is not None else 1.0/CONTROL_RATE_HZ)
        self.last_control_time = now
        self.heading_error_integral += he * dt
        self.heading_error_integral = max(-INTEGRAL_CLAMP,
                                          min(INTEGRAL_CLAMP, self.heading_error_integral))
        lat_err = self.lateral_offset - self.lateral_setpoint
        base = (KP*he + KI*self.heading_error_integral
                - KD*self.latest_yaw_rate
                + LATERAL_GAIN*LATERAL_SIGN*lat_err)
        ang = CMD_STEER_CENTER - (STEER_SIGN * STEER_POWER * base)
        return max(-MAX_ANGULAR_CMD, min(MAX_ANGULAR_CMD, ang))

    def _choose_embankment_side(self, obs):
        if obs['left_min'] - obs['right_min'] > 0.20: return +1
        if obs['right_min'] - obs['left_min'] > 0.20: return -1
        if obs['lateral_center'] >  0.02: return +1
        if obs['lateral_center'] < -0.02: return -1
        return +1

    def _restore_pre_avoid_heading(self):
        if self.pre_avoid_target_yaw is not None:
            self.target_yaw = self.pre_avoid_target_yaw
            self.get_logger().info(
                f'Heading restored to pre-avoid yaw={self.target_yaw:+.4f}')
            self.pre_avoid_target_yaw = None
        self.lateral_offset = 0.0
        self.lateral_setpoint = 0.0
        self.heading_error_integral = 0.0
        self.last_dr_time = None
        self.embankment_side = 0

    def _real_critical(self, obs):
        return (obs['cluster_ok'] and
                math.isfinite(obs['min_dist']) and
                obs['min_dist'] < OBSTACLE_DIST_CRITICAL and
                self.obstacle_confirm_count >= OBSTACLE_CONFIRM_FRAMES)

    # =========================================================
    # ROW-END TURN HELPERS
    # =========================================================
    def _begin_turn(self, label: str):
        # Alternate direction each row pair to produce serpentine traversal.
        # Pair 0 (rows 1->2): both turns RIGHT.
        # Pair 1 (rows 2->3): both turns LEFT.
        # Pair 2 (rows 3->4): both turns RIGHT. etc.
        if self.turn_pair_count % 2 == 0:
            angle = -math.pi / 2.0   # right
            dir_label = 'RIGHT'
        else:
            angle = +math.pi / 2.0   # left
            dir_label = 'LEFT'
        self.pre_turn_target_yaw = self.target_yaw
        self.target_yaw = self.wrap_angle(self.target_yaw + angle)
        self.heading_error_integral = 0.0
        # Clear lateral DR state so the obstacle-avoidance lateral_offset
        # bookkeeping doesn't fight the new heading.
        self.lateral_offset = 0.0
        self.lateral_setpoint = 0.0
        self.last_dr_time = None
        self.send_alert(
            f'{label} ({dir_label}, pair {self.turn_pair_count}): '
            f'target_yaw {math.degrees(self.pre_turn_target_yaw):+.1f}'
            f' -> {math.degrees(self.target_yaw):+.1f} deg')

    def _cone_trigger_ready(self) -> bool:
        return (self.cone_info['confirmed'] and
                self.cone_info['distance'] < CONE_PREEMPT_DIST_M and
                not self.in_post_turn_cooldown())

    # =========================================================
    # MAIN CONTROL LOOP
    # =========================================================
    def control_loop(self):
        # --- IMU checks ---
        if self.last_imu_time is None or self.latest_yaw is None:
            self.publish_stop(); self.maybe_log('Waiting for /imu...'); return
        if (self.get_clock().now() - self.last_imu_time).nanoseconds * 1e-9 > IMU_TIMEOUT:
            self.publish_stop(); self.maybe_log('IMU timeout'); return
        if not self.heading_ready:
            self.publish_stop()
            el = (self.get_clock().now() - self.calibration_start_time).nanoseconds * 1e-9
            self.maybe_log(f'Cal {len(self.calibration_yaws)}/{CALIBRATION_MIN_SAMPLES} '
                           f'{el:.1f}/{CALIBRATION_TIME_SEC:.1f}s')
            return

        self.update_lateral_dr()
        obs  = self.obstacle_info
        cone = self.cone_info

        # =====================================================
        # === GLOBAL CONE TRIGGER (highest priority) ===
        # If a confirmed orange cone is within approach range AND we are
        # currently in any obstacle-avoidance state (HEADING_HOLD,
        # STOPPED, DRIFT_OUT, TRAVERSE, DRIFT_BACK), drop everything and
        # jump straight to CONE_APPROACH_1. This stops the depth pipeline
        # from misclassifying the cone itself as a corridor obstacle and
        # latching the rover into DRIFT_OUT or STOPPED before the row-end
        # turn ever gets a chance to start.
        # Only checked when NOT already in a row-end state -- the row-end
        # state machine handles its own transitions.
        # =====================================================
        if (ROW_END_TURN_ENABLED and
                self.state not in ROW_END_STATES and
                self._cone_trigger_ready()):
            self.send_alert(
                f'Row-end cone at {cone["distance"]:.2f}m '
                f'(was in {self.state}). Ditching obstacle avoidance, '
                f'beginning row-end turn sequence.')
            self.row_turn_in_progress = True
            # Wipe all obstacle-avoidance bookkeeping so nothing stale
            # leaks into the row-end sequence.
            self.lateral_offset = 0.0
            self.lateral_setpoint = 0.0
            self.last_dr_time = None
            self.heading_error_integral = 0.0
            self.embankment_side = 0
            self.pre_avoid_target_yaw = None
            self.brake_active = False
            self.brake_end_time = None
            self.obstacle_confirm_count = 0
            self.obstacle_clear_count = 0
            self.set_state(STATE_CONE_APPROACH_1)
            return

        # =====================================================
        # === ROW-END STATES (highest priority once entered) ===
        # While in any of these states, normal obstacle avoidance is
        # suppressed -- the cone-based turn sequence runs to completion
        # before HEADING_HOLD takes over again.
        # =====================================================

        # === STATE: CONE_APPROACH_1 ===
        if self.state == STATE_CONE_APPROACH_1:
            ang = self.compute_pid_angular()
            self.publish_cmd(LINEAR_SPEED_AVOID, ang)
            self.maybe_log(
                f'APPROACH 1 | cone d={cone["distance"]:.2f}m | '
                f'conf={self.cone_confirm_count}')

            if (cone['confirmed'] and
                    cone['distance'] <= CONE_TURN_TRIGGER_M):
                self._begin_turn('TURN 1 (row exit)')
                self.set_state(STATE_TURN_1)
                return

            if (self.state_elapsed() > CONE_LOST_TIMEOUT_SEC and
                    not cone['confirmed']):
                self.send_alert('Cone 1 lost during approach. Resuming hold.')
                self.set_state(STATE_HEADING_HOLD)
                return
            return

        # === STATE: TURN_1 ===
        if self.state == STATE_TURN_1:
            # Pivot in place: linear=0, angular at TURN_ANGULAR_SPEED in the
            # direction of the heading error. Bypass publish_cmd because its
            # MAX_ANGULAR_CMD clamp would limit the turn rate.
            he = self.wrap_angle(self.latest_yaw - self.target_yaw)
            # he > 0 means current yaw is ahead of target -> need to turn
            # negative (CW); he < 0 means current yaw is behind target ->
            # turn positive (CCW). cmd.z sign matches -he.
            turn_dir = -1.0 if he > 0 else +1.0
            tw = Twist()
            tw.linear.x = 0.0
            tw.angular.z = turn_dir * TURN_ANGULAR_SPEED
            self.cmd_pub.publish(tw)
            self.last_published_linear = 0.0

            err_deg = abs(math.degrees(he))
            self.maybe_log(
                f'TURN 1 PIVOT | err={err_deg:.1f}deg | '
                f'cmd.z={tw.angular.z:+.3f} | t={self.state_elapsed():.1f}s')

            if err_deg < math.degrees(TURN_HEADING_TOLERANCE_RAD):
                self.send_alert(
                    f'Turn 1 complete (err={err_deg:.1f}deg). '
                    f'Heading transit between rows.')
                self.heading_error_integral = 0.0
                self.cone_confirm_count = 0
                self.set_state(STATE_TRANSIT)
                return

            if self.state_elapsed() > TURN_TIMEOUT_SEC:
                if err_deg > 30.0:
                    self.send_alert(
                        f'Turn 1 FAILED: only {90.0 - err_deg:.1f}deg of turn '
                        f'achieved before timeout. Stopping for safety.')
                    self.set_state(STATE_STOPPED)
                    return
                self.send_alert(
                    f'Turn 1 timeout (err={err_deg:.1f}deg, near complete). '
                    f'Continuing to transit.')
                self.cone_confirm_count = 0
                self.set_state(STATE_TRANSIT)
                return
            return

        # === STATE: TRANSIT_BETWEEN_ROWS ===
        if self.state == STATE_TRANSIT:
            ang = self.compute_pid_angular()
            self.publish_cmd(TRANSIT_LINEAR_SPEED, ang)
            self.maybe_log(
                f'TRANSIT | t={self.state_elapsed():.1f}s | '
                f'cone d={cone["distance"]:.2f}m | conf={self.cone_confirm_count}')

            if (cone['confirmed'] and
                    cone['distance'] < CONE_APPROACH_DIST_M):
                self.send_alert(
                    f'Cone 2 at {cone["distance"]:.2f}m. Approaching.')
                self.set_state(STATE_CONE_APPROACH_2)
                return

            if self.state_elapsed() > TRANSIT_TIMEOUT_SEC:
                self.send_alert(
                    'Transit timeout: forcing turn 2 without cone.')
                self._begin_turn('TURN 2 (forced, no cone)')
                self.set_state(STATE_TURN_2)
                return
            return

        # === STATE: CONE_APPROACH_2 ===
        if self.state == STATE_CONE_APPROACH_2:
            ang = self.compute_pid_angular()
            self.publish_cmd(LINEAR_SPEED_AVOID, ang)
            self.maybe_log(
                f'APPROACH 2 | cone d={cone["distance"]:.2f}m | '
                f'conf={self.cone_confirm_count}')

            if (cone['confirmed'] and
                    cone['distance'] <= CONE_TURN_TRIGGER_M):
                self._begin_turn('TURN 2 (row entry)')
                self.set_state(STATE_TURN_2)
                return

            if (self.state_elapsed() > CONE_LOST_TIMEOUT_SEC and
                    not cone['confirmed']):
                self.send_alert(
                    'Cone 2 lost during approach. Forcing turn anyway.')
                self._begin_turn('TURN 2 (forced, cone lost)')
                self.set_state(STATE_TURN_2)
                return
            return

        # === STATE: TURN_2 ===
        if self.state == STATE_TURN_2:
            # Pivot in place: linear=0, angular at TURN_ANGULAR_SPEED in
            # the direction of the heading error. Bypass publish_cmd so the
            # MAX_ANGULAR_CMD clamp does not slow the rotation.
            he = self.wrap_angle(self.latest_yaw - self.target_yaw)
            turn_dir = -1.0 if he > 0 else +1.0
            tw = Twist()
            tw.linear.x = 0.0
            tw.angular.z = turn_dir * TURN_ANGULAR_SPEED
            self.cmd_pub.publish(tw)
            self.last_published_linear = 0.0

            err_deg = abs(math.degrees(he))
            self.maybe_log(
                f'TURN 2 PIVOT | err={err_deg:.1f}deg | '
                f'cmd.z={tw.angular.z:+.3f} | t={self.state_elapsed():.1f}s')

            if err_deg < math.degrees(TURN_HEADING_TOLERANCE_RAD):
                self.turn_pair_count += 1
                self.send_alert(
                    f'Turn 2 complete (err={err_deg:.1f}deg). '
                    f'Pair {self.turn_pair_count} done. Settling before hold.')
                self.heading_error_integral = 0.0
                self.last_turn_complete_time = self.get_clock().now()
                self.heading_lock_time = self.get_clock().now()
                self.cone_confirm_count = 0
                self.row_turn_in_progress = False
                self.lateral_offset = 0.0
                self.lateral_setpoint = 0.0
                self.last_dr_time = None
                self.pre_avoid_target_yaw = None
                self.embankment_side = 0
                self.settle_start_forward = self.forward_distance_total
                self.set_state(STATE_POST_TURN_SETTLE)
                return

            if self.state_elapsed() > TURN_TIMEOUT_SEC:
                if err_deg > 30.0:
                    self.send_alert(
                        f'Turn 2 FAILED: only {90.0 - err_deg:.1f}deg achieved. '
                        f'Stopping for safety.')
                    self.set_state(STATE_STOPPED)
                    return
                self.turn_pair_count += 1
                self.send_alert(
                    f'Turn 2 timeout (err={err_deg:.1f}deg). '
                    f'Pair {self.turn_pair_count} done (best effort). Settling.')
                self.last_turn_complete_time = self.get_clock().now()
                self.heading_lock_time = self.get_clock().now()
                self.cone_confirm_count = 0
                self.row_turn_in_progress = False
                self.lateral_offset = 0.0
                self.lateral_setpoint = 0.0
                self.last_dr_time = None
                self.pre_avoid_target_yaw = None
                self.embankment_side = 0
                self.settle_start_forward = self.forward_distance_total
                self.set_state(STATE_POST_TURN_SETTLE)
                return
            return

        # === STATE: POST_TURN_SETTLE ===
        # Drive straight at low speed with angular=0 for a short distance,
        # so the wheels physically straighten and the rover clears the
        # turn region before obstacle avoidance comes back online.
        if self.state == STATE_POST_TURN_SETTLE:
            tw = Twist()
            tw.linear.x = POST_TURN_SETTLE_LINEAR_SPEED
            tw.angular.z = 0.0
            self.cmd_pub.publish(tw)
            self.last_published_linear = POST_TURN_SETTLE_LINEAR_SPEED

            settled_dist = max(
                0.0,
                self.forward_distance_total - self.settle_start_forward)
            self.maybe_log(
                f'SETTLE | dist={settled_dist:.2f}/{POST_TURN_SETTLE_DISTANCE_M:.2f}m '
                f'| t={self.state_elapsed():.1f}s')

            done_by_distance = settled_dist >= POST_TURN_SETTLE_DISTANCE_M
            done_by_timeout  = self.state_elapsed() >= POST_TURN_SETTLE_TIMEOUT_SEC

            if done_by_distance or done_by_timeout:
                why = 'distance' if done_by_distance else 'timeout'
                self.send_alert(
                    f'Post-turn settle complete by {why}. '
                    f'Re-enabling obstacle avoidance.')
                self.heading_error_integral = 0.0
                self.lateral_offset = 0.0
                self.lateral_setpoint = 0.0
                self.last_dr_time = None
                self.set_state(STATE_HEADING_HOLD)
                return
            return

        # =====================================================
        # === OBSTACLE-AVOIDANCE STATES (cropRowAvoid) ===
        # =====================================================

        if self.state == STATE_STOPPED:
            if self.brake_active:
                t = self.get_clock().now().nanoseconds * 1e-9
                if t < self.brake_end_time:
                    self.publish_cmd(BRAKE_LINEAR_X, CMD_STEER_CENTER)
                    self.maybe_log(f'STOPPED brake d={obs["min_dist"]:.2f}m'); return
                self.brake_active = False; self.brake_end_time = None
            self.publish_stop()
            self.maybe_log(f'STOPPED d={obs["min_dist"]:.2f}m')
            if not obs['center_blocked'] and obs['min_dist'] > DETECT_DISTANCE_M:
                self.send_alert('Cleared. Resuming.')
                self._restore_pre_avoid_heading()
                self.set_state(STATE_HEADING_HOLD)
            return

        if self.state == STATE_DRIFT_OUT:
            if self._real_critical(obs):
                self.send_alert('CRITICAL during DRIFT_OUT.')
                self.set_state(STATE_STOPPED); return

            travelled = max(0.0, self.forward_distance_total - self.avoid_start_forward)
            travel_p = self.clamp01(travelled / DRIFT_OUT_DISTANCE_M)
            if math.isfinite(obs['min_dist']):
                denom = max(0.10, DETECT_DISTANCE_M - FULL_EMBANKMENT_BY_DISTANCE_M)
                distance_p = self.clamp01((DETECT_DISTANCE_M - obs['min_dist']) / denom)
            else:
                distance_p = 0.0
            p = max(travel_p, distance_p)
            target = self.embankment_side * EMBANKMENT_LATERAL_SETPOINT_M
            self.lateral_setpoint = self.smoothstep(p) * target

            self.publish_cmd(LINEAR_SPEED_AVOID, self.compute_pid_angular())
            self.maybe_log(
                f'DRIFT_OUT {"L" if self.embankment_side>0 else "R"} '
                f'p={p*100:.0f}% lat={self.lateral_offset*100:+.0f}cm '
                f'set={self.lateral_setpoint*100:+.0f}cm '
                f'd={obs["min_dist"]:.2f}m '
                f'elapsed={self.state_elapsed():.1f}/{DRIFT_OUT_TIMEOUT_SEC:.1f}s')

            travelled_total = max(0.0, self.forward_distance_total - self.avoid_start_forward)
            lat_close = abs(self.lateral_offset - target) <= LATERAL_EMBANK_TOLERANCE_M
            travel_done = travelled_total >= DRIFT_OUT_DISTANCE_M
            if p >= 1.0 and (lat_close or travel_done):
                self.lateral_setpoint = target
                self.set_state(STATE_TRAVERSE)
                return
            if self.state_elapsed() >= DRIFT_OUT_TIMEOUT_SEC:
                self.send_alert(
                    f'DRIFT_OUT timeout after {DRIFT_OUT_TIMEOUT_SEC:.1f}s '
                    f'(lat={self.lateral_offset*100:+.0f}cm, p={p*100:.0f}%). '
                    f'Falling through to DRIFT_BACK.')
                self.set_state(STATE_DRIFT_BACK)
            return

        if self.state == STATE_TRAVERSE:
            if self._real_critical(obs):
                self.send_alert('CRITICAL during TRAVERSE.')
                self.set_state(STATE_STOPPED); return
            if self.embankment_side > 0 and not obs['left_clear'] and \
                    obs['left_min'] < SHOULDER_BLOCK_STOP_DISTANCE_M:
                self.send_alert('Left shoulder critically blocked.')
                self.set_state(STATE_STOPPED); return
            if self.embankment_side < 0 and not obs['right_clear'] and \
                    obs['right_min'] < SHOULDER_BLOCK_STOP_DISTANCE_M:
                self.send_alert('Right shoulder critically blocked.')
                self.set_state(STATE_STOPPED); return

            self.lateral_setpoint = self.embankment_side * EMBANKMENT_LATERAL_SETPOINT_M
            self.publish_cmd(LINEAR_SPEED_AVOID, self.compute_pid_angular())

            if not obs['center_blocked']:
                self.obstacle_clear_count += 1
            else:
                self.obstacle_clear_count = 0

            travelled = max(0.0, self.forward_distance_total - self.traverse_start_forward)
            self.maybe_log(
                f'TRAVERSE {"L" if self.embankment_side>0 else "R"} '
                f'lat={self.lateral_offset*100:+.0f}cm '
                f'trav={travelled:.2f}m clr={self.obstacle_clear_count}/{OBSTACLE_CLEAR_FRAMES}')

            if (travelled >= TRAVERSE_MIN_DISTANCE_M and
                    self.obstacle_clear_count >= OBSTACLE_CLEAR_FRAMES):
                self.send_alert('Passed. Drifting back.')
                self.set_state(STATE_DRIFT_BACK)
            return

        if self.state == STATE_DRIFT_BACK:
            if self._real_critical(obs):
                self.send_alert('CRITICAL during DRIFT_BACK.')
                self.set_state(STATE_STOPPED); return
            travelled = max(0.0, self.forward_distance_total - self.drift_back_start_forward)
            p = self.clamp01(travelled / DRIFT_BACK_DISTANCE_M)
            self.lateral_setpoint = self.drift_back_start_setpoint * (1.0 - self.smoothstep(p))
            self.publish_cmd(LINEAR_SPEED_AVOID, self.compute_pid_angular())
            self.maybe_log(
                f'DRIFT_BACK p={p*100:.0f}% lat={self.lateral_offset*100:+.0f}cm '
                f'set={self.lateral_setpoint*100:+.0f}cm '
                f'elapsed={self.state_elapsed():.1f}/{DRIFT_BACK_TIMEOUT_SEC:.1f}s')
            if p >= 1.0 and abs(self.lateral_offset) <= LATERAL_HOME_TOLERANCE_M:
                self._restore_pre_avoid_heading()
                self.set_state(STATE_HEADING_HOLD); return
            if p >= 1.0:
                self.lateral_setpoint = 0.0
            if self.state_elapsed() >= DRIFT_BACK_TIMEOUT_SEC:
                self.send_alert(
                    f'DRIFT_BACK timeout after {DRIFT_BACK_TIMEOUT_SEC:.1f}s '
                    f'(lat={self.lateral_offset*100:+.0f}cm). Reverting to HEADING_HOLD.')
                self._restore_pre_avoid_heading()
                self.set_state(STATE_HEADING_HOLD); return
            return

        # =====================================================
        # === STATE: HEADING_HOLD (default cruise) ===
        # =====================================================
        ang = self.compute_pid_angular()

        # Note: cone trigger is checked globally at the top of control_loop,
        # so it's not repeated here.

        # Priority 2: corridor obstacle -> embankment bypass (cropRowAvoid).
        if (obs['detected'] and obs['center_blocked'] and
                obs['min_dist'] < DETECT_DISTANCE_M):
            if not obs['left_clear'] and not obs['right_clear']:
                self.send_alert(f'Both shoulders blocked at {obs["min_dist"]:.2f}m. STOP.')
                self.set_state(STATE_STOPPED); return
            self.embankment_side = self._choose_embankment_side(obs)
            self.avoid_start_forward = self.forward_distance_total
            self.lateral_setpoint = 0.0
            self.heading_error_integral = 0.0
            self.pre_avoid_target_yaw = self.target_yaw
            self.send_alert(
                f'Obstacle at {obs["min_dist"]:.2f}m '
                f'(h~{obs["height_estimate"]*100:.0f}cm). '
                f'Drift {"L" if self.embankment_side>0 else "R"}. '
                f'Saved yaw={self.pre_avoid_target_yaw:+.4f}.')
            self.set_state(STATE_DRIFT_OUT); return

        # Priority 3: emergency stop floor.
        if self._real_critical(obs):
            self.send_alert(f'EMERGENCY at {obs["min_dist"]:.2f}m.')
            self.set_state(STATE_STOPPED); return

        # Default: cruise on heading.
        speed = self.get_current_speed()
        self.publish_cmd(speed, ang)

        he = self.wrap_angle(self.latest_yaw - self.target_yaw)
        cd = ' COOLDOWN' if self.in_post_turn_cooldown() else ''
        self.maybe_log(
            f'HOLD{cd} ye={he:+.4f} cmd.z={ang:+.3f} spd={speed:.2f} '
            f'lat={self.lateral_offset*100:+.1f}cm '
            f'obs={obs["min_dist"]:.2f}m h={obs["height_estimate"]*100:.0f}cm '
            f'cone={cone["distance"]:.2f}m({self.cone_confirm_count})')


def main(args=None):
    rclpy.init(args=args)
    node = ImuHeadingHoldObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Exit.')
    finally:
        try:
            if rclpy.ok(): node.publish_stop()
        except Exception: pass
        node.stop_video_recording()
        try: node.destroy_node()
        except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()