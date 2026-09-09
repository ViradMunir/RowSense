"""
RowSense — YOLOv26 training entry point.

Trains the under-canopy wheat detector (wheat heads, tillers, leaf stripe
rust) on the augmented Roboflow dataset.

Hyperparameters below match Table 6 of the project report, which
documents the run that produced the reported results:
mAP@0.5 = 0.817; stripe rust 97.3 P / 97.3 R, wheat heads 81.8 P /
77.3 R, tillers 60.8 P / 70.2 R.
"""

from ultralytics import YOLO
import torch


def main():
    # Check GPU
    print("CUDA available:", torch.cuda.is_available())

    if torch.cuda.is_available():
        print("GPU:", torch.cuda.get_device_name(0))
        torch.cuda.empty_cache()

    # Load YOLO26m pretrained model.
    # The 'm' variant is the accuracy / inference-latency balance point
    # for deployment on the Raspberry Pi 5.
    model = YOLO("yolo26m.pt")

    # Train
    model.train(
        data="dataset_custom.yaml",

        # Main training settings
        epochs=175,
        imgsz=640,
        batch=8,
        patience=40,

        # Windows settings
        # On Linux, raise to 4-8 for a meaningful data-loading speedup.
        workers=0,

        # GPU
        device=0,

        # Performance
        amp=True,
        cache="disk",

        # Optimisation
        optimizer="AdamW",
        lr0=0.001,
        cos_lr=True,
        close_mosaic=15,

        # Loss gains.
        # Box gain is set high relative to class gain because accurate
        # localisation of small wheat heads matters more downstream than
        # fine class margins.
        # The classification criterion is binary cross-entropy — this is
        # Ultralytics' default (BCEWithLogitsLoss) and has no flag of its own.
        box=7.5,
        cls=1.5,

        # Augmentation, useful for wheat field variation
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        degrees=5,
        translate=0.1,
        scale=0.5,
        fliplr=0.5,
        mosaic=1.0,
        mixup=0.1,

        # Output
        plots=True,
        save=True,
        project="runs_wheat",
        name="yolo26m_wheat_heads_tillers_640"
    )


if __name__ == "__main__":
    main()
