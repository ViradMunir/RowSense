"""
RowSense — YOLOv26 inference and training-curve plotting.

Two jobs:

  1. Run the trained detector over a folder of test images and save the
     annotated results.
  2. Read the training run's results.csv and emit one figure per metric.
     Where a metric has both a train and a validation series (the box,
     classification and DFL losses), both are drawn on the same axes so
     the generalisation gap is visible at a glance.

Ultralytics' own results.png packs everything into a single grid with
train and validation losses in separate panels, which makes the gap hard
to read and the panels too small to drop into a report. This produces
standalone, report-ready figures instead.

Usage:
    python predict.py                          # inference + plots, using defaults below
    python predict.py --no-predict             # plots only
    python predict.py --no-plots               # inference only
    python predict.py --run-dir runs_wheat/my_run
"""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")            # no display on a headless training box
import matplotlib.pyplot as plt
import pandas as pd
from ultralytics import YOLO


# ============================================================
# DEFAULTS — edit these, or override on the command line
# ============================================================
RUN_DIR = Path("runs_wheat/yolo26m_wheat_heads_tillers_640")
WEIGHTS = RUN_DIR / "weights" / "best.pt"
SOURCE = Path("test")            # folder of images to run inference on
PLOT_DIR = RUN_DIR / "figures"   # where the generated figures land
DEVICE = "cuda"

# Loss pairs: each is drawn as one figure with both series overlaid.
LOSS_PAIRS = [
    ("box_loss", "Box Loss"),
    ("cls_loss", "Classification Loss"),
    ("dfl_loss", "DFL Loss"),
]

# Validation-only metrics: one single-series figure each.
VAL_METRICS = [
    ("metrics/precision(B)", "Precision", "Precision"),
    ("metrics/recall(B)", "Recall", "Recall"),
    ("metrics/mAP50(B)", "mAP@0.5", "mAP@0.5"),
    ("metrics/mAP50-95(B)", "mAP@0.5:0.95", "mAP@0.5:0.95"),
]

# Match the styling used in the project report.
TRAIN_STYLE = dict(color="#1f77b4", linestyle="-", linewidth=1.6, label="Train")
VAL_STYLE = dict(color="#ff7f0e", linestyle="--", linewidth=1.6, label="Validation")


def run_inference(weights: Path, source: Path, device: str) -> None:
    """Run the detector over `source` and save annotated images."""
    if not weights.exists():
        raise FileNotFoundError(f"Weights not found: {weights}")
    if not source.exists():
        raise FileNotFoundError(f"Inference source not found: {source}")

    print(f"Loading weights: {weights}")
    model = YOLO(str(weights))

    print(f"Running inference on: {source}")
    model.predict(source=str(source), save=True, device=device)
    print("Inference complete — annotated images saved under runs/detect/")


def load_results(run_dir: Path) -> pd.DataFrame:
    """Load results.csv, normalising the column names.

    Ultralytics has shipped results.csv with leading spaces in the header
    across several versions, so strip before doing anything else.
    """
    csv_path = run_dir / "results.csv"
    if not csv_path.exists():
        raise FileNotFoundError(
            f"No results.csv in {run_dir}. Point --run-dir at a completed "
            f"training run."
        )

    df = pd.read_csv(csv_path)
    df.columns = [c.strip() for c in df.columns]
    print(f"Loaded {len(df)} epochs from {csv_path}")
    return df


def save_figure(fig, out_dir: Path, filename: str) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / filename
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {path}")


def plot_loss_pair(df: pd.DataFrame, key: str, title: str, out_dir: Path) -> None:
    """Plot train and validation series for one loss on shared axes."""
    train_col, val_col = f"train/{key}", f"val/{key}"

    if train_col not in df.columns and val_col not in df.columns:
        print(f"  skipping {title}: neither series present in results.csv")
        return

    epochs = df["epoch"] if "epoch" in df.columns else range(1, len(df) + 1)

    fig, ax = plt.subplots(figsize=(7, 4.5))
    if train_col in df.columns:
        ax.plot(epochs, df[train_col], **TRAIN_STYLE)
    if val_col in df.columns:
        ax.plot(epochs, df[val_col], **VAL_STYLE)

    ax.set_title(f"{title} (Train vs Validation)")
    ax.set_xlabel("Epoch")
    ax.set_ylabel(title)
    ax.grid(True, alpha=0.3)
    ax.legend()

    save_figure(fig, out_dir, f"{key}_train_vs_val.png")


def plot_val_metric(df: pd.DataFrame, col: str, title: str,
                    ylabel: str, out_dir: Path) -> None:
    """Plot a single validation-only metric."""
    if col not in df.columns:
        print(f"  skipping {title}: column not in results.csv")
        return

    epochs = df["epoch"] if "epoch" in df.columns else range(1, len(df) + 1)

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(epochs, df[col], color="#2ca02c", linewidth=1.6, label=title)

    ax.set_title(f"{title} (Validation)")
    ax.set_xlabel("Epoch")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend()

    # Metrics are all in [0, 1]; a fixed axis keeps figures comparable.
    ax.set_ylim(0, 1)

    filename = title.lower().replace("@", "").replace(":", "_").replace(" ", "_")
    save_figure(fig, out_dir, f"{filename}.png")


def plot_all(run_dir: Path, out_dir: Path) -> None:
    df = load_results(run_dir)

    print("Plotting losses (train vs validation):")
    for key, title in LOSS_PAIRS:
        plot_loss_pair(df, key, title, out_dir)

    print("Plotting validation metrics:")
    for col, title, ylabel in VAL_METRICS:
        plot_val_metric(df, col, title, ylabel, out_dir)

    print(f"Done — figures in {out_dir}")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--run-dir", type=Path, default=RUN_DIR,
                   help="Training run directory containing results.csv and weights/")
    p.add_argument("--weights", type=Path, default=None,
                   help="Model weights (default: <run-dir>/weights/best.pt)")
    p.add_argument("--source", type=Path, default=SOURCE,
                   help="Folder of images to run inference on")
    p.add_argument("--plot-dir", type=Path, default=None,
                   help="Output directory for figures (default: <run-dir>/figures)")
    p.add_argument("--device", default=DEVICE, help="cuda, cpu, or a device index")
    p.add_argument("--no-predict", action="store_true", help="Skip inference")
    p.add_argument("--no-plots", action="store_true", help="Skip plotting")
    return p.parse_args()


def main():
    args = parse_args()
    weights = args.weights or (args.run_dir / "weights" / "best.pt")
    plot_dir = args.plot_dir or (args.run_dir / "figures")

    if not args.no_predict:
        run_inference(weights, args.source, args.device)

    if not args.no_plots:
        plot_all(args.run_dir, plot_dir)


if __name__ == "__main__":
    main()
