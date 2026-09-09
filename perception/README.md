# Perception

Training and inference scripts for the RowSense YOLO detector.

**Status:** the final training and prediction scripts are being recovered
and will be added here. Model configuration for the reported run is
documented in the main [README](../README.md#training):
`yolo26m`, 175 epochs, 640 px, batch 8, AdamW (lr 0.001), BCE loss,
box/class loss gains 7.5/1.5, early-stopping patience 40.

Expected contents:
- `train.py` — training entry point
- `predict.py` — batch inference on a test folder
- `dataset_custom.yaml` — dataset paths and class names