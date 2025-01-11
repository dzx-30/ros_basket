from ultralytics import YOLO
import onnx

model = YOLO("../workspace/best_seg.pt")

success = model.export(format="onnx", dynamic=True, simplify=True)
