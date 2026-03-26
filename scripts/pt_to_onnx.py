from ultralytics import YOLO

# Load model
model = YOLO("obb_best.pt")

# Export to ONNX
model.export(format="onnx", opset=12, dynamic=False, simplify=True)

print("Export complete")
