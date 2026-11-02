from ultralytics import YOLO

model = YOLO("yolo11n.pt")
results = model("foto2.png", save=True)
print("[+]  concluido, roxo! Abraço")