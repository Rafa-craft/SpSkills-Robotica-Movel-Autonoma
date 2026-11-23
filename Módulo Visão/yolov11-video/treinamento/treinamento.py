from ultralytics import YOLO
import multiprocessing

if __name__ == "__main__":
    multiprocessing.freeze_support()
    model = YOLO("yolov8n.pt")

    results = model.train(
        data="data.yaml",
        epochs=50,
        imgsz=640,
        batch=4,
        workers=0,
        patience=10,
        device="cpu"
    )

    print("[+] treinamento concluido")