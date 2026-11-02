import cv2 
from ultralytics import YOLO
from collections import Counter

model = YOLO("yolo11n.pt")
cap = cv2.VideoCapture(0)

while True:
    
    ret, frame = cap.read()

    if not ret:
        break

    results = model(frame, verbose=False)

    annoted_frame = results[0].plot()

    classes_detectadas = results[0].boxes.cls.tolist()

    classes_nomes = model.names

    contagem_por_id = Counter(classes_detectadas)

    contagem_final = {classes_nomes[int(cls_id)]: count for cls_id, count in contagem_por_id.items() }

    print(f'Contagem por classe: {contagem_final}')

    cv2.imshow("Webcam Example", annoted_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()