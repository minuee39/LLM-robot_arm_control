from ultralytics import YOLO
import cv2
from pathlib import Path

from vision.color_detector import detect_color

MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "yolov8n.pt"
model = YOLO(str(MODEL_PATH))
cap = cv2.VideoCapture(0)


while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, conf=0.4)
    annotated = frame.copy()

    for box in results[0].boxes:
        cls_id = int(box.cls[0])
        name = model.names[cls_id]
        conf = float(box.conf[0])
        x1, y1, x2, y2 = map(int, box.xyxy[0])

        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        crop = frame[y1:y2, x1:x2]
        color = detect_color(crop)

        label = f"{color} {name} {conf:.2f} ({cx},{cy})"

        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)

        cv2.putText(
            annotated,
            label,
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )

    cv2.imshow("YOLO Color Center", annotated)

    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
