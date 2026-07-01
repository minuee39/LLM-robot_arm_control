from ultralytics import YOLO
import cv2
import numpy as np
from pathlib import Path

MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "yolov8n.pt"
model = YOLO(str(MODEL_PATH))
cap = cv2.VideoCapture(0)


def detect_color(crop):
    """
    YOLO가 검출한 객체 영역 crop에서 대표 색상을 판단한다.
    HSV 색공간 기준으로 빨강, 파랑, 초록, 노랑, 검정, 흰색 정도를 구분한다.
    """

    if crop.size == 0:
        return "unknown"

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

    color_ranges = {
        "red": [
            ((0, 70, 50), (10, 255, 255)),
            ((170, 70, 50), (180, 255, 255)),
        ],
        "blue": [
            ((100, 70, 50), (130, 255, 255)),
        ],
        "green": [
            ((40, 70, 50), (85, 255, 255)),
        ],
        "yellow": [
            ((20, 70, 50), (35, 255, 255)),
        ],
        "black": [
            ((0, 0, 0), (180, 255, 50)),
        ],
        "white": [
            ((0, 0, 200), (180, 40, 255)),
        ],
    }

    scores = {}

    for color_name, ranges in color_ranges.items():
        mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for lower, upper in ranges:
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            mask_total = cv2.bitwise_or(mask_total, mask)

        scores[color_name] = cv2.countNonZero(mask_total)

    best_color = max(scores, key=scores.get)

    if scores[best_color] < 50:
        return "unknown"

    return best_color


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
