import cv2
import easyocr
from ultralytics import YOLO
from PIL import Image, ImageDraw
import numpy as np
import datetime
import csv

# YOLO 모델 로드
yolo_model = YOLO("/home/jinsa/Documents/Automatic-License-Plate-Recognition-using-YOLOv8-main/license_plate_detector.pt")

# EasyOCR 리더 초기화
reader = easyocr.Reader(['ko'])

# 웹캠 초기화
cap = cv2.VideoCapture("/dev/video3")  # 0은 기본 웹캠을 의미합니다. 다른 카메라를 사용하려면 이 값을 변경하세요.

# OCR 결과를 저장할 리스트
plate_texts = []

while True:
    # 프레임 읽기
    ret, frame = cap.read()
    if not ret:
        break

    # BGR에서 RGB로 변환
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # YOLO 모델을 사용해 번호판 검출
    results = yolo_model(frame_rgb)

    # PIL 이미지로 변환
    image_pil = Image.fromarray(frame_rgb)
    draw = ImageDraw.Draw(image_pil)

    # YOLO 결과를 순회하며 번호판 영역 추출 및 OCR 수행
    for result in results:
        boxes = result.boxes
        for box in boxes:
            # 클래스가 번호판인 경우에만 처리 (클래스 ID는 모델에 따라 다를 수 있음)
            if int(box.cls) == 0:  # 번호판 클래스 ID를 확인하고 수정해야 합니다
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                
                # 번호판 영역 이미지 추출
                license_plate_img = frame_rgb[y_min:y_max, x_min:x_max]
                
                # EasyOCR을 사용해 텍스트 추출
                ocr_result = reader.readtext(license_plate_img)
                
                if ocr_result:
                    plate_text = ocr_result[0][1]  # 첫 번째 검출된 텍스트 사용
                    print(f"Detected license plate: {plate_text}")
                    
                    # 검출된 번호판 영역에 사각형 그리기
                    draw.rectangle([x_min, y_min, x_max, y_max], outline="green", width=2)
                    
                    # OCR 결과와 포착 시간을 리스트에 저장
                    capture_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    plate_texts.append((plate_text, capture_time))

    # PIL 이미지를 OpenCV 이미지로 변환
    frame_with_detections = cv2.cvtColor(np.array(image_pil), cv2.COLOR_RGB2BGR)

    # 결과 프레임 표시
    cv2.imshow('License Plate Detection', frame_with_detections)

    # 'q' 키를 눌러 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원 해제
cap.release()
cv2.destroyAllWindows()

# OCR 결과를 CSV 파일로 저장
with open('detected_license_plates.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["License Plate Text", "Capture Time"])
    writer.writerows(plate_texts)

