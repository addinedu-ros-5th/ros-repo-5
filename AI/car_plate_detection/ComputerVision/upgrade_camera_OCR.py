import cv2
import easyocr
from ultralytics import YOLO
from PIL import Image, ImageDraw
import numpy as np
import datetime
import csv
import os
import re

# Define the directory to save the CSV file
save_dir = "/mnt/workspace"  # This should match the container path used in the Docker run command

os.makedirs(save_dir, exist_ok=True)

csv_file_path = os.path.join(save_dir, "detected_license_plates.csv")

yolo_model = YOLO("AI/car_plate_detection/Automatic-License-Plate-Recognition-using-YOLOv8-main/license_plate_detector.pt")

reader = easyocr.Reader(['ko'])

cap = cv2.VideoCapture("/dev/video0")  

if not cap.isOpened():
    print("Error: Camera not accessible")
    exit()

print("Camera initialized successfully")

plate_texts = []

# Track the last detected license plate texts and their counts
detected_text_counts = {}

# Identify license plate text containing Korean characters
def contains_korean_format(text):
    pattern = re.compile(r'^\d{2,3}[가-힣]\d{4}$')
    return bool(pattern.match(text))

while True:

    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame (stream end?). Exiting ...")
        break
    print("Frame captured successfully")


    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


    results = yolo_model(frame_rgb)
    print(f"YOLO results: {results}")


    image_pil = Image.fromarray(frame_rgb)
    draw = ImageDraw.Draw(image_pil)

    
    for result in results:
        boxes = result.boxes
        print(f"Detected {len(boxes)} boxes")
        for box in boxes:
            
            if int(box.cls) == 0:  
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                print(f"Detected plate region: {x_min, y_min, x_max, y_max}")

                # plate image extraction
                license_plate_img = frame_rgb[y_min:y_max, x_min:x_max]

                # EasyOCR
                ocr_result = reader.readtext(license_plate_img)

                if ocr_result:
                    plate_text = ocr_result[0][1]  
                    print(f"Detected license plate: {plate_text}")

                
                    draw.rectangle([x_min, y_min, x_max, y_max], outline="green", width=2)

                    
                    if plate_text in detected_text_counts:
                        detected_text_counts[plate_text] += 1
                    else:
                        detected_text_counts[plate_text] = 1

                    # Save text detected more than 5 times
                    if detected_text_counts[plate_text] == 5:
                        capture_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        plate_texts.append((plate_text, capture_time))
                        # To avoid saving the same text multiple times after reaching 5 detections
                        detected_text_counts[plate_text] = 0
                else:
                    print("No text detected by OCR")
            
    
    
    frame_with_detections = cv2.cvtColor(np.array(image_pil), cv2.COLOR_RGB2BGR)

    cv2.imshow('License Plate Detection', frame_with_detections)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()


with open(csv_file_path, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["License Plate Text", "Capture Time"])
    writer.writerows(plate_texts)

# print("Detected license plates:")
# for plate_text, capture_time in plate_texts:
#     print(f"{capture_time}: {plate_text}")

print("Program finished.")
