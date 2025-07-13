import json
import csv
import time
import math
import os
import shutil
import sys
from ultralytics import YOLO
from pathlib import Path
import cv2
import numpy as np


class YOLOWebcamProcessor:
    def __init__(self, model, output_dir):
        self.model = model
        self.output_dir = output_dir
        self.csv_output = []
        self.confidences = []
        self.max_object_count = 0
        self.classNames = model.names  # Use model-provided class names
        print(self.classNames) 
        self.should_shutdown = False

    def run(self):
        cap = cv2.VideoCapture(6)
        if not cap.isOpened():
            print("Failed to open webcam.")
            return

        while not self.should_shutdown:
            ret, img = cap.read()
            if not ret:
                continue

            results = self.model(img, conf=0.80, stream=True)
            print("Inference done on device:", self.model.device)

            object_count = 0
            fontScale = 1

            for r in results:
                if not hasattr(r, "obb") or r.obb is None or r.obb.xywhr is None:
                    print("No OBB Results Found!")
                    continue

                object_count = 0
                fontScale = 1

                xywhr = r.obb.xywhr.cpu().numpy()
                confs = r.obb.conf.cpu().numpy()
                clss = r.obb.cls.cpu().numpy()

                for i in range(xywhr.shape[0]):
                    x, y, w, h, angle = xywhr[i]
                    confidence = float(confs[i])
                    cls = int(clss[i])
                    label = self.classNames.get(cls, f"class_{cls}")

                    # 회전 박스 → 사각형 좌표
                    rect = ((x, y), (w, h), angle * 180 / math.pi)
                    points = np.int0(cv2.boxPoints(rect))
                    cv2.drawContours(img, [points], 0, (0, 0, 255), 2)

                    cv2.putText(img, f"{label}: {confidence:.2f}", (int(x), int(y) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                    self.csv_output.append([x, y, w, h, angle, confidence, label])
                    self.confidences.append(confidence)
                    object_count += 1


            self.max_object_count = max(self.max_object_count, object_count)
            cv2.putText(img, f"Objects_count: {object_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 1)

            if object_count > 0:
                filename = f'output_{int(time.time())}.jpg'
                cv2.imwrite(os.path.join(self.output_dir, filename), img)

            # display_img = cv2.resize(img, (img.shape[1]*2, img.shape[0]*2))
            cv2.imshow("Detection", img)

            key = cv2.waitKey(10)
            if key == ord('q'):
                print("Shutting down...")
                self.should_shutdown = True

        cap.release()
        cv2.destroyAllWindows()

    def save_output(self):
        with open(os.path.join(self.output_dir, 'output.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            # writer.writerow(['X1', 'Y1', 'X2', 'Y2', 'Confidence', 'Class'])
            writer.writerow(['X_center', 'Y_center', 'Width', 'Height', 'Angle(deg)', 'Confidence', 'Class'])
            writer.writerows(self.csv_output)

        with open(os.path.join(self.output_dir, 'output.json'), 'w') as f:
            json.dump(self.csv_output, f)

        with open(os.path.join(self.output_dir, 'statistics.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Max Object Count', 'Average Confidence'])
            avg_conf = sum(self.confidences)/len(self.confidences) if self.confidences else 0
            writer.writerow([self.max_object_count, avg_conf])



def main():
    # model_path = input("Enter path to model file (.pt, .engine, .onnx): ").strip()
    model_path = "/home/rokey/choi_ws/src/fitomi/resource/final.pt"

    if not os.path.exists(model_path):
        print(f"File not found: {model_path}")
        exit(1)

    suffix = Path(model_path).suffix.lower()

    if suffix == '.pt':
        model = YOLO(model_path)
    elif suffix in ['.onnx', '.engine']:
        model = YOLO(model_path, task='detect')
    else:
        print(f"Unsupported model format: {suffix}")
        exit(1)

    output_dir = './output'
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.mkdir(output_dir)

    processor = YOLOWebcamProcessor(model, output_dir)
    processor.run()
    # processor.save_output()
    print("Shutdown complete.")
    sys.exit(0)


if __name__ == '__main__':
    main()
