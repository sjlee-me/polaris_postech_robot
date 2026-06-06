from ultralytics import YOLOWorld, YOLOE
from PIL import Image
import sys


def detect_object(image_path: str, target_class: str = "fire extinguisher"):
    # model = YOLOWorld("yolov8x-worldv2.pt")
    model = YOLOE("yoloe-26x-seg.pt")
    model.set_classes([target_class])

    image = Image.open(image_path)
    img_w, img_h = image.size

    results = model.predict(image_path)

    print(f"Image path: {image_path}")
    print(f"Image resolution (W x H): {img_w} x {img_h}")
    print(f"Target class: {target_class}")

    for result in results:
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            print("No objects detected.")
            continue

        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            bbox_w = x2 - x1
            bbox_h = y2 - y1
            conf = float(box.conf[0])

            print(f"\n[Detection {i + 1}]")
            print(f"  BBox size (W x H): {bbox_w:.2f} x {bbox_h:.2f}")
            print(f"  BBox coords (x1, y1, x2, y2): "
                  f"({x1:.2f}, {y1:.2f}, {x2:.2f}, {y2:.2f})")
            print(f"  Confidence: {conf:.4f}")
    
    # save the image with detected bounding boxes for visualization
    annotated_image = results[0].plot()
    # Save the annotated image to a new file
    # annotated_image is numpy.ndarray
    annotated_image_pil = Image.fromarray(annotated_image)
    annotated_image_pil.save("annotated_cone_0.jpg")


if __name__ == "__main__":

    image_path = "/hdd/hdd3/lsj/polaris/MQTT-PROTOCOL-GUIDE/cone_0.jpg"
    target_class = "traffic cone"
    detect_object(image_path, target_class)
