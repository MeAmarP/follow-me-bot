import cv2
import time

def start_camera_stream(output_path='captured_image.jpg', buffer_time=2):
    cap = cv2.VideoCapture(0)  # Change index if multiple cams or use CAP_V4L2 for PiCam

    if not cap.isOpened():
        print("❌ Error: Cannot open camera")
        return

    print(f"✅ Camera stream started. Waiting {buffer_time} seconds for camera to stabilize...")
    time.sleep(buffer_time)

    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to grab frame")
    else:
        cv2.imwrite(output_path, frame)
        print(f"✅ Image saved to {output_path}")

    cap.release()
    cv2.destroyAllWindows()

def main():
    start_camera_stream()

if __name__ == "__main__":
    main()
