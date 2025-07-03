import cv2

def start_camera_stream():
    cap = cv2.VideoCapture(0)  # Change index if multiple cams or use CAP_V4L2 for PiCam

    if not cap.isOpened():
        print("❌ Error: Cannot open camera")
        return

    print("✅ Camera stream started. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to grab frame")
            break

        cv2.imshow('Raspberry Pi Camera Stream', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def main():
    start_camera_stream()

if __name__ == "__main__":
    main()
