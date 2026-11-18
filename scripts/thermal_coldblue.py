import cv2

DEVICE = "/dev/video4"
UPSCALE = 40

cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)

if not cap.isOpened():
    raise RuntimeError(f"Cannot open thermal camera: {DEVICE}")

print("Thermal camera started. Press Q to quit.")

while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame")
        break

    thermal = cv2.applyColorMap(
        frame, cv2.COLORMAP_JET
    )

    height, width = thermal.shape[:2]

    upscaled = cv2.resize(
        thermal,
        (width * UPSCALE, height * UPSCALE),
        interpolation=cv2.INTER_CUBIC
    )

    cv2.imshow("Thermal Camera - 20x Upscaled", upscaled)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
