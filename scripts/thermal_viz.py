import cv2
import argparse

PALETTES = {
    "ironbow": cv2.COLORMAP_INFERNO,
    "rainbow": cv2.COLORMAP_TURBO,
}


def apply_thermal_palette(frame, palette):
    if palette == "whitehot":
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    if palette == "blackhot":
        inverted = cv2.bitwise_not(frame)
        return cv2.cvtColor(inverted, cv2.COLOR_GRAY2BGR)

    return cv2.applyColorMap(frame, PALETTES[palette])


def draw_overlay(frame, fps, width, height):
    h, w = frame.shape[:2]

    # Only display overlay when image is large enough
    if w < 320 or h < 240:
        return frame

    text = f"32x24 -> {width}x{height} | {fps:.1f} FPS"

    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.6
    thickness = 2

    (text_w, text_h), baseline = cv2.getTextSize(
        text, font, scale, thickness
    )

    x, y = 12, 12 + text_h

    overlay = frame.copy()

    cv2.rectangle(
        overlay,
        (5, 5),
        (text_w + 22, text_h + baseline + 18),
        (0, 0, 0),
        -1
    )

    cv2.addWeighted(
        overlay, 0.55,
        frame, 0.45,
        0, frame
    )

    cv2.putText(
        frame,
        text,
        (x, y),
        font,
        scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_AA
    )

    return frame


def main():
    parser = argparse.ArgumentParser(
        description="Display an upscaled thermal camera feed."
    )

    parser.add_argument(
        "device",
        nargs="?",
        default="/dev/video4",
        help="Video device (default: /dev/video4)"
    )

    parser.add_argument(
        "upscale",
        nargs="?",
        type=int,
        default=20,
        help="Frame upscale factor (default: 20)"
    )

    parser.add_argument(
        "--palette",
        choices=["ironbow", "whitehot", "blackhot", "rainbow"],
        default="ironbow",
        help="Thermal color palette (default: ironbow)"
    )

    args = parser.parse_args()

    if args.upscale < 1:
        parser.error("Upscale factor must be at least 1")

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)

    if not cap.isOpened():
        raise RuntimeError(
            f"Cannot open thermal camera: {args.device}"
        )

    print(f"Thermal camera: {args.device}")
    print(f"Upscale: {args.upscale}x")
    print(f"Palette: {args.palette}")
    print("Press Q to quit.")

    fps = 0.0
    frame_count = 0
    fps_start = cv2.getTickCount()

    try:
        while True:
            ret, frame = cap.read()

            if not ret:
                print("Failed to grab frame")
                break

            if len(frame.shape) == 3:
                gray = cv2.cvtColor(
                    frame,
                    cv2.COLOR_BGR2GRAY
                )
            else:
                gray = frame

            thermal = apply_thermal_palette(
                gray,
                args.palette
            )

            height, width = thermal.shape[:2]

            upscaled = cv2.resize(
                thermal,
                (width * args.upscale, height * args.upscale),
                interpolation=cv2.INTER_CUBIC
            )

            frame_count += 1

            current_tick = cv2.getTickCount()

            elapsed = (
                current_tick - fps_start
            ) / cv2.getTickFrequency()

            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                fps_start = current_tick

            upscaled = draw_overlay(
                upscaled,
                fps,
                width * args.upscale,
                height * args.upscale
            )

            cv2.imshow(
                f"Thermal Camera - {args.palette}",
                upscaled
            )

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
