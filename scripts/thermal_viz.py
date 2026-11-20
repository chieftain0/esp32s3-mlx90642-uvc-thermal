
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
        help="Frame upscale factor (default: 40)"
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

    try:
        while True:
            ret, frame = cap.read()

            if not ret:
                print("Failed to grab frame")
                break

            if len(frame.shape) == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame

            thermal = apply_thermal_palette(gray, args.palette)

            height, width = thermal.shape[:2]

            upscaled = cv2.resize(
                thermal,
                (width * args.upscale, height * args.upscale),
                interpolation=cv2.INTER_CUBIC
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
