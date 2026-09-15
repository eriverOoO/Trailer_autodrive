"""Extract image candidates from an on-board video; manual masks still required."""
import argparse
from pathlib import Path
import cv2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--camera', choices=['front', 'rear'], required=True)
    parser.add_argument('--every', type=int, default=15)
    args = parser.parse_args()
    if args.every < 1 or not args.video.is_file():
        parser.error('Need an existing video and positive sampling interval')
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        parser.error('Cannot open video')
    args.output.mkdir(parents=True, exist_ok=True)
    i, saved = 0, 0
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if i % args.every == 0:
                destination = args.output/f'{args.video.stem}_{args.camera}_{i:08d}.jpg'
                if destination.exists():
                    raise FileExistsError(destination)
                if not cv2.imwrite(str(destination), frame):
                    raise OSError(destination)
                saved += 1
            i += 1
    finally:
        capture.release()
    print(f'{saved} image candidates; label polygons before training')


if __name__ == '__main__':
    main()
