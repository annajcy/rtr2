#!/usr/bin/env python3

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main(argv: list[str]) -> int:
    if len(argv) < 2 or len(argv) > 4:
        print(
            f"usage: {Path(argv[0]).name} <frame_dir> [output_mp4] [fps]",
            file=sys.stderr,
        )
        return 1

    frame_dir = Path(argv[1])
    output_path = Path(argv[2]) if len(argv) >= 3 else frame_dir / "offline.mp4"
    fps = argv[3] if len(argv) >= 4 else "100"

    cmd = [
        "ffmpeg",
        "-y",
        "-framerate",
        fps,
        "-i",
        str(frame_dir / "frame_%06d.png"),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(output_path),
    ]
    return subprocess.run(cmd, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
