#!/usr/bin/env python3
"""Copy freshly-built firmware binaries into tools/flash/bin/.

Run this after every build to keep the committed binaries up to date:
    python tools/flash/package.py

Or build and refresh in one shot with the idf.ps1 wrapper:
    .\\tools\\idf.ps1 build -rb
"""
import json
import shutil
import sys
from pathlib import Path

FLASH_DIR = Path(__file__).parent
BIN_DIR   = FLASH_DIR / "bin"
REPO_ROOT = FLASH_DIR.parent.parent
BUILD_DIR = REPO_ROOT / "build"

# Binaries to copy: (path relative to BUILD_DIR, destination relative to BIN_DIR)
BINARIES = [
    ("bootloader/bootloader.bin",        "bootloader/bootloader.bin"),
    ("partition_table/partition-table.bin", "partition_table/partition-table.bin"),
    ("mlx90900_esp32_s3_demo.bin",        "mlx90900_esp32_s3_demo.bin"),
]


def main():
    missing = [src for src, _ in BINARIES if not (BUILD_DIR / src).exists()]
    if missing:
        print("ERROR: missing build artifacts (run 'idf.py build' first):")
        for m in missing:
            print(f"  {BUILD_DIR / m}")
        sys.exit(1)

    for src_rel, dst_rel in BINARIES:
        src = BUILD_DIR / src_rel
        dst = BIN_DIR / dst_rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        size_kb = src.stat().st_size // 1024
        print(f"  {dst.relative_to(FLASH_DIR)!s:<50} {size_kb} KB")

    _update_flasher_args()
    print(f"\nDone. Commit the updated files in {FLASH_DIR.relative_to(REPO_ROOT)}")


def _update_flasher_args():
    """Copy flasher_args.json and rewrite binary paths to use the bin/ prefix."""
    src = BUILD_DIR / "flasher_args.json"
    if not src.exists():
        print("WARNING: flasher_args.json not found in build dir, skipping.")
        return

    with src.open() as f:
        config = json.load(f)

    # Rewrite all file references to point under bin/
    def _prefix(path: str) -> str:
        return "bin/" + path.lstrip("/")

    config["flash_files"] = {
        offset: _prefix(p) for offset, p in config["flash_files"].items()
    }
    for key in ("bootloader", "app", "partition-table"):
        if key in config:
            config[key]["file"] = _prefix(config[key]["file"])

    dst = FLASH_DIR / "flasher_args.json"
    with dst.open("w") as f:
        json.dump(config, f, indent=4)
    print(f"  {'flasher_args.json':<50} updated")


if __name__ == "__main__":
    main()
