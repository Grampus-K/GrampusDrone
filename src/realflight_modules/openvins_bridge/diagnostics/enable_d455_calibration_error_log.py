#!/usr/bin/env python3
"""Expose the suppressed IMU EEPROM read exception in librealsense v2.56.3.

This only edits a verified SDK source file. Build/install remain explicit steps.
"""

import argparse
import hashlib
from pathlib import Path
import sys


ORIGINAL_SHA256 = "a303f2acfd39e7b870db8e7bcddabbd4a407b41d071bffd1ab8ae07adaf8ae04"
SOURCE_PATH = Path("src/ds/ds-calib-parsers.cpp")
OLD_BLOCK = b'''            catch (const std::exception&)
            {
                // in case calibration table errors (invalid table, empty table, or corrupted table), data is invalid and default intrinsic and extrinsic will be used
                LOG_WARNING("IMU Calibration is not available, default intrinsic and extrinsic will be used.");
            }'''
NEW_BLOCK = OLD_BLOCK.replace(
    b"catch (const std::exception&)", b"catch (const std::exception& e)"
).replace(
    b'will be used.");', b'will be used. Cause: " << e.what());'
)


def patch_source(sdk_root):
    source = sdk_root.resolve() / SOURCE_PATH
    original_or_patched = source.read_bytes()
    # Normalize only for validation; retain the existing checkout's line endings.
    normalized = original_or_patched.replace(b"\r\n", b"\n")
    if normalized.count(NEW_BLOCK) == 1:
        restored = normalized.replace(NEW_BLOCK, OLD_BLOCK, 1)
        if hashlib.sha256(restored).hexdigest() == ORIGINAL_SHA256:
            print("Already patched: {}".format(source))
            return
    if hashlib.sha256(normalized).hexdigest() != ORIGINAL_SHA256:
        raise ValueError("Unexpected SDK source; refusing to edit {}".format(source))
    if normalized.count(OLD_BLOCK) != 1:
        raise ValueError("Expected exactly one calibration exception handler")

    newline = b"\r\n" if b"\r\n" in original_or_patched else b"\n"
    patched = normalized.replace(OLD_BLOCK, NEW_BLOCK, 1).replace(b"\n", newline)
    backup = source.with_name(source.name + ".grampus-original")
    try:
        with backup.open("xb") as handle:
            handle.write(original_or_patched)
    except FileExistsError:
        if backup.read_bytes() != original_or_patched:
            raise ValueError("Existing backup differs; refusing to overwrite {}".format(backup))
    source.write_bytes(patched)
    print("Patched logging only: {}".format(source))
    print("Original backup: {}".format(backup))
    print("Next: rebuild and install this SDK's existing build directory.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sdk_root", type=Path, help="librealsense v2.56.3 source directory")
    args = parser.parse_args()
    try:
        patch_source(args.sdk_root)
    except (OSError, ValueError) as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
