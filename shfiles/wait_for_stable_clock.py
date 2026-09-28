#!/usr/bin/env python3

import os
import sys
import time
from datetime import datetime


MIN_VALID_EPOCH = float(os.environ.get("MIN_VALID_EPOCH", "1735689600"))
STABLE_SECONDS = float(os.environ.get("CLOCK_STABLE_SECONDS", "10"))
STEP_TOLERANCE = float(os.environ.get("CLOCK_STEP_TOLERANCE", "0.5"))
WAIT_TIMEOUT = float(os.environ.get("CLOCK_WAIT_TIMEOUT", "0"))


def main():
    started = time.monotonic()
    stable_since = None
    previous_wall = time.time()
    previous_monotonic = time.monotonic()
    last_report = 0.0

    print(
        "Waiting for a plausible system clock that remains stable for "
        f"{STABLE_SECONDS:.1f} seconds...",
        flush=True,
    )

    while True:
        time.sleep(0.25)
        wall_now = time.time()
        monotonic_now = time.monotonic()
        wall_elapsed = wall_now - previous_wall
        monotonic_elapsed = monotonic_now - previous_monotonic
        clock_step = abs(wall_elapsed - monotonic_elapsed)
        plausible = wall_now >= MIN_VALID_EPOCH

        if plausible and clock_step <= STEP_TOLERANCE:
            if stable_since is None:
                stable_since = monotonic_now
            if monotonic_now - stable_since >= STABLE_SECONDS:
                print(
                    "System clock accepted: "
                    + datetime.fromtimestamp(wall_now).isoformat(timespec="seconds"),
                    flush=True,
                )
                return 0
        else:
            stable_since = None

        if monotonic_now - last_report >= 30.0:
            if not plausible:
                reason = "calendar time is earlier than the configured minimum"
            else:
                reason = f"clock step detected ({clock_step:.3f} seconds)"
            print(f"Still waiting: {reason}", flush=True)
            last_report = monotonic_now

        if WAIT_TIMEOUT > 0 and monotonic_now - started >= WAIT_TIMEOUT:
            print("Timed out while waiting for a stable system clock", file=sys.stderr)
            return 1

        previous_wall = wall_now
        previous_monotonic = monotonic_now


if __name__ == "__main__":
    sys.exit(main())
