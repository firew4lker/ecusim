# SPDX-License-Identifier: GPL-3.0-or-later
"""Minimal generic OBD-II tester for the bench (or against the simulator without hardware).

    ecusim tester                                            # udp_multicast, summary
    ecusim tester --interface socketcan --channel can1 --live 0C,05,0B,10,0E,06
    ecusim tester --interface socketcan --channel can1 --clear
"""

from __future__ import annotations

import argparse
import sys
import time

from .obd.can_transport import CanConfig
from .obd.client import NegativeResponse, ObdClient, ObdTimeout
from .obd.pids import MODE01, SUPPORT_PIDS

SUMMARY = [0x0C, 0x05, 0x0F, 0x0B, 0x10, 0x04, 0x0E, 0x06, 0x07, 0x03, 0x44, 0x34, 0x15, 0x42, 0x3C, 0x1F]


def fmt(pid: int, v: object) -> str:
    p = MODE01[pid]
    if isinstance(v, float):
        return f"{v:.2f} {p.unit}"
    return f"{v} {p.unit}".strip()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="ecusim tester", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--interface", default="udp_multicast")
    ap.add_argument("--channel", default="239.74.163.2")
    ap.add_argument("--bitrate", type=int, default=500_000)
    ap.add_argument("--tty-baudrate", type=int, default=None, help="serial speed for slcan on ttyUSB/COM ports")
    ap.add_argument("--29bit", dest="extended", action="store_true")
    ap.add_argument("--live", help="comma-separated hex PIDs to poll continuously")
    ap.add_argument("--rate", type=float, default=5.0, help="live polls per second")
    ap.add_argument("--clear", action="store_true", help="send Mode 04 (clear DTCs) and exit")
    args = ap.parse_args(argv)

    cfg = CanConfig(args.interface, args.channel, args.bitrate, args.extended, tty_baudrate=args.tty_baudrate)
    try:
        client = ObdClient(cfg)
    except Exception as e:  # python-can raises different types per backend
        print(f"cannot open CAN interface {cfg.interface}:{cfg.channel}: {e}", file=sys.stderr)
        return 2
    with client as c:
        try:
            if args.live:
                pids = [int(x, 16) for x in args.live.split(",")]
                while True:
                    vals = {}
                    for i in range(0, len(pids), 6):
                        vals.update(c.read_pids(pids[i : i + 6]))
                    print("  ".join(f"{p:02X}={fmt(p, vals[p])}" for p in pids if p in vals), flush=True)
                    time.sleep(1.0 / args.rate)
            if args.clear:
                c.clear_dtcs()
                print("Mode 04 accepted: DTCs cleared")
                return 0
            supported = c.supported_pids()
            print("Supported Mode 01 PIDs:", " ".join(f"{p:02X}" for p in sorted(supported - set(SUPPORT_PIDS))))
            print("VIN:     ", c.vin())
            print("ECU name:", c.ecu_name())
            print("CAL ID:  ", c.vehicle_info(0x04).rstrip(b"\x00").decode())
            print("CVN:     ", c.vehicle_info(0x06).hex().upper())
            print("Stored DTCs (03):   ", " ".join(c.dtcs(0x03)) or "none")
            print("Pending DTCs (07):  ", " ".join(c.dtcs(0x07)) or "none")
            print("Permanent DTCs (0A):", " ".join(c.dtcs(0x0A)) or "none")
            print("Readiness:", ", ".join(f"{k}={v}" for k, v in c.readiness().items()))
            for p in SUMMARY:
                if p in supported:
                    print(f"  {p:02X} {MODE01[p].name:40s} {fmt(p, c.read_pid(p))}")
        except ObdTimeout as e:
            print(f"timeout: {e} (is the ECU powered and on the same bus?)", file=sys.stderr)
            return 1
        except NegativeResponse as e:
            print(e, file=sys.stderr)
            return 1
        except KeyboardInterrupt:
            pass
        except BrokenPipeError:  # output piped into head/less that exited
            import os

            sys.stdout = open(os.devnull, "w")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
