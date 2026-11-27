# SPDX-License-Identifier: GPL-3.0-or-later
"""Headless ECU simulator on a CAN bus, with a small command console (the Qt UI arrives in M3).

    python -m ecusim --interface socketcan --channel vcan0
    python -m ecusim --interface slcan --channel /dev/ttyACM0 --bitrate 500000 --start
    python -m ecusim --interface udp_multicast --channel 239.74.163.2 --start   # no hardware
"""

from __future__ import annotations

import argparse
import sys
import time

from .core.signals import KEY_OFF, KEY_ON
from .obd.can_transport import CanConfig, EcuCanServer
from .obd.server import ObdServer, ServerConfig
from .obd.vehicle_info import VehicleInfo, vin_is_valid
from .persistence import StateStore, capture
from .runtime import RealtimeRunner
from .sim import Simulator

HELP = """commands:
  on | off            key on / key off
  start               key on and crank until the engine runs
  pedal <0-100>       accelerator pedal %
  status              print live values
  bus                 print CAN statistics and the last frames
  speed <x>           time acceleration (1 = real time)
  quit"""


def _parse_padding(s: str) -> int | None:
    return None if s.lower() == "none" else int(s, 16)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m ecusim", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--interface", default="udp_multicast", help="python-can interface (default udp_multicast)")
    ap.add_argument("--channel", default="239.74.163.2", help="python-can channel")
    ap.add_argument("--bitrate", type=int, default=500_000)
    ap.add_argument("--tty-baudrate", type=int, default=None,
                    help="serial speed for slcan adapters on ttyUSB/COM ports (default 115200)")
    ap.add_argument("--29bit", dest="extended", action="store_true", help="use 29-bit identifiers")
    ap.add_argument("--padding", default="AA", help="frame padding byte in hex, or 'none'")
    ap.add_argument("--ambient", type=float, default=15.0, help="ambient/soak temperature °C")
    ap.add_argument("--baro", type=float, default=101.3, help="barometric pressure kPa")
    ap.add_argument("--vin", default=None)
    ap.add_argument("--region", choices=["eu", "us", "world"], default="eu")
    ap.add_argument("--start", action="store_true", help="start the engine immediately")
    ap.add_argument("--time-scale", type=float, default=1.0)
    ap.add_argument("--no-console", action="store_true", help="run without the interactive console")
    ap.add_argument("--state-dir", default=None, help="load/save ECU keep-alive memory here")
    args = ap.parse_args(argv)

    info = VehicleInfo(region=args.region) if args.vin is None else VehicleInfo(vin=args.vin, region=args.region)
    if not vin_is_valid(info.vin):
        print(f"warning: VIN {info.vin} has an invalid check digit", file=sys.stderr)

    store = StateStore(args.state_dir) if args.state_dir else None
    kam, nvm = store.load() if store else (None, None)
    if kam or nvm:
        sim = StateStore.build_sim(kam, nvm, args.ambient)
        if args.vin is None:
            info = StateStore.vehicle_info(nvm)
    else:
        sim = Simulator(ambient_c=args.ambient, baro_kpa=args.baro)
    runner = RealtimeRunner(sim, time_scale=args.time_scale)
    obd = ObdServer(runner.diag_source, info, ServerConfig(),
                    on_clear=lambda: runner.submit(lambda s: s.ecu.diag.clear()))
    cfg = CanConfig(args.interface, args.channel, args.bitrate, args.extended, _parse_padding(args.padding),
                    tty_baudrate=args.tty_baudrate)
    can_srv = EcuCanServer(cfg, obd.handle, runner.powered)

    runner.start()
    try:
        can_srv.start()
    except Exception as e:  # python-can raises different types per backend
        runner.stop()
        print(f"cannot open CAN interface {cfg.interface}:{cfg.channel}: {e}", file=sys.stderr)
        return 2
    addr = cfg.addr
    print(f"ECU on {cfg.interface}:{cfg.channel} {'29' if cfg.extended else '11'}-bit, "
          f"request 0x{addr['request']:X}/0x{addr['functional']:X}, response 0x{addr['response']:X}")
    print(f"VIN {info.vin}", flush=True)

    def key(k: str) -> None:
        runner.submit(lambda s: setattr(s.driver, "key", k))

    def start_engine() -> None:
        runner.submit(lambda s: s.request_start())

    def status() -> None:
        d = runner.latest
        print(f"t={d['t']:.1f}s rpm={d['rpm']:.0f} ect={d['ect_c']:.1f}°C map={d['map_kpa']:.1f}kPa "
              f"maf={d['maf_gps']:.2f}g/s spark={d['spark_deg']:.1f}° stft={d['stft_pct']:+.1f}% "
              f"ltft={d['ltft_pct']:+.1f}% fuel_status={int(d['fuel_status'])} cat={d['cat_est_c']:.0f}°C")

    if args.start:
        start_engine()
    else:
        key(KEY_ON)

    try:
        if args.no_console:
            while True:
                time.sleep(1.0)
        print(HELP)
        for line in sys.stdin:
            parts = line.split()
            if not parts:
                continue
            cmd = parts[0].lower()
            if cmd in ("quit", "exit", "q"):
                break
            elif cmd == "on":
                key(KEY_ON)
            elif cmd == "off":
                key(KEY_OFF)
            elif cmd == "start":
                start_engine()
            elif cmd == "pedal" and len(parts) == 2:
                v = max(0.0, min(100.0, float(parts[1]))) / 100.0
                runner.submit(lambda s, v=v: setattr(s.driver, "pedal", v))
            elif cmd == "speed" and len(parts) == 2:
                runner.time_scale = max(0.1, float(parts[1]))
            elif cmd == "status":
                status()
            elif cmd == "bus":
                print(can_srv.stats)
                for e in list(can_srv.trace)[-10:]:
                    print(f"{e.t:12.3f} {e.direction} {e.arb_id:X} {e.data.hex(' ')}")
            else:
                print(HELP)
    except KeyboardInterrupt:
        pass
    finally:
        can_srv.stop()
        if store is not None:
            done = []
            runner.submit(lambda s: done.append(capture(s, info)))
            time.sleep(0.1)
            if done:
                store.save(*done[0])
        runner.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
