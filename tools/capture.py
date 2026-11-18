"""Configure the board, record a capture, and optionally show a live view.

    py capture.py --port COM5 --label wall_50cm --seconds 20
    py capture.py --port COM5 --view                         # live view, record until closed
    py capture.py --port COM5 --drivers ch,cx --label swap   # each driver on the other board
    py capture.py --port COM5 --cnh 0,32,1,2 --res 8         # finer bins, 2x2-zone aggregates

Writes captures/<timestamp>_<label>.vlog (raw frames) and a .json sidecar.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import queue
import sys
import threading
import time
from pathlib import Path

import numpy as np
import serial

import vlog


def open_port(name: str, baud: int) -> serial.Serial:
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = name, baud, 0.05
    # Deassert both so the DevKit's auto-reset circuit leaves the ESP32 running.
    s.dtr = False
    s.rts = False
    s.open()
    return s


def build_commands(a: argparse.Namespace) -> list[str]:
    cmds = []
    if a.i2c:
        cmds.append(f"i2c {a.i2c}")
    if a.drivers:
        da, db = a.drivers.split(",")
        cmds.append(f"drv {da} {db}")
    if a.res:
        cmds.append(f"res {a.res}")
    if a.hz:
        cmds.append(f"hz {a.hz}")
    if a.cnh:
        cmds.append("cnh off" if a.cnh == "off" else "cnh " + " ".join(a.cnh.split(",")))
    cmds += a.cmd or []
    return cmds


class Reader(threading.Thread):
    """Reads the port, appends raw bytes to the file once recording starts,
    and publishes decoded events."""

    def __init__(self, ser: serial.Serial):
        super().__init__(daemon=True)
        self.ser = ser
        self.parser = vlog.StreamParser()
        self.events: queue.Queue = queue.Queue()
        self.file = None
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.bytes = 0

    def record_to(self, f) -> None:
        with self.lock:
            self.file = f

    def run(self) -> None:
        while not self.stop.is_set():
            data = self.ser.read(65536)
            if not data:
                continue
            with self.lock:
                if self.file:
                    self.file.write(data)
                    self.bytes += len(data)
            for ev in self.parser.feed(data):
                self.events.put(ev)


def handle(ev, state: dict, verbose: bool = True) -> None:
    if ev[0] == "text":
        if verbose:
            print("  |", ev[1])
        return
    _, ftype, payload = ev
    if ftype == vlog.FT_INFO:
        j = json.loads(payload.decode("utf-8", "replace"))
        state["info"][j["port"]] = j
        state["last_info"] = time.time()
        state["infos"].append(j)
        cnh = j["cnh"]
        extra = f" cnh {cnh['aggregates']}x{cnh['bins']} bins" if cnh["active"] else ""
        print(f"  INFO port {j['port']}: {j['event']}, driver {j['loaded']}, ids 0x{j['device_id']:02X}/"
              f"0x{j['revision_id']:02X}, {int(j['zones'] ** 0.5)}x{int(j['zones'] ** 0.5)} @ {j['hz']} Hz"
              f"{extra}{', error: ' + j['last_error'] if j['last_error'] else ''}")
    elif ftype == vlog.FT_LOG:
        msg = payload.decode("utf-8", "replace")
        if msg.startswith("ok out bin"):
            state["bin_ok"] = True
        if verbose:
            print("  #", msg)
    elif ftype == vlog.FT_RANGE:
        d = vlog.decode_range(payload)
        state["range"][d["port"]] = d
        state["count"][d["port"]] = state["count"].get(d["port"], 0) + 1
        state["hist"].setdefault(d["port"], []).append(d)
        del state["hist"][d["port"]][:-200]
    elif ftype == vlog.FT_CNH:
        d = vlog.decode_cnh(payload)
        state["cnh"][d["port"]] = d
        state["cnh_count"][d["port"]] = state["cnh_count"].get(d["port"], 0) + 1


def drain(reader: Reader, state: dict, verbose: bool = True) -> None:
    while True:
        try:
            handle(reader.events.get_nowait(), state, verbose)
        except queue.Empty:
            return


def settle(reader: Reader, state: dict, ser: serial.Serial, cmds: list[str]) -> None:
    """Send commands, then wait until the ports stop reconfiguring."""
    # Opening the port can reset some DevKits; repeat until the firmware answers.
    deadline = time.time() + 8
    while not state.get("bin_ok"):
        if time.time() > deadline:
            raise SystemExit("no answer from the board: wrong port, baud, or firmware not flashed?")
        ser.write(b"\nout bin\n")
        t = time.time()
        while time.time() - t < 0.5 and not state.get("bin_ok"):
            drain(reader, state)
            time.sleep(0.02)
    for c in cmds:
        print(f"> {c}")
        ser.write((c + "\n").encode())
        time.sleep(0.15)
    ser.write(b"status\n")
    deadline = time.time() + 20
    state["last_info"] = time.time()
    while time.time() < deadline:
        drain(reader, state)
        # Firmware downloads take ~1-2 s; done once INFO goes quiet for 2.5 s.
        if time.time() - state["last_info"] > 2.5 and state["info"]:
            return
        time.sleep(0.05)
    print("warning: ports still reconfiguring after 20 s")


def print_rates(state: dict, prev: dict, dt_s: float, reader: Reader) -> None:
    parts = []
    for p in sorted(state["count"]):
        n = state["count"][p] - prev.get(p, 0)
        c = state["cnh_count"].get(p, 0) - prev.get(p + "cnh", 0)
        r = state["range"].get(p, {})
        parts.append(f"{p}[{r.get('driver', '?')}] {n / dt_s:4.1f} fps"
                     + (f" cnh {c / dt_s:4.1f}" if c else "")
                     + f" read {r.get('read_us', 0) / 1000:5.1f} ms")
        prev[p], prev[p + "cnh"] = state["count"][p], state["cnh_count"].get(p, 0)
    print("  " + " | ".join(parts) + f" | crc err {reader.parser.crc_errors} | {reader.bytes / 1e6:.1f} MB")


def live_view(state: dict, reader: Reader, args) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    fig.canvas.manager.set_window_title("VL53L7 live view - click a zone")
    sel = {"zone": 27}
    ims = {}
    for i, p in enumerate("AB"):
        ax = axes[0, i]
        ims[p] = ax.imshow(np.zeros((8, 8)), vmin=0, vmax=args.max_mm, cmap="viridis_r")
        ax.set_xticks([])
        ax.set_yticks([])
        fig.colorbar(ims[p], ax=ax, fraction=0.046, label="mm")
    marker = {p: axes[0, i].plot([], [], "rs", ms=14, mfc="none", mew=2)[0] for i, p in enumerate("AB")}
    lines = {p: axes[1, 0].plot([], [], label=p)[0] for p in "AB"}
    axes[1, 0].set_xlabel("frame")
    axes[1, 0].set_ylabel("selected zone distance, mm")
    axes[1, 0].legend()
    ax_h = axes[1, 1]

    def on_click(e):
        for i, p in enumerate("AB"):
            r = state["range"].get(p)
            if e.inaxes is axes[0, i] and r and e.xdata is not None:
                side = 8 if r["zones"] == 64 else 4
                sel["zone"] = int(round(e.ydata)) * side + int(round(e.xdata))

    fig.canvas.mpl_connect("button_press_event", on_click)

    def update(_):
        drain(reader, state, verbose=False)
        z = sel["zone"]
        for i, p in enumerate("AB"):
            r = state["range"].get(p)
            if not r:
                continue
            side = 8 if r["zones"] == 64 else 4
            zz = min(z, r["zones"] - 1)
            d = r["distance_mm"][:, 0].astype(float)
            ok = (r["nb_target_detected"] > 0) & np.isin(r["target_status"][:, 0], vlog.VALID_STATUS)
            img = np.where(ok, d, np.nan).reshape(side, side)
            if args.flip:
                img = img[::-1, ::-1]
            ims[p].set_data(img)
            ims[p].set_extent((-0.5, side - 0.5, side - 0.5, -0.5))
            axes[0, i].set_title(f"port {p} [{r['driver']}]  {r['temp_c']} C  read {r['read_us'] / 1000:.0f} ms")
            rr, cc = divmod(zz, side)
            if args.flip:
                rr, cc = side - 1 - rr, side - 1 - cc
            marker[p].set_data([cc], [rr])
            hist = state["hist"].get(p, [])
            ys = [h["distance_mm"][min(z, h["zones"] - 1), 0] for h in hist]
            lines[p].set_data(range(len(ys)), ys)
        axes[1, 0].relim()
        axes[1, 0].autoscale_view()
        ax_h.cla()
        for p, c in state["cnh"].items():
            agg = vlog.zone_to_aggregate(min(z, c["zones"] - 1), c["zones"], c["merge"], c["cols"])
            x = vlog.bin_centers_mm(c["start_bin"], c["bins"], c["sub"])
            ax_h.step(x, c["hist"][agg], where="mid", label=f"port {p} CNH, aggregate {agg}")
            ax_h.axhline(c["ambient"][agg], ls=":", c="gray", label="ambient")
            r = state["range"].get(p)
            if r and r["nb_target_detected"][min(z, r["zones"] - 1)]:
                ax_h.axvline(r["distance_mm"][min(z, r["zones"] - 1), 0], c="r", label="ranged distance")
        ax_h.set_xlabel("nominal bin centre, mm (uncalibrated)")
        ax_h.set_title(f"zone {z}")
        if state["cnh"]:
            ax_h.legend(fontsize=8)
        return []

    _anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)  # noqa: F841 - must stay referenced
    plt.tight_layout()
    plt.show()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serial port, e.g. COM5")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--label", default="capture", help="appended to the file name")
    ap.add_argument("--notes", default="", help="free text stored in the .json sidecar")
    ap.add_argument("--seconds", type=float, default=0, help="record this long (0 = until Ctrl+C / window closed)")
    ap.add_argument("--out-dir", default=str(Path(__file__).resolve().parent.parent / "captures"))
    ap.add_argument("--drivers", help="driver for port A,B: cx|ch|off, e.g. cx,ch (default) or ch,cx (swap)")
    ap.add_argument("--res", type=int, choices=(4, 8))
    ap.add_argument("--hz", type=int)
    ap.add_argument("--cnh", help="off, or start_bin,bins,sub[,merge] e.g. 0,18,2,1")
    ap.add_argument("--i2c", type=int, help="I2C clock in Hz (re-inits both sensors)")
    ap.add_argument("--cmd", action="append", help="raw firmware command, repeatable, e.g. --cmd 'b sharp 0'")
    ap.add_argument("--view", action="store_true", help="live view (needs matplotlib)")
    ap.add_argument("--flip", action="store_true", help="rotate the live view 180 degrees")
    ap.add_argument("--max-mm", type=int, default=1500, help="colour scale limit for the live view")
    args = ap.parse_args()

    ser = open_port(args.port, args.baud)
    reader = Reader(ser)
    reader.start()
    state = {"info": {}, "infos": [], "range": {}, "cnh": {}, "count": {}, "cnh_count": {}, "hist": {},
             "last_info": 0.0}
    settle(reader, state, ser, build_commands(args))

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = dt.datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + "".join(
        ch if ch.isalnum() or ch in "-_" else "_" for ch in args.label)
    vpath = out_dir / (stem + ".vlog")
    f = open(vpath, "wb")
    reader.record_to(f)
    ser.write(b"status\n")  # put the config at the head of the file
    t_start = time.time()
    print(f"recording to {vpath}" + (f" for {args.seconds:g} s" if args.seconds else " (Ctrl+C to stop)"))

    try:
        if args.view:
            live_view(state, reader, args)
        else:
            prev: dict = {}
            last = time.time()
            while not args.seconds or time.time() - t_start < args.seconds:
                time.sleep(0.1)
                drain(reader, state)
                if time.time() - last >= 2:
                    print_rates(state, prev, time.time() - last, reader)
                    last = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"status\n")
        time.sleep(0.3)
        reader.stop.set()
        reader.join(timeout=1)
        reader.record_to(None)
        f.close()
        ser.close()
        meta = {
            "label": args.label,
            "notes": args.notes,
            "started": dt.datetime.fromtimestamp(t_start).isoformat(timespec="seconds"),
            "seconds": round(time.time() - t_start, 1),
            "commands": build_commands(args),
            "port_info": state["info"],
            "crc_errors": reader.parser.crc_errors,
        }
        vpath.with_suffix(".json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
        print(f"saved {vpath} ({reader.bytes / 1e6:.1f} MB, {reader.parser.crc_errors} CRC errors)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
