#!/usr/bin/env python3
"""
picoExtRef: ./pico.py clean | build | flash [--webserver] | webserver | all | lint [--check]
  build     cmake + make  →  build/firmware.elf
  flash     OpenOCD program build/firmware.elf; --webserver starts Flask dashboard
  webserver Flask dashboard only (USB CDC serial; no OpenOCD / reflash)
  all       build, then flash, then webserver (typical dev cycle)
  lint      clang-format using ./.clang-format; --check = dry-run for CI

  Web UI: pip install -r requirements.txt  (flask, pyserial)

  Build only in one clone path. CMake stores absolute paths in build/CMakeCache.txt — do not copy
  build/ between directories (e.g. ~/git/picoExtRef vs ~/orangebox/picoExtRef). If you develop over
  SSH, run ./pico.py on that host in the real checkout; do not reuse build/ from a different path
  (e.g. a local Cursor workspace vs the remote tree).
"""

from __future__ import annotations

import argparse
import errno
import glob
import multiprocessing
import os
import re
import shlex
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import NamedTuple, Optional

# --- logging (user pattern) -------------------------------------------------

loggingStartup = datetime.now()
loggingState = "build"
loggingExpectedDuration = None
guiCallback = None


class GuiState(NamedTuple):
    msg: str
    img_path: Optional[str] = None
    waiting_for_input: bool = False
    has_error: bool = False
    progress: Optional[object] = None


def showProgress():
    return None


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


class LogOBJ:
    def __init__(
        self,
        call,
        msg=None,
        image=None,
        nofail=False,
        silent=False,
        noTime=False,
        expectedDuration=None,
    ):
        global loggingExpectedDuration

        self.call = call
        self.msg = msg
        self.start = datetime.now()
        self.done = False
        self.nofail = nofail
        self.silent = silent
        self.stderr = ""
        self.stdout = ""

        elapsed = (datetime.now() - loggingStartup).total_seconds()

        if expectedDuration is not None:
            loggingExpectedDuration = elapsed + expectedDuration

        if not silent:
            if not noTime:
                print(
                    "{0}{1:10.2f} {2}".format(
                        bcolors.BOLD, elapsed, datetime.now().strftime("%H:%M:%S")
                    ),
                    end=" ",
                )

            print("{0}{1}{2}".format(bcolors.OKBLUE, loggingState, bcolors.ENDC), end=" ")

            display_msg = call if msg is None else msg
            print("{}".format(display_msg), end=" ", flush=True)

            if guiCallback is not None:
                try:
                    guiCallback(
                        GuiState(
                            display_msg,
                            img_path=image,
                            waiting_for_input=(loggingState == "userinput"),
                            has_error=(loggingState == "error"),
                            progress=showProgress(),
                        )
                    )
                except Exception:
                    pass

    def assignOutput(self, stdout, stderr):
        self.stderr = stderr
        self.stdout = stdout

    def ok(self):
        self.done = True
        if self.silent:
            return

        print("[ {}OK{} ]".format(bcolors.OKGREEN, bcolors.ENDC))

    def fail(self):
        self.done = True

        if self.silent:
            return

        if self.nofail:
            print("", flush=True)
            return

        print("[ {}FAIL{} ]".format(bcolors.FAIL, bcolors.ENDC), flush=True)

        print(self.call)

        if self.stdout != "":
            print(bcolors.WARNING + self.stdout + bcolors.ENDC, flush=True)
        if self.stderr != "":
            print(bcolors.FAIL + self.stderr + bcolors.ENDC, flush=True)

    def ignore(self):
        self.done = True

        if self.silent:
            return

        print("", flush=True)

    def __del__(self):
        if not self.done and not self.silent:
            raise Exception("logger died without result")


def _shell_run(cmd: str, *, cwd=None) -> tuple[int, bytes, bytes]:
    """Run `cmd` through bash -c (only subprocess callsite in this file)."""
    my_env = os.environ.copy()
    my_env["LC_ALL"] = "C"
    process = subprocess.Popen(
        ["bash", "-c", cmd],
        env=my_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=cwd,
    )
    stdout, stderr = process.communicate()
    code = process.returncode if process.returncode is not None else 0
    return code, stdout, stderr


def command(command: str, log=None, cwd=None, nofail=False, silent=False) -> bytes:
    logo = LogOBJ(command, log, nofail=nofail, silent=silent)
    code, stdout, stderr = _shell_run(command, cwd=cwd)
    logo.assignOutput(stdout.decode("utf-8", errors="replace"), stderr.decode("utf-8", errors="replace"))
    if code != 0:
        logo.fail()
        if log is None:
            raise Exception("command failed {}\n{}\n{}\n".format(command, stdout, stderr))
        else:
            print("command failed {}\n{}\n{}\n".format(command, stdout, stderr))
            raise Exception("Failure : " + log)
    logo.ok()
    return stdout


# --- project paths ----------------------------------------------------------

SCRIPT_DIR = Path(__file__).resolve().parent
BUILD_DIR = SCRIPT_DIR / "build"
DEFAULT_ELF = BUILD_DIR / "firmware.elf"
PICO_SDK_DEFAULT = Path.home() / "git" / "pico-sdk"

# Flash / TTY env defaults (see do_flash / find_firmware_tty)
DEFAULT_USB_VID = "1209"
DEFAULT_USB_PID = "0c01"
DEFAULT_VENDOR_SUBSTR = "orangebox"
DEFAULT_PRODUCT_SUBSTR = "picoextref"
DEFAULT_PICOPROBE_VID = "2e8a"
DEFAULT_PICOPROBE_PID = "000c"


def _env(k: str, default: str) -> str:
    v = os.environ.get(k)
    return v if v is not None and v != "" else default


def norm_usb_id(s: str) -> str:
    s = s.strip().lower()
    if s.startswith("0x"):
        s = s[2:]
    s = re.sub(r"[^0-9a-f]", "", s)
    if not s:
        return ""
    return f"{int(s, 16):04x}"


def _udev_properties(tty: Path) -> dict[str, str]:
    if not shutil.which("udevadm"):
        return {}
    cmd = f"udevadm info -q property -n {shlex.quote(str(tty))} 2>/dev/null"
    code, out_b, _ = _shell_run(cmd)
    if code != 0:
        return {}
    out = out_b.decode("utf-8", errors="replace")
    d: dict[str, str] = {}
    for line in out.splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            d[k] = v
    return d


def tty_usb_ids(props: dict[str, str]) -> tuple[str, str]:
    vid = props.get("ID_VENDOR_ID") or props.get("ID_USB_VENDOR_ID") or ""
    pid = props.get("ID_MODEL_ID") or props.get("ID_USB_MODEL_ID") or ""
    return norm_usb_id(vid), norm_usb_id(pid)


def tty_usb_vendor_lower(props: dict[str, str]) -> str:
    v = props.get("ID_VENDOR_ENC") or props.get("ID_VENDOR") or ""
    return v.replace("_", " ").lower()


def tty_usb_product_lower(props: dict[str, str]) -> str:
    m = props.get("ID_MODEL_ENC") or props.get("ID_MODEL") or ""
    return m.replace("_", " ").lower()


def tty_verify_primary_strings(props: dict[str, str], tty: Path) -> bool:
    vs = _env("PICO_USB_VENDOR_SUBSTR", DEFAULT_VENDOR_SUBSTR).lower()
    ps = _env("PICO_USB_PRODUCT_SUBSTR", DEFAULT_PRODUCT_SUBSTR).lower()
    if not vs and not ps:
        return True
    v = tty_usb_vendor_lower(props)
    p = tty_usb_product_lower(props)
    if vs:
        if v:
            if vs not in v:
                return False
        else:
            print(
                f"[flash] WARNING: no USB vendor string from udev for {tty}; accepting VID:PID only",
                file=sys.stderr,
            )
    if ps:
        if p:
            if ps not in p:
                return False
        else:
            print(
                f"[flash] WARNING: no USB product string from udev for {tty}; accepting VID:PID only",
                file=sys.stderr,
            )
    return True


def find_firmware_tty() -> Path:
    """Resolve /dev/ttyACM* for firmware CDC (VID/PID + optional udev string checks)."""
    wait_s = float(_env("PICO_TTY_WAIT_S", "20"))
    max_loops = int(wait_s * 5)
    want_vid = norm_usb_id(_env("PICO_USB_VID", DEFAULT_USB_VID))
    want_pid = norm_usb_id(_env("PICO_USB_PID", DEFAULT_USB_PID))
    pico_vid = norm_usb_id(_env("PICO_PICOPROBE_VID", DEFAULT_PICOPROBE_VID))
    pico_pid = norm_usb_id(_env("PICO_PICOPROBE_PID", DEFAULT_PICOPROBE_PID))

    fixed = os.environ.get("PICO_TTY", "").strip()
    if fixed:
        p = Path(fixed)
        print(f"[flash] Waiting for fixed PICO_TTY={p} (up to {wait_s:.0f}s)...", file=sys.stderr)
        for _ in range(max_loops):
            if p.exists():
                return p
            time.sleep(0.2)
        raise RuntimeError(f"{p} never appeared")

    if not shutil.which("udevadm"):
        print(
            "[flash] WARNING: udevadm missing; cannot match VID:PID — set PICO_TTY or install udev",
            file=sys.stderr,
        )

    print(f"[flash] Waiting for CDC {want_vid}:{want_pid}", file=sys.stderr)
    vs = _env("PICO_USB_VENDOR_SUBSTR", DEFAULT_VENDOR_SUBSTR)
    ps = _env("PICO_USB_PRODUCT_SUBSTR", DEFAULT_PRODUCT_SUBSTR)
    if vs or ps:
        print(
            f"[flash] String checks: vendor~='{vs}', product~='{ps}'",
            file=sys.stderr,
        )

    for _ in range(max_loops):
        for tty_str in sorted(glob.glob("/dev/ttyACM[0-9]*")):
            tty = Path(tty_str)
            props = _udev_properties(tty)
            vid, pid = tty_usb_ids(props)
            if not vid or not pid:
                continue
            if vid == want_vid and pid == want_pid:
                if tty_verify_primary_strings(props, tty):
                    print(
                        f"[flash] Matched {tty} (USB {vid}:{pid} {tty_usb_vendor_lower(props)} | {tty_usb_product_lower(props)})",
                        file=sys.stderr,
                    )
                    return tty
                print(
                    f"[flash] Skipping {tty} (USB {vid}:{pid} — VID:PID OK but vendor/product string mismatch)",
                    file=sys.stderr,
                )
                continue
            if vid == "2e8a" and pid == "1010":
                print(
                    f"[flash] Matched {tty} (USB {vid}:{pid} — legacy; reflash for {want_vid}:{want_pid})",
                    file=sys.stderr,
                )
                return tty
            if vid == "2e8a" and pid == "000a":
                print(
                    f"[flash] Matched {tty} (USB {vid}:{pid} — stock SDK CDC; reflash for {want_vid}:{want_pid})",
                    file=sys.stderr,
                )
                return tty
        time.sleep(0.2)

    print(
        f"[flash] No {want_vid}:{want_pid} (or legacy 2e8a:1010 / 2e8a:000a); fallback: first ttyACM* not Picoprobe {pico_vid}:{pico_pid}...",
        file=sys.stderr,
    )
    for _ in range(max_loops):
        for tty_str in sorted(glob.glob("/dev/ttyACM[0-9]*")):
            tty = Path(tty_str)
            props = _udev_properties(tty)
            vid, pid = tty_usb_ids(props)
            if not vid or not pid:
                continue
            if vid == pico_vid and pid == pico_pid:
                continue
            print(
                f"[flash] Using {tty} (USB {vid}:{pid}) — verify this is the target Pico",
                file=sys.stderr,
            )
            return tty
        time.sleep(0.2)

    raise RuntimeError("no suitable /dev/ttyACM* — connect USB or set PICO_TTY")


def _tail_settle_s() -> float:
    return float(_env("PICO_TAIL_SETTLE_S", "1.0"))


def _tail_ready_timeout_s() -> float:
    return float(_env("PICO_TAIL_READY_TIMEOUT_S", "15"))


def wait_tty_cdc_ready(tty: Path) -> None:
    """After USB re-enumeration, /dev/ttyACM* can exist before the CDC interface accepts I/O (cat → EIO)."""
    time.sleep(_tail_settle_s())
    deadline = time.time() + _tail_ready_timeout_s()
    attempt = 0
    while time.time() < deadline:
        attempt += 1
        try:
            fd = os.open(str(tty), os.O_RDWR | os.O_NOCTTY)
            os.close(fd)
            if attempt > 1:
                print(f"[flash] CDC ready on {tty} after {attempt} open attempt(s)", file=sys.stderr)
            return
        except OSError as e:
            if e.errno in (errno.EIO, errno.ENXIO, errno.ENODEV, errno.EBUSY):
                time.sleep(0.15)
                continue
            raise
    raise OSError(errno.EIO, f"{tty} not accepting opens after USB reset (try increasing PICO_TAIL_SETTLE_S)")


def run_flask_dashboard(tty: Path, *, log_prefix: str = "[web]") -> None:
    """Open CDC, set baud, then run Flask (pico_gpsdo_web)."""
    baud_s = _env("PICO_SERIAL_BAUD", "9600")
    try:
        baud = int(baud_s)
    except ValueError:
        raise SystemExit(f"invalid PICO_SERIAL_BAUD: {baud_s!r}")
    try:
        port = int(_env("PICO_WEB_PORT", "8765"))
    except ValueError:
        raise SystemExit("invalid PICO_WEB_PORT")
    # 0.0.0.0: reachable from this machine via 127.0.0.1 and from other hosts on the LAN.
    # Use PICO_WEB_HOST=127.0.0.1 to listen on localhost only.
    host = _env("PICO_WEB_HOST", "0.0.0.0")
    print(f"{log_prefix} Flask dashboard for {tty} — Ctrl+C to exit", file=sys.stderr)
    print(
        f"{log_prefix} Note: boot logs are emitted before the port opens; refresh log once running.",
        file=sys.stderr,
    )
    print(
        f"{log_prefix} Waiting for CDC to accept I/O (settle {_tail_settle_s()}s, timeout {_tail_ready_timeout_s()}s)...",
        file=sys.stderr,
    )
    wait_tty_cdc_ready(tty)
    deadline = time.time() + 3.0
    stty_cmd = f"stty -F {shlex.quote(str(tty))} {shlex.quote(baud_s)}"
    while time.time() < deadline:
        code, _, _ = _shell_run(stty_cmd)
        if code == 0:
            break
        time.sleep(0.1)
    try:
        from pico_gpsdo_web import run_dashboard
    except ImportError as e:
        raise SystemExit(
            "Flask dashboard requires: pip install -r requirements.txt  (flask, pyserial)"
        ) from e
    run_dashboard(str(tty), host=host, port=port, baud=baud)


def which_clang_format() -> str:
    for name in ("clang-format", "clang-format-18", "clang-format-17", "clang-format-16"):
        p = shutil.which(name)
        if p:
            return p
    raise SystemExit("clang-format not found in PATH (install clang-format or llvm-tools)")


def collect_clang_format_sources() -> list[Path]:
    """All *.c, *.h, *.cpp, *.hpp, *.cc, *.cxx under the repo, excluding build trees and hidden dirs."""
    exts = {".c", ".h", ".cpp", ".hpp", ".cc", ".cxx"}
    out: list[Path] = []
    for p in SCRIPT_DIR.rglob("*"):
        if not p.is_file():
            continue
        if p.suffix.lower() not in exts:
            continue
        try:
            rel = p.relative_to(SCRIPT_DIR)
        except ValueError:
            continue
        rel_parts = rel.parts
        if "build" in rel_parts or "_deps" in rel_parts:
            continue
        if any(part.startswith(".") for part in rel_parts):
            continue
        out.append(p)
    return sorted(out)


def do_lint_format(*, check_only: bool) -> None:
    """Run clang-format using .clang-format at repo root (-i in place, or --dry-run --check)."""
    cfg = SCRIPT_DIR / ".clang-format"
    if not cfg.is_file():
        raise SystemExit(f"Missing {cfg}")
    cf = which_clang_format()
    files = collect_clang_format_sources()
    if not files:
        print("[lint] no C/C++ sources found (excluding build/, _deps/, hidden dirs)", file=sys.stderr)
        return

    global loggingState
    loggingState = "lint"

    batch_size = 64
    for i in range(0, len(files), batch_size):
        batch = files[i : i + batch_size]
        parts: list[str] = [cf]
        if check_only:
            parts.extend(["--dry-run", "--Werror"])
        else:
            parts.append("-i")
        parts.append("--style=file")
        parts.extend(str(f) for f in batch)
        cmd = " ".join(shlex.quote(p) for p in parts)
        try:
            command(cmd, cwd=str(SCRIPT_DIR), silent=True)
        except Exception as e:
            raise SystemExit(1) from e

    mode = "check" if check_only else "formatted"
    print(f"[lint] clang-format {mode}: {len(files)} file(s) ({cf})", file=sys.stderr)


def make_command() -> str:
    j = max(1, multiprocessing.cpu_count() or 4)
    return f"make -j{j}"


def do_clean() -> None:
    global loggingState
    loggingState = "clean"
    for p in (BUILD_DIR,):
        if p.exists():
            shutil.rmtree(p)
            print(f"[clean] removed {p}", file=sys.stderr)


def _cmake_cache_wrong_source_tree() -> Optional[str]:
    """If CMakeCache was generated for another source path, return an error message; else None."""
    cache = BUILD_DIR / "CMakeCache.txt"
    if not cache.is_file():
        return None
    want = str(SCRIPT_DIR.resolve())
    try:
        text = cache.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    m = re.search(r"^CMAKE_HOME_DIRECTORY:\w+=(.+)$", text, re.MULTILINE)
    if not m:
        return None
    got = m.group(1).strip().rstrip("/")
    if got == want.rstrip("/"):
        return None
    return (
        "build/CMakeCache.txt belongs to a different source directory than this checkout.\n"
        f"  configured for: {got}\n"
        f"  this ./pico.py is: {want}\n"
        "Run: ./pico.py clean\n"
        "Then build only here — do not copy build/ between two clone paths or machines.\n"
        "If you use SSH: run ./pico.py on the host where this repository lives; "
        "do not mix build/ from a local IDE path with the remote tree."
    )


def check_orangebox_string(elf: Path) -> None:
    if not elf.is_file():
        return
    if not shutil.which("arm-none-eabi-strings"):
        return
    cmd = f"arm-none-eabi-strings {shlex.quote(str(elf))} 2>/dev/null"
    code, out_b, _ = _shell_run(cmd)
    if code != 0:
        return
    out = out_b.decode("utf-8", errors="replace")
    if not re.search(r"^Orangebox$", out, re.MULTILINE):
        print(
            f"[build] WARNING: {elf} does not contain USB manufacturer string \"Orangebox\".",
            file=sys.stderr,
        )
        print(
            "[build] lsusb will show 2e8a:000a until CMakeLists.txt USBD_* definitions are used; try: rm -rf build && ./pico.py build",
            file=sys.stderr,
        )


def do_build(flash: bool, webserver: bool) -> None:
    global loggingState
    loggingState = "build"
    os.environ.setdefault("PICO_SDK_PATH", str(PICO_SDK_DEFAULT.expanduser()))

    bad = _cmake_cache_wrong_source_tree()
    if bad:
        print(bad, file=sys.stderr)
        raise SystemExit(1)

    src = SCRIPT_DIR.resolve()
    bld = BUILD_DIR.resolve()
    BUILD_DIR.mkdir(parents=True, exist_ok=True)

    board_dir = (SCRIPT_DIR / "boards").resolve()
    cmake_cmd = (
        f"cmake -S {shlex.quote(str(src))} -B {shlex.quote(str(bld))} "
        f"-DPICO_BOARD_HEADER_DIRS={shlex.quote(str(board_dir))} "
        f"-DPICO_BOARD=pico_ocxo"
    )
    command(cmake_cmd, log="cmake", cwd=str(bld))
    command(make_command(), log="make", cwd=str(BUILD_DIR))

    check_orangebox_string(DEFAULT_ELF)

    if flash:
        print("[build] make finished; running flash...", file=sys.stderr)
        do_flash(webserver=webserver)


def do_flash(*, webserver: bool) -> None:
    global loggingState
    loggingState = "flash"
    path = DEFAULT_ELF.resolve()
    if not path.is_file():
        raise SystemExit(f"firmware ELF not found: {path}")

    loggingState = "openocd"
    ef = str(path)
    # Tcl: program {path} — braces allow spaces in path
    openocd_cmd = (
        f'openocd -s tcl -c "adapter speed 5000" -f interface/cmsis-dap.cfg '
        f'-f target/rp2040.cfg -c "program {{{ef}}} verify reset exit"'
    )
    command(openocd_cmd, log="openocd")
    print("[flash] OpenOCD finished (device reset; USB may re-enumerate)", file=sys.stderr)

    if webserver:
        tty = find_firmware_tty()
        run_flask_dashboard(tty, log_prefix="[flash]")


def do_webserver() -> None:
    """Flask dashboard on existing firmware (no OpenOCD)."""
    global loggingState
    loggingState = "webserver"
    tty = find_firmware_tty()
    run_flask_dashboard(tty, log_prefix="[web]")


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="pico.py",
        description="Build / flash / clean / format (see module docstring).",
    )
    sub = parser.add_subparsers(dest="cmd", required=True, metavar="CMD")

    sub.add_parser("clean", help="Remove build/")

    sub.add_parser("build", help="Configure (cmake) and compile → build/firmware.elf")

    p_flash = sub.add_parser("flash", help="Program build/firmware.elf via OpenOCD")
    p_flash.add_argument(
        "--webserver",
        action="store_true",
        help="Then open Flask dashboard on USB serial (pip install -r requirements.txt)",
    )

    sub.add_parser("all", help="build + flash + --webserver (one step)")

    sub.add_parser(
        "webserver",
        help="Flask GPSDO dashboard on USB CDC only (no build/flash; same env vars as flash --webserver)",
    )

    p_lint = sub.add_parser("lint", help="Run clang-format using ./.clang-format")
    p_lint.add_argument(
        "--check",
        action="store_true",
        help="Dry-run only; exit non-zero if sources need formatting",
    )

    args = parser.parse_args()
    if args.cmd == "clean":
        do_clean()
        return
    if args.cmd == "build":
        do_build(False, False)
        return
    if args.cmd == "flash":
        do_flash(webserver=args.webserver)
        return
    if args.cmd == "all":
        do_build(True, True)
        return
    if args.cmd == "webserver":
        do_webserver()
        return
    if args.cmd == "lint":
        do_lint_format(check_only=args.check)
        return


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
