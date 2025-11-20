#!/usr/bin/env python3
# main.py — RFID + Buzzer + ST7789 240x280 video UI
# Fix: avoid stalls by SKIPPING token read by default; optional bounded token read via env.

import os, sys, time, json, base64, argparse, re, shutil, subprocess
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, TimeoutError

# Keep ffmpeg quiet unless overridden
os.environ.setdefault("FFMPEG_LOGLEVEL", "error")

from st7789_direct import ST7789Direct
from video_player   import VideoPlayer

# PiicoDev
from PiicoDev_RFID import PiicoDev_RFID
from PiicoDev_Buzzer import PiicoDev_Buzzer
from PiicoDev_Unified import sleep_ms

# ---------- Files / constants ----------
SCRIPT_DIR = Path(__file__).resolve().parent
os.chdir(SCRIPT_DIR)

VIDEOS = {
    "idle":     str(SCRIPT_DIR / "videos" / "idle.mp4"),
    "scan":     str(SCRIPT_DIR / "videos" / "scan.mp4"),
    "unlocked": str(SCRIPT_DIR / "videos" / "unlocked.mp4"),
    "error":    str(SCRIPT_DIR / "videos" / "error.mp4"),
}

DEFAULT_CONFIG = {
    "lat": -42.8346711,
    "lng": 147.4739913,
    "cmd": "card-authenticate-unlock",
    "item_id": 32,
    "business_id": 19,
}
CONFIG_PATH = os.environ.get("NFC_CFG", str(Path.home() / ".config/nfc-scanner/config.json"))
LOG_PATH    = os.environ.get("NFC_LOG", str(Path.home() / ".config/nfc-scanner/events.log"))

SERVER_HOST = "app-salto-bookmy.remote.beetleblack.com.au"
SERVER_PATH = "/slb-app/api.php"
SERVER_URL  = f"https://{SERVER_HOST}{SERVER_PATH}"

# HTTPS timeouts
HTTP_CONNECT_TIMEOUT = float(os.getenv("HTTP_CONNECT_TIMEOUT", "5.0"))
HTTP_READ_TIMEOUT    = float(os.getenv("HTTP_READ_TIMEOUT", "15.0"))
HTTP_TOTAL_TIMEOUT   = float(os.getenv("HTTP_TOTAL_TIMEOUT", "15.0"))

# Bypass scan video completely (debug)
SCAN_VIDEO_ENABLED = os.getenv("SCAN_VIDEO", "1").lower() not in ("0","false","off","no")

# Optional bounded token read budget (ms). Default 0 = **skip** token read.
TOKEN_BUDGET_MS = int(os.getenv("NFC_TOKEN_BUDGET_MS", "0"))

# Also allow dry-run (no network) for bench testing
SEND_DRY = os.getenv("SEND_DRY", "0").lower() in ("1","true","yes","on")

# ---------- logger ----------
def _ensure_parent(path: str | Path):
    p = Path(path); p.parent.mkdir(parents=True, exist_ok=True)

def log_line(msg: str):
    ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    line = f"{ts} {msg}"
    print(line, flush=True)
    try:
        _ensure_parent(LOG_PATH)
        with open(LOG_PATH, "a", encoding="utf-8") as f:
            f.write(line + "\n")
    except Exception as e:
        sys.stderr.write(f"[LOG] write failed: {e}\n")

def log_env_summary():
    spi_hz   = int(os.getenv("LCD_SPI_HZ", "28000000"))
    spi_mode = int(os.getenv("LCD_SPI_MODE", "3"))
    xfer     = int(os.getenv("LCD_XFER", "32768"))
    fps      = int(os.getenv("VID_FPS", "22"))
    bgr_env  = os.getenv("LCD_BGR", "0").lower()
    inv_env  = os.getenv("LCD_INV", "1").lower()
    bgr      = bgr_env in ("1","true","yes","on")
    inv      = inv_env in ("1","true","yes","on")
    log_line(
        f"[ENV] LCD_SPI_HZ={spi_hz} LCD_SPI_MODE={spi_mode} LCD_XFER={xfer} VID_FPS={fps} "
        f"LCD_BGR={'BGR' if bgr else 'RGB'} LCD_INV={'ON' if inv else 'OFF'} "
        f"HTTP_CONNECT_TIMEOUT={HTTP_CONNECT_TIMEOUT} HTTP_READ_TIMEOUT={HTTP_READ_TIMEOUT} HTTP_TOTAL_TIMEOUT={HTTP_TOTAL_TIMEOUT} "
        f"SCAN_VIDEO={'ON' if SCAN_VIDEO_ENABLED else 'OFF'} DRY={SEND_DRY} NFC_TOKEN_BUDGET_MS={TOKEN_BUDGET_MS}"
    )
    return spi_hz, spi_mode, xfer, fps, bgr, inv

# ---------- config ----------
def ensure_config():
    cfg = DEFAULT_CONFIG.copy()
    p = Path(CONFIG_PATH)
    if p.exists():
        try:
            cfg.update(json.loads(p.read_text()))
        except Exception as e:
            log_line(f"[CFG] parse warn: {e}")
    else:
        _ensure_parent(p)
        p.write_text(json.dumps(cfg, indent=2))
        log_line(f"[CFG] wrote default config to {p}")

    # quick field overrides
    try:
        if "NFC_LAT" in os.environ: cfg["lat"] = float(os.environ["NFC_LAT"])
        if "NFC_LNG" in os.environ: cfg["lng"] = float(os.environ["NFC_LNG"])
    except Exception as e:
        log_line(f"[CFG] env lat/lng override error: {e}")

    log_line(f"[CFG] lat={cfg['lat']} lng={cfg['lng']} cmd={cfg['cmd']} item_id={cfg['item_id']} business_id={cfg['business_id']}")
    return cfg

def safe_preview_json(doc: dict) -> str:
    if os.getenv("LOG_TX_FULL", "0") not in ("1","true","yes","on"):
        d = dict(doc)
        if isinstance(d.get("access_token"), str):
            t=d["access_token"]; d["access_token"]=f"{t[:3]}…{t[-3:]}" if len(t)>7 else "***"
        if isinstance(d.get("new_access_token"), str):
            t=d["new_access_token"]; d["new_access_token"]=f"{t[:3]}…{t[-3:]}" if len(t)>7 else "***"
        if isinstance(d.get("card_id"), str):
            c=d["card_id"]; d["card_id"]=f"…{c[-6:]}" if len(c)>6 else c
        return json.dumps(d, separators=(",",":"))
    return json.dumps(doc, separators=(",",":"))

def interpret(text: str) -> bool:
    try:
        doc = json.loads(text)
        s = doc.get("status", False)
        if isinstance(s, bool): return s
        if isinstance(s, (int,float)): return s != 0
        if isinstance(s, str): return s.strip().lower() in ("true","ok","success","1","y","yes")
        return False
    except Exception as e:
        log_line(f"[JSON] parse error: {e}")
        return False

# ---------- HTTP (curl first, requests fallback) ----------
def _snip(s: str) -> str:
    n = int(os.getenv("LOG_RX_MAX", "2000"))
    return s if len(s)<=n else s[:n]+"…"

def _split_httpstatus(resp: str):
    marker = "HTTPSTATUS:"
    i = resp.rfind(marker)
    if i == -1: return resp, 0
    body = resp[:i]
    try: code = int(resp[i+len(marker):].strip())
    except Exception: code = 0
    return body, code

def send_http(payload: dict, req_id: int) -> tuple[bool, int, str, str]:
    """
    Returns (ok, status_code, body_snip, transport).
    """
    if SEND_DRY:
        log_line(f"[HTTP#{req_id}] DRY RUN — not sending. payload={safe_preview_json(payload)}")
        return False, -1, "DRY-RUN", "dry"

    js  = json.dumps(payload, separators=(",",":"))
    b64 = base64.b64encode(js.encode()).decode()
    preview = safe_preview_json(payload)
    log_line(f"[HTTP#{req_id}] PREP url={SERVER_URL} json_len={len(js)} b64_len={len(b64)} payload={preview}")

    curl = shutil.which("curl")
    if curl:
        cmd = [
            curl, "-sS", "--fail-with-body",
            "-A", "PiVideoRFID/1.0",
            "--connect-timeout", f"{HTTP_CONNECT_TIMEOUT:.2f}",
            "-m", f"{HTTP_TOTAL_TIMEOUT:.2f}",
            "-w", "HTTPSTATUS:%{http_code}",
            "-F", f"b64={b64}",
            SERVER_URL
        ]
        log_line(f"[HTTP#{req_id}] REQ START (curl multipart)")
        log_line(f"[HTTP#{req_id}] CMD: {curl} -sS --fail-with-body -A PiVideoRFID/1.0 --connect-timeout {HTTP_CONNECT_TIMEOUT:.2f} -m {HTTP_TOTAL_TIMEOUT:.2f} -w HTTPSTATUS:%{{http_code}} -F b64=<omitted> {SERVER_URL}")
        t0=time.monotonic()
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=HTTP_TOTAL_TIMEOUT+0.3)
            resp = (out.stdout or "") + (out.stderr or "")
        except subprocess.TimeoutExpired:
            dt=(time.monotonic()-t0)*1000
            log_line(f"[HTTP#{req_id}] TIMEOUT curl after {dt:.0f}ms")
            return False, -1, "TIMEOUT", "curl"

        body, status = _split_httpstatus(resp)
        if 200 <= status < 300:
            ok = interpret(body)
            log_line(f"[HTTP#{req_id}] RSP curl status={status} ok={ok} dur_ms={(time.monotonic()-t0)*1000:.0f} body_len={len(body)}")
            return ok, status, _snip(body), "curl"

        # retry as form
        cmd2 = [
            curl, "-sS", "--fail-with-body",
            "-A", "PiVideoRFID/1.0",
            "--connect-timeout", f"{HTTP_CONNECT_TIMEOUT:.2f}",
            "-m", f"{HTTP_TOTAL_TIMEOUT:.2f}",
            "-w", "HTTPSTATUS:%{http_code}",
            "--data-urlencode", f"b64={b64}",
            SERVER_URL
        ]
        log_line(f"[HTTP#{req_id}] RETRY (curl form)")
        t1=time.monotonic()
        try:
            out2 = subprocess.run(cmd2, capture_output=True, text=True, timeout=HTTP_TOTAL_TIMEOUT+0.3)
            resp2 = (out2.stdout or "") + (out2.stderr or "")
        except subprocess.TimeoutExpired:
            dt=(time.monotonic()-t1)*1000
            log_line(f"[HTTP#{req_id}] TIMEOUT curl(form) after {dt:.0f}ms")
            return False, -1, "TIMEOUT", "curl"

        body2, status2 = _split_httpstatus(resp2)
        if 200 <= status2 < 300:
            ok = interpret(body2)
            log_line(f"[HTTP#{req_id}] RSP curl(FORM) status={status2} ok={ok} dur_ms={(time.monotonic()-t1)*1000:.0f} body_len={len(body2)}")
            return ok, status2, _snip(body2), "curl"

        log_line(f"[HTTP#{req_id}] FAIL curl status={status2} body_snip={_snip(body2)!r}")
        return False, status2, _snip(body2), "curl"

    # requests fallback
    try:
        import requests
        from requests.adapters import HTTPAdapter
        from urllib3.util.retry import Retry
        s = requests.Session()
        s.headers.update({"User-Agent":"PiVideoRFID/1.0","Accept":"application/json"})
        retry = Retry(total=1, backoff_factor=0.2, status_forcelist=(429,500,502,503,504),
                      allowed_methods=frozenset(["POST"]), raise_on_status=False, respect_retry_after_header=True)
        s.mount("https://", HTTPAdapter(pool_connections=1, pool_maxsize=1, max_retries=retry))

        t2=time.monotonic()
        r = s.post(SERVER_URL, files={"b64": (None, b64)}, timeout=(HTTP_CONNECT_TIMEOUT, HTTP_READ_TIMEOUT), verify=True)
        if r.ok and ("POST Failed" in (r.text or "")):
            log_line(f"[HTTP#{req_id}] switching to form (requests)")
            r = s.post(SERVER_URL, data={"b64": b64}, timeout=(HTTP_CONNECT_TIMEOUT, HTTP_READ_TIMEOUT), verify=True)

        status = getattr(r, "status_code", 0)
        body   = getattr(r, "text", "") or ""
        ok     = interpret(body) if getattr(r, "ok", False) else False
        log_line(f"[HTTP#{req_id}] RSP requests status={status} ok={ok} dur_ms={(time.monotonic()-t2)*1000:.0f} body_len={len(body)}")
        return ok, status, _snip(body), "requests"
    except Exception as e:
        log_line(f"[HTTP#{req_id}] EXC requests: {e}")
        return False, -1, f"EXC: {e}", "requests"

# ---------- RFID / buzzer ----------
def read_uid_fast(rfid: PiicoDev_RFID, timeout_s=1.2) -> str | None:
    t0=time.monotonic()
    while time.monotonic()-t0 < timeout_s:
        try:
            if rfid.tagPresent():
                u = rfid.readID()
                if u: return str(u)
        except Exception as e:
            log_line(f"[RFID] uid read err: {e}")
            sleep_ms(40)
            try: rfid = PiicoDev_RFID()
            except Exception as e2:
                log_line(f"[RFID] reinit failed: {e2}")
                sleep_ms(80)
        sleep_ms(6)
    return None

def _worker_readtext(rfid: PiicoDev_RFID):
    """Called in a worker to avoid blocking the main thread."""
    try:
        raw = rfid.readText()
        return raw if isinstance(raw, str) and raw.strip() else ""
    except Exception as e:
        return f"__ERR__:{e}"

def read_token_with_budget(rfid: PiicoDev_RFID, budget_ms: int) -> str | None:
    """Non-blocking: run readText() in a worker; return within budget or None."""
    if budget_ms <= 0:
        return None
    budget_s = max(0.01, budget_ms / 1000.0)
    t0 = time.monotonic()
    with ThreadPoolExecutor(max_workers=1) as ex:
        fut = ex.submit(_worker_readtext, rfid)
        try:
            raw = fut.result(timeout=budget_s)
        except TimeoutError:
            log_line(f"[RFID] readText TIMEOUT after {budget_ms}ms")
            return None
    dt = (time.monotonic() - t0) * 1000.0
    if raw.startswith("__ERR__:"):
        log_line(f"[RFID] readText err: {raw[8:]} (took {dt:.0f}ms)")
        return None
    # extract token-ish
    m = re.findall(r"[A-Za-z0-9]{6,}", raw)
    tok = m[0] if m else ""
    log_line(f"[RFID] readText ok len_raw={len(raw)} token_len={len(tok)} took={dt:.0f}ms")
    return tok or None

def beep_scan(buzz: PiicoDev_Buzzer):
    try: buzz.tone(1100,60); sleep_ms(40); buzz.tone(1500,70)
    except Exception: pass

def beep_ok(buzz: PiicoDev_Buzzer):
    try: buzz.tone(1000,70); sleep_ms(15); buzz.tone(1400,90)
    except Exception: pass

def beep_fail(buzz: PiicoDev_Buzzer):
    try: buzz.tone(300,120); sleep_ms(25); buzz.tone(230,140)
    except Exception: pass

# ---------- LCD / video ----------
def lcd_init():
    spi_hz, spi_mode, xfer, fps, bgr, inv = log_env_summary()
    dev = ST7789Direct(
        width=240, height=280,
        dc=20, rst=16,
        speed_hz=spi_hz,
        spi_mode=spi_mode,
        rotation=0,      # portrait
        invert=inv,      # defaults: INV=1, BGR=0 (RGB)
        bgr=bgr,
        x_offset=0, y_offset=20,
        transfer_chunk=xfer
    )
    dev.clear((0,0,0))
    log_line(f"[LCD] init OK (RGB={'BGR' if bgr else 'RGB'}, invert={'ON' if inv else 'OFF'}, spi={spi_hz}Hz, xfer={xfer})")
    return dev, fps, bgr

def play_state(vp: VideoPlayer, name: str, loop: bool):
    path = VIDEOS[name]
    if not Path(path).exists():
        log_line(f"[VIDEO] MISSING: {path} ({name})")
    else:
        log_line(f"[VIDEO] play name={name} loop={loop} file={path}")
    vp.play(path, loop=loop)

# ---------- app ----------
def run():
    device, fps, bgr = lcd_init()
    rfid = PiicoDev_RFID()
    buzz = PiicoDev_Buzzer()
    cfg  = ensure_config()

    vp   = VideoPlayer(device, width=240, height=280, fps=fps, bgr=bgr)
    log_line("[APP] ready — entering idle")
    play_state(vp, "idle", loop=True)

    scan_no = 0
    try:
        while True:
            uid = read_uid_fast(rfid, timeout_s=0.2)
            if not uid:
                continue

            scan_no += 1
            log_line(f"[SCAN#{scan_no}] tag detected uid={uid}")
            beep_scan(buzz)

            if SCAN_VIDEO_ENABLED:
                play_state(vp, "scan", loop=True)
                log_line(f"[SCAN#{scan_no}] scan video queued (returned)")
            else:
                log_line(f"[SCAN#{scan_no}] scan video DISABLED — continuing immediately")

            # small debounce so antenna settles
            sleep_ms(150)

            # ---- NEW: token read is **skipped by default** to avoid stalls ----
            log_line(f"[SCAN#{scan_no}] token-read START (budget_ms={TOKEN_BUDGET_MS})")
            old_token = read_token_with_budget(rfid, TOKEN_BUDGET_MS)  # None if disabled or timeout
            if old_token:
                log_line(f"[SCAN#{scan_no}] token-read DONE token_len={len(old_token)}")
            else:
                log_line(f"[SCAN#{scan_no}] token-read SKIPPED/EMPTY")

            payload = {
                "lat": float(cfg["lat"]),
                "lng": float(cfg["lng"]),
                "cmd": str(cfg["cmd"]),
                "card_id": uid,
                "item_id": int(cfg["item_id"]),
                "business_id": int(cfg["business_id"]),
                "access_token": old_token or "",
                "new_access_token": old_token or "",
            }
            log_line(f"[SCAN#{scan_no}] HTTP about to send… (watch for [HTTP#{scan_no}] PREP)")

            ok, status, body_snip, transport = send_http(payload, scan_no)
            log_line(f"[HTTP#{scan_no}] FINAL transport={transport} ok={ok} status={status} body_snip={body_snip!r}")

            if ok:
                beep_ok(buzz)
                log_line(f"[SCAN#{scan_no}] UNLOCKED — playing success in full")
                vp.stop()                              # <— add this line
                play_state(vp, "unlocked", loop=False)
                vp.wait_done()  
            else:
                beep_fail(buzz)
                log_line(f"[SCAN#{scan_no}] ERROR — playing error in full")
                vp.stop()                              # <— add this line
                play_state(vp, "error", loop=False)
                vp.wait_done()

            play_state(vp, "idle", loop=True)

    except KeyboardInterrupt:
        log_line("[APP] Ctrl-C — exiting")
    finally:
        try: vp.close()
        except Exception: pass
        try: device.clear((0,0,0))
        except Exception: pass
        log_line("[APP] shutdown complete")

def main():
    ap = argparse.ArgumentParser(description="RFID video UI")
    ap.add_argument("--test", action="store_true", help="play all videos once")
    ap.add_argument("--probe-colors", action="store_true", help="flash solid colours for 0.5s each")
    args = ap.parse_args()

    if args.test:
        device, fps, bgr = lcd_init()
        vp = VideoPlayer(device, width=240, height=280, fps=fps, bgr=bgr)
        for name in ("idle","scan","unlocked","error"):
            play_state(vp, name, loop=False); vp.wait_done(timeout=12.0)
        vp.close(); return

    if args.probe_colors:
        device, *_ = lcd_init()
        from time import sleep
        for name, col in [("RED",(255,0,0)),("GREEN",(0,255,0)),("BLUE",(0,0,255)),("WHITE",(255,255,255)),("BLACK",(0,0,0))]:
            log_line(f"[PROBE] {name}"); device.clear(col); sleep(0.5)
        log_line("[PROBE] done"); return

    run()

if __name__ == "__main__":
    main()
