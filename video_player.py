# video_player.py
import os, subprocess, threading, time, shutil
from pathlib import Path

FFLOG      = os.getenv("FFMPEG_LOGLEVEL", "quiet")  # quiet ffmpeg
NO_BLIT    = os.getenv("NO_BLIT", "0").lower() in ("1","true","yes","on")
FF_THREADS = int(os.getenv("FF_THREADS", "2"))       # ffmpeg decoder threads

def _read_exact(pipe, n: int, stop_event: threading.Event) -> bytes | None:
    buf = bytearray(n); view = memoryview(buf); got = 0
    while got < n and not stop_event.is_set():
        try:
            chunk = pipe.read(n - got)
        except Exception as e:
            print(f"[VID] read_exact exception: {e}", flush=True)
            return None
        if not chunk:
            return None  # EOF or ffmpeg exited
        view[got:got+len(chunk)] = chunk
        got += len(chunk)
    return bytes(buf) if got == n else None

class VideoPlayer:
    """
    Pre-emptive single-clip player:
      - play(path, loop): stops current clip, starts new one
      - wait_done(): waits for non-looping clip to finish
      - stop(): pre-empts current clip

    Device must implement: blit_rgb565(frame_bytes)  # len = w*h*2 (big-endian)
    """
    def __init__(self, device, width: int, height: int, fps: int = 22, bgr: bool = False):
        self.dev = device
        self.w   = int(width)
        self.h   = int(height)
        self.fps = max(1, int(fps))
        self.bgr = bool(bgr)

        self._lock    = threading.RLock()
        self._thread  = None
        self._proc    = None
        self._stop    = threading.Event()
        self._done    = threading.Event()
        self._is_loop = False
        self._current = None

    def play(self, path: str, loop: bool = True):
        with self._lock:
            self._stop_playback_locked()
            self._stop.clear(); self._done.clear()
            self._is_loop = bool(loop)
            self._current = str(path)
            t = threading.Thread(target=self._run_clip, args=(self._current, self._is_loop), daemon=True)
            self._thread = t; t.start()

    def wait_done(self, timeout: float | None = None):
        if self._is_loop: return False
        return self._done.wait(timeout=timeout)

    def stop(self):
        with self._lock:
            self._stop_playback_locked()

    def close(self): self.stop()

    # ---------- internals ----------
    def _stop_playback_locked(self):
        self._stop.set()
        if self._proc is not None:
            try: self._proc.terminate()
            except Exception: pass
            try: self._proc.kill()
            except Exception: pass
            self._proc = None
        t = self._thread
        if t and t.is_alive():
            try: t.join(timeout=1.0)
            except Exception: pass
        self._thread = None

    def _run_clip(self, path: str, loop: bool):
        if not path or not Path(path).exists():
            print(f"[VID] missing file: {path}", flush=True)
            if not loop: self._done.set()
            return

        ff = shutil.which("ffmpeg")
        if not ff:
            print("[VID] ffmpeg not found", flush=True)
            if not loop: self._done.set()
            return

        # Output raw RGB565 **big-endian** at panel size & CFR. No audio, quiet.
        cmd = [ff, "-hide_banner", "-nostdin", "-loglevel", FFLOG]
        if loop:
            cmd += ["-stream_loop", "-1"]
        cmd += [
            "-i", path,
            "-an",
            "-vf", f"scale={self.w}:{self.h}:flags=bilinear,fps={self.fps}",
            "-pix_fmt", "rgb565be",
            "-fflags", "nobuffer",
            "-f", "rawvideo", "-"
        ]
        if FF_THREADS > 0:
            cmd += ["-threads", str(FF_THREADS)]

        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0
            )
            print(f"[VID] ffmpeg started pid={self._proc.pid} loop={loop} file={path}", flush=True)
        except Exception as e:
            print(f"[VID] ffmpeg start failed: {e}", flush=True)
            if not loop: self._done.set()
            return

        frame_bytes   = self.w * self.h * 2
        target_dt     = 1.0 / self.fps
        drop_enabled  = loop  # only drop on looping clips (e.g., scan.mp4)
        out           = self._proc.stdout
        frames_seen   = 0
        dropped_total = 0
        last_log      = time.monotonic()

        while not self._stop.is_set():
            read_start = time.perf_counter()
            chunk = _read_exact(out, frame_bytes, self._stop)
            if chunk is None:
                rc = self._proc.poll()
                print(f"[VID] EOF/break after {frames_seen} frame(s); ffmpeg rc={rc}", flush=True)
                break

            # Blit (or decode-only if NO_BLIT=1)
            blit_start = time.perf_counter()
            if not NO_BLIT:
                try:
                    self.dev.blit_rgb565(chunk)
                except Exception as e:
                    print(f"[VID] blit error (frame {frames_seen+1}): {e}", flush=True)
                    break
            blit_dt = time.perf_counter() - blit_start

            frames_seen += 1

            # Heartbeat & stats
            now = time.monotonic()
            if frames_seen <= 3 or (now - last_log) > 2.0:
                if dropped_total:
                    print(f"[VID] frame {frames_seen} ok (dropped {dropped_total} so far) // {self._current}", flush=True)
                else:
                    print(f"[VID] frame {frames_seen} ok // {self._current}", flush=True)
                last_log = now

            # If we're late, drop frames to catch up (only for looped videos)
            if drop_enabled and blit_dt > target_dt:
                behind = blit_dt - target_dt
                # Drop up to 4 frames per cycle to catch up, but don't go crazy
                to_drop = min(4, int(behind / target_dt) + 1)
                dropped = 0
                for _ in range(to_drop):
                    junk = _read_exact(out, frame_bytes, self._stop)
                    if junk is None:
                        break
                    dropped += 1
                dropped_total += dropped
                if dropped:
                    print(f"[VID] dropped {dropped} frame(s) to maintain realtime", flush=True)

        # Cleanup
        try:
            if self._proc: self._proc.terminate()
        except Exception: pass
        try:
            if self._proc: self._proc.kill()
        except Exception: pass
        rc = None
        try:
            if self._proc: rc = self._proc.wait(timeout=0.2)
        except Exception: pass
        self._proc = None
        print(f"[VID] ffmpeg closed rc={rc} frames_total={frames_seen} dropped_total={dropped_total}", flush=True)

        if not loop and not self._stop.is_set():
            self._done.set()
