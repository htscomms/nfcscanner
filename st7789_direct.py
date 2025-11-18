# st7789_direct.py
# Minimal, fast ST7789 (240x280) SPI driver with RGB565 blit
# Works with Raspberry Pi (spidev + RPi.GPIO)
import os, time
import spidev
import RPi.GPIO as GPIO

# ST7789 commands
SWRESET = 0x01
SLPOUT  = 0x11
DISPON  = 0x29
CASET   = 0x2A
RASET   = 0x2B
RAMWR   = 0x2C
MADCTL  = 0x36
COLMOD  = 0x3A
INVON   = 0x21
INVOFF  = 0x20

# MADCTL bits (ST7789)
MADCTL_MY  = 0x80
MADCTL_MX  = 0x40
MADCTL_MV  = 0x20
MADCTL_BGR = 0x08

def _pack_rgb565(r, g, b):
    return bytes([
        ((r & 0xF8)      ) | ((g & 0xE0) >> 5),
        ((g & 0x1C) << 3) |  (b >> 3)
    ])

class ST7789Direct:
    """
    Direct SPI driver for 240x280 ST7789V2 panels (e.g. Waveshare 1.69").
    Provides:
      - clear((r,g,b))
      - blit_rgb565(frame_bytes)  # len == width*height*2
      - close()
    """
    def __init__(
        self,
        width=240, height=280,
        dc=20, rst=16, bl=None,
        speed_hz=None, spi_mode=None,
        rotation=0, invert=False, bgr=False,
        x_offset=0, y_offset=0,
        transfer_chunk=None,
        bus=0, device=0
    ):
        self.width  = int(width)
        self.height = int(height)
        self.x_off  = int(x_offset)
        self.y_off  = int(y_offset)
        self.dc     = int(dc)
        self.rst    = int(rst)
        self.bl     = bl if bl is None else int(bl)

        # Environment fallbacks
        self.speed_hz = int(speed_hz if speed_hz is not None else int(os.getenv("LCD_SPI_HZ", "28000000")))
        self.spi_mode = int(spi_mode if spi_mode is not None else int(os.getenv("LCD_SPI_MODE", "3")))
        self.chunk    = int(transfer_chunk if transfer_chunk is not None else int(os.getenv("LCD_XFER", "32768")))
        self.rotation = int(rotation) & 3
        self.invert   = bool(invert)
        self.bgr      = bool(bgr)

        # GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dc,  GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.rst, GPIO.OUT, initial=GPIO.HIGH)
        if self.bl is not None:
            GPIO.setup(self.bl, GPIO.OUT, initial=GPIO.HIGH)

        # SPI
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.mode = self.spi_mode
        self.spi.max_speed_hz = self.speed_hz
        self.spi.no_cs = False
        # writebytes2 accepts bytes (fast path)
        self._write = self.spi.writebytes2

        self._reset()
        self._init_panel()

    # ---------- low-level ----------
    def _reset(self):
        GPIO.output(self.rst, GPIO.HIGH); time.sleep(0.005)
        GPIO.output(self.rst, GPIO.LOW);  time.sleep(0.020)
        GPIO.output(self.rst, GPIO.HIGH); time.sleep(0.120)

    def _cmd(self, c, data: bytes | None = None):
        GPIO.output(self.dc, GPIO.LOW)
        self._write([c & 0xFF])
        if data:
            GPIO.output(self.dc, GPIO.HIGH)
            # split large payloads to keep kernel happy
            for i in range(0, len(data), self.chunk):
                self._write(data[i:i+self.chunk])

    def _data(self, data: bytes):
        GPIO.output(self.dc, GPIO.HIGH)
        for i in range(0, len(data), self.chunk):
            self._write(data[i:i+self.chunk])

    def _madctl_value(self):
        v = MADCTL_BGR if self.bgr else 0x00
        if self.rotation == 0:      # 240x280 portrait
            v |= 0
        elif self.rotation == 1:    # 90°
            v |= (MADCTL_MX | MADCTL_MV)
        elif self.rotation == 2:    # 180°
            v |= (MADCTL_MX | MADCTL_MY)
        else:                       # 270°
            v |= (MADCTL_MY | MADCTL_MV)
        return v & 0xFF

    def _init_panel(self):
        # Software reset
        self._cmd(SWRESET); time.sleep(0.120)
        # Pixel format: 16-bit
        self._cmd(COLMOD, b'\x55')  # 16bpp
        time.sleep(0.010)
        # Memory access control
        self._cmd(MADCTL, bytes([self._madctl_value()]))

        # Inversion
        self._cmd(INVON if self.invert else INVOFF)
        time.sleep(0.010)

        # Out of sleep & display on
        self._cmd(SLPOUT); time.sleep(0.120)
        self._cmd(DISPON); time.sleep(0.020)

        # Prime full window (optional)
        self._set_window(0, 0, self.width-1, self.height-1)

    def _set_window(self, x0, y0, x1, y1):
        x0 += self.x_off; x1 += self.x_off
        y0 += self.y_off; y1 += self.y_off
        # CASET
        self._cmd(CASET, bytes([
            (x0 >> 8) & 0xFF, x0 & 0xFF,
            (x1 >> 8) & 0xFF, x1 & 0xFF
        ]))
        # RASET
        self._cmd(RASET, bytes([
            (y0 >> 8) & 0xFF, y0 & 0xFF,
            (y1 >> 8) & 0xFF, y1 & 0xFF
        ]))
        # Write memory command ready
        GPIO.output(self.dc, GPIO.LOW)
        self._write([RAMWR])
        GPIO.output(self.dc, GPIO.HIGH)

    # ---------- public ----------
    def clear(self, rgb=(0, 0, 0)):
        r, g, b = rgb
        px = _pack_rgb565(r, g, b)
        line = px * self.width
        self._set_window(0, 0, self.width-1, self.height-1)
        for _ in range(self.height):
            for i in range(0, len(line), self.chunk):
                self._write(line[i:i+self.chunk])

    def blit_rgb565(self, frame: bytes):
        """
        Draw a full-frame RGB565 buffer (len == width*height*2).
        """
        expected = self.width * self.height * 2
        if not isinstance(frame, (bytes, bytearray, memoryview)) or len(frame) != expected:
            raise ValueError(f"blit_rgb565: expected {expected} bytes, got {len(frame) if hasattr(frame,'__len__') else '??'}")

        # Full-screen write
        self._set_window(0, 0, self.width-1, self.height-1)
        # Stream in chunks
        for i in range(0, expected, self.chunk):
            self._write(frame[i:i+self.chunk])

    # Backwards-compat alias (some code may call dev.blit(...))
    blit = blit_rgb565

    def close(self):
        try:
            self.clear((0, 0, 0))
        except Exception:
            pass
        try:
            self.spi.close()
        except Exception:
            pass
        try:
            if self.bl is not None:
                GPIO.output(self.bl, GPIO.LOW)
            GPIO.cleanup([self.dc, self.rst] + ([] if self.bl is None else [self.bl]))
        except Exception:
            pass
