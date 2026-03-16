import machine
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
from utime import sleep

VERSION_STRING = "0.1.0"

class DotterBoard:
    OLED_WIDTH_PIXELS = 128
    OLED_HEIGHT_PIXELS = 64
    OLED_I2C_SDA_PIN = 0
    OLED_I2C_SCL_PIN = 1

    SLOT_SENSOR_I_PIN = 16
    SLOT_SENSOR_Q_PIN = 15

    BTN_UP_PIN = 9
    BTN_DN_PIN = 8

    components_per_sprocket_hole = 1
    sprocket_holes_per_component = -1

    def __init__(self):
        i2c = I2C(0, scl=Pin(self.OLED_I2C_SCL_PIN), sda=Pin(self.OLED_I2C_SDA_PIN))
        self.oled = SSD1306_I2C(self.OLED_WIDTH_PIXELS, self.OLED_HEIGHT_PIXELS, i2c)
        self.slot_sensor_i = Pin(self.SLOT_SENSOR_I_PIN, Pin.IN, Pin.PULL_UP)
        self.slot_sensor_q = Pin(self.SLOT_SENSOR_Q_PIN, Pin.IN, Pin.PULL_UP)
        self.btn_up = Pin(self.BTN_UP_PIN, Pin.IN, Pin.PULL_UP)
        self.btn_dn = Pin(self.BTN_DN_PIN, Pin.IN, Pin.PULL_UP)
        self.component_count = 0

    def draw_screen(self):
        self.oled.fill(0)

        # Large component count (3x-scaled font, centered)
        self._draw_large_number(str(self.component_count), y=1, scale=3)

        # Tape pitch label (small text, centered below counter)
        if self.components_per_sprocket_hole > 0:
            label = "{}cmp/hole".format(self.components_per_sprocket_hole)
        else:
            label = "{}hole/cmp".format(self.sprocket_holes_per_component)
        lx = max(0, (128 - len(label) * 8) // 2)
        self.oled.text(label, lx, 26, 1)

        # Separator line
        self.oled.hline(0, 36, 128, 1)

        # Tape diagram showing pitch relationship
        self._draw_tape(38)

        self.oled.show()

    def _draw_large_number(self, s, y, scale):
        """Render string s at y using scale * the built-in 8x8 font."""
        import framebuf
        tmp_buf = bytearray(8)  # 8 cols * 1 byte/col = 8 bytes for MONO_HLSB 8x8
        tmp = framebuf.FrameBuffer(tmp_buf, 8, 8, framebuf.MONO_HLSB)
        char_w = 8 * scale
        x = max(0, (128 - len(s) * char_w) // 2)
        for ch in s:
            tmp.fill(0)
            tmp.text(ch, 0, 0, 1)
            for row in range(8):
                for col in range(8):
                    if tmp.pixel(col, row):
                        self.oled.fill_rect(x + col * scale, y + row * scale, scale, scale, 1)
            x += char_w

    def _draw_circle(self, cx, cy, r):
        """Draw a circle outline at (cx, cy) with radius r (midpoint algorithm)."""
        f = 1 - r
        ddF_x = 1
        ddF_y = -2 * r
        x, y = 0, r
        self.oled.pixel(cx,     cy + r, 1)
        self.oled.pixel(cx,     cy - r, 1)
        self.oled.pixel(cx + r, cy,     1)
        self.oled.pixel(cx - r, cy,     1)
        while x < y:
            if f >= 0:
                y -= 1
                ddF_y += 2
                f += ddF_y
            x += 1
            ddF_x += 2
            f += ddF_x
            self.oled.pixel(cx + x, cy + y, 1)
            self.oled.pixel(cx - x, cy + y, 1)
            self.oled.pixel(cx + x, cy - y, 1)
            self.oled.pixel(cx - x, cy - y, 1)
            self.oled.pixel(cx + y, cy + x, 1)
            self.oled.pixel(cx - y, cy + x, 1)
            self.oled.pixel(cx + y, cy - x, 1)
            self.oled.pixel(cx - y, cy - x, 1)

    def _draw_tape(self, y_top):
        """Draw an SMT tape strip from y_top to the bottom of the screen.

        Sprocket holes (circles) run along the bottom edge.
        Component pockets (rectangles) fill the upper zone, sized and spaced
        according to components_per_sprocket_hole / sprocket_holes_per_component.
        """
        W = 128
        tape_h = 63 - y_top        # ~25px tall
        N_HOLES = 8
        hole_pitch = W // N_HOLES  # 16px per sprocket hole

        # Tape border
        self.oled.rect(0, y_top, W, tape_h, 1)

        # Component pocket zone (upper portion of strip)
        pocket_y = y_top + 2
        pocket_h = tape_h - 12     # leaves room for hole row below

        # Sprocket hole circles (bottom portion of strip)
        hole_cy = y_top + tape_h - 5
        hole_r  = 3

        if self.components_per_sprocket_hole > 0:
            # N component pockets packed between each pair of sprocket holes
            n = self.components_per_sprocket_hole
            slot_w   = hole_pitch // n
            pocket_w = max(2, slot_w - 2)
            for i in range(N_HOLES * n):
                px = i * slot_w + (slot_w - pocket_w) // 2
                if px + pocket_w <= W:
                    self.oled.rect(px, pocket_y, pocket_w, pocket_h, 1)
        else:
            # One wide component pocket spanning N sprocket hole slots
            n        = self.sprocket_holes_per_component
            group_w  = hole_pitch * n
            pocket_w = max(4, group_w - 4)
            for i in range(N_HOLES // n + 1):
                px = i * group_w + (group_w - pocket_w) // 2
                if px < W:
                    self.oled.rect(px, pocket_y, min(pocket_w, W - px), pocket_h, 1)

        for i in range(N_HOLES):
            self._draw_circle(i * hole_pitch + hole_pitch // 2, hole_cy, hole_r)


    def update(self):
        self.draw_screen()