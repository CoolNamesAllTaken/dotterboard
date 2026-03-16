import machine
import rp2
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
from utime import sleep


# ---------------------------------------------------------------------------
# PIO quadrature encoder — faithful MicroPython port of the Raspberry Pi SDK
# quadrature_encoder example.
#
# How it works (matching the SDK):
#
#   Y  = running signed encoder count (incremented / decremented in PIO).
#   ISR = scratch register used to build the 4-bit state-transition key.
#   OSR = holds the 4-bit key {prev_state[1:0], curr_state[1:0]} between loops;
#         its top 2 bits are extracted as "prev" on the next iteration.
#
#   Each loop:
#     1. Push Y to the RX FIFO (non-blocking) so the host always has a fresh count.
#     2. out(isr, 2) — shift the top 2 bits of OSR (= prev state) into ISR[1:0].
#     3. in_(pins, 2) — shift current pin state into ISR[1:0], pushing prev to [3:2].
#        ISR[3:0] is now the 4-bit index (prev<<2 | curr).
#     4. mov(osr, isr) — save the full 4-bit key in OSR for the next iteration.
#     5. mov(pc, isr)  — jump to table entry 0–15 for the correct action.
#
#   Shift direction: SHIFT_LEFT for both IN and OUT.
#     IN  SHIFT_LEFT: new bits enter ISR at the LSB; ISR shifts left (toward MSB).
#     OUT SHIFT_LEFT: bits leave OSR from the MSB end.
#     out(isr, 2) writes to ISR using IN_SHIFTDIR (SHIFT_LEFT), so bits land at
#     ISR[1:0] and ISR shifts left 2.  in_(pins,2) does the same.  After both:
#       ISR[3:2] = prev_state,  ISR[1:0] = curr_state.
#     mov(pc, isr) uses ISR[4:0] → address = (prev<<2)|curr. Correct.
#
#   OSR starts at 0 (hardware reset), which is a valid prev_state (00).  No
#   priming is needed, and OUT without autopull never stalls — it just reuses
#   whatever is in OSR.
#
# Pin mapping:
#   in_base     = Q pin (SLOT_SENSOR_Q_PIN = 11) → in_(pins,2) bit 0
#   in_base + 1 = I pin (SLOT_SENSOR_I_PIN = 12) → in_(pins,2) bit 1
#   state = (I<<1) | Q
#
# If the count runs in the wrong direction, swap the +1/−1 sense in update().
# ---------------------------------------------------------------------------

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT, out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def _quadrature_encoder_prog():

    # ---- Jump table: addresses 0–15 ----
    # Index = (prev_state << 2) | curr_state,  state = (I<<1)|Q

    # prev = 00
    jmp("update")           #  0: 00→00  no change
    jmp("decrement")        #  1: 00→01  −1
    jmp("increment")        #  2: 00→10  +1
    jmp("update")           #  3: 00→11  no change (error transition)

    # prev = 01
    jmp("increment")        #  4: 01→00  +1
    jmp("update")           #  5: 01→01  no change
    jmp("update")           #  6: 01→10  no change (error transition)
    jmp("decrement")        #  7: 01→11  −1

    # prev = 10
    jmp("decrement")        #  8: 10→00  −1
    jmp("update")           #  9: 10→01  no change (error transition)
    jmp("update")           # 10: 10→10  no change
    jmp("increment")        # 11: 10→11  +1

    # prev = 11 (last two entries implemented in-place)
    jmp("update")           # 12: 11→00  no change (error transition)
    jmp("increment")        # 13: 11→01  +1

    label("decrement")      # 14: 11→10  −1
    # jmp(y_dec, target) decrements Y unconditionally; target must be the next
    # address so there is no conditional side-effect (mirrors SDK "JMP Y--, update").
    jmp(y_dec, "update")    # Y--; always falls through to "update" at addr 15

    # ---- Main loop ----
    label("update")         # 15
    mov(isr, y)
    push(noblock)           # stream count to host; drop if FIFO full

    # Build 4-bit key:  ISR[3:2]=prev, ISR[1:0]=curr
    out(isr, 2)             # prev state (top 2 bits of OSR) → ISR[1:0], ISR<<=2
    in_(pins, 2)            # curr state → ISR[1:0], ISR<<=2

    mov(osr, isr)           # save 4-bit key in OSR for next iteration
    mov(pc, isr)            # computed jump to table[ISR[4:0]] = table[(prev<<2)|curr]

    # ---- Increment: Y++ via negate/dec/negate ----
    label("increment")
    mov(y, invert(y))
    jmp(y_dec, "inc_cont")
    label("inc_cont")
    mov(y, invert(y))
    # Implicit .wrap → back to "update" (address 15)


VERSION_STRING = "0.1.0"


class DotterBoard:
    OLED_WIDTH_PIXELS = 128
    OLED_HEIGHT_PIXELS = 64
    OLED_I2C_SDA_PIN = 0
    OLED_I2C_SCL_PIN = 1

    SLOT_SENSOR_I_PIN = 12
    SLOT_SENSOR_Q_PIN = 11

    BTN_UP_PIN = 6
    BTN_DN_PIN = 7

    components_per_sprocket_hole = 1
    sprocket_holes_per_component = -1

    def __init__(self):
        i2c = I2C(0, scl=Pin(self.OLED_I2C_SCL_PIN), sda=Pin(self.OLED_I2C_SDA_PIN))
        self.oled = SSD1306_I2C(self.OLED_WIDTH_PIXELS, self.OLED_HEIGHT_PIXELS, i2c)

        # Q (11) = in_base, I (12) = in_base+1.  Must be consecutive GPIOs.
        Pin(self.SLOT_SENSOR_I_PIN, Pin.IN, Pin.PULL_UP)
        q_pin = Pin(self.SLOT_SENSOR_Q_PIN, Pin.IN, Pin.PULL_UP)

        # Run at full sysclk: SM loop ≤ 10 cycles → max ~12.5 M steps/s at 125 MHz.
        # Lower freq = lower power if your encoder is slow (pass max_step_rate*10).
        self._encoder_sm = rp2.StateMachine(
            0,
            _quadrature_encoder_prog,
            freq=125_000_000,
            in_base=q_pin,
        )
        self._encoder_sm.active(1)

        # Track the last signed count read from PIO so we can delta it.
        self._prev_count = 0

        self.btn_up = Pin(self.BTN_UP_PIN, Pin.IN, Pin.PULL_UP)
        self.btn_dn = Pin(self.BTN_DN_PIN, Pin.IN, Pin.PULL_UP)
        self.component_count = 0
        self._sub_hole = 0        # fractional sprocket-hole accumulator (×4 quadrature)
        self._hole_remainder = 0  # partial-component accumulator (holes/cmp mode)
        self._btn_up_held = False
        self._btn_dn_held = False
        self._both_held = False
        self._reset_consumed = False

    # ------------------------------------------------------------------
    # Encoder helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _u32_to_s32(v):
        """Reinterpret an unsigned 32-bit integer as a signed 32-bit integer."""
        return v if v < 0x80000000 else v - 0x100000000

    def _read_count(self):
        """Drain the RX FIFO and return the most recent signed encoder count.

        The PIO pushes Y continuously (non-blocking), so the FIFO may contain
        several stale values.  We discard all but the newest.  Returns the
        previous value if the FIFO is empty (no motion since last call).
        """
        latest = None
        while self._encoder_sm.rx_fifo():
            latest = self._encoder_sm.get()
        if latest is None:
            return self._prev_count
        return self._u32_to_s32(latest)

    # ------------------------------------------------------------------

    def draw_screen(self):
        self.oled.fill(0)

        self._draw_large_number(str(self.component_count), y=1, scale=3)

        if self.components_per_sprocket_hole > 0:
            label = "{}cmp/hole".format(self.components_per_sprocket_hole)
        else:
            label = "{}hole/cmp".format(self.sprocket_holes_per_component)
        lx = max(0, (128 - len(label) * 8) // 2)
        self.oled.text(label, lx, 26, 1)

        self.oled.hline(0, 36, 128, 1)
        self._draw_tape(38)
        self.oled.show()

    def _draw_large_number(self, s, y, scale):
        import framebuf
        tmp_buf = bytearray(8)
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
        W = 128
        tape_h = 63 - y_top
        N_HOLES = 8
        hole_pitch = W // N_HOLES
        self.oled.rect(0, y_top, W, tape_h, 1)
        pocket_y = y_top + 2
        pocket_h = tape_h - 12
        hole_cy = y_top + tape_h - 5
        hole_r  = 3
        if self.components_per_sprocket_hole > 0:
            n = self.components_per_sprocket_hole
            slot_w   = hole_pitch // n
            pocket_w = max(2, slot_w - 2)
            for i in range(N_HOLES * n):
                px = i * slot_w + (slot_w - pocket_w) // 2
                if px + pocket_w <= W:
                    self.oled.rect(px, pocket_y, pocket_w, pocket_h, 1)
        else:
            n        = self.sprocket_holes_per_component
            group_w  = hole_pitch * n
            pocket_w = max(4, group_w - 4)
            for i in range(N_HOLES // n + 1):
                px = i * group_w + (group_w - pocket_w) // 2
                if px < W:
                    self.oled.rect(px, pocket_y, min(pocket_w, W - px), pocket_h, 1)
        for i in range(N_HOLES):
            self._draw_circle(i * hole_pitch + hole_pitch // 2, hole_cy, hole_r)

    def _pitch_up(self):
        self._hole_remainder = 0
        if self.sprocket_holes_per_component > 0:
            if self.sprocket_holes_per_component == 2:
                self.components_per_sprocket_hole = 1
                self.sprocket_holes_per_component = -1
            else:
                self.sprocket_holes_per_component -= 1
        else:
            self.components_per_sprocket_hole += 1

    def _pitch_dn(self):
        self._hole_remainder = 0
        if self.components_per_sprocket_hole > 0:
            if self.components_per_sprocket_hole == 1:
                self.sprocket_holes_per_component = 2
                self.components_per_sprocket_hole = -1
            else:
                self.components_per_sprocket_hole -= 1
        else:
            self.sprocket_holes_per_component += 1

    def update(self):
        # --- Encoder ---
        # Read the latest Y value from PIO and compute the delta since last call.
        # Y increments/decrements by 1 per quadrature edge; 4 edges = 1 full cycle
        # = 1 sprocket hole.
        count = self._read_count()
        delta = count - self._prev_count
        self._prev_count = count

        if delta:
            self._sub_hole += delta
            holes = int(self._sub_hole / 4)   # truncate toward zero
            self._sub_hole -= holes * 4
            if holes:
                if self.components_per_sprocket_hole > 0:
                    self.component_count += holes * self.components_per_sprocket_hole
                else:
                    self._hole_remainder += holes
                    n = self.sprocket_holes_per_component
                    comp_delta = int(self._hole_remainder / n)
                    self._hole_remainder -= comp_delta * n
                    self.component_count += comp_delta

        # --- Buttons ---
        btn_up = self.btn_up.value()
        btn_dn = self.btn_dn.value()

        up_fell = not self._btn_up_held and not btn_up
        up_rose = self._btn_up_held and btn_up
        dn_fell = not self._btn_dn_held and not btn_dn
        dn_rose = self._btn_dn_held and btn_dn

        if up_fell:
            self._btn_up_held = True
        if dn_fell:
            self._btn_dn_held = True
        if self._btn_up_held and self._btn_dn_held:
            self._both_held = True

        if up_rose:
            if self._both_held:
                self.component_count = 0
                self._both_held = False
                self._reset_consumed = True
            elif not self._reset_consumed:
                self._pitch_up()
            self._btn_up_held = False

        if dn_rose:
            if self._both_held:
                self.component_count = 0
                self._both_held = False
                self._reset_consumed = True
            elif not self._reset_consumed:
                self._pitch_dn()
            self._btn_dn_held = False

        if not self._btn_up_held and not self._btn_dn_held:
            self._reset_consumed = False

        self.draw_screen()