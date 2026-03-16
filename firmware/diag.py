# Run this on the Pico to diagnose why the encoder count is stuck at 0.
# Paste into the REPL or run as a script.

import rp2
from machine import Pin
from utime import sleep_ms

SLOT_SENSOR_I_PIN = 12
SLOT_SENSOR_Q_PIN = 11

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT, out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def _quadrature_encoder_prog():
    jmp("update")
    jmp("decrement")
    jmp("increment")
    jmp("update")
    jmp("increment")
    jmp("update")
    jmp("update")
    jmp("decrement")
    jmp("decrement")
    jmp("update")
    jmp("update")
    jmp("increment")
    jmp("update")
    jmp("increment")
    label("decrement")
    jmp(y_dec, "update")
    label("update")
    mov(isr, y)
    push(noblock)
    out(isr, 2)
    in_(pins, 2)
    mov(osr, isr)
    mov(pc, isr)
    label("increment")
    mov(y, invert(y))
    jmp(y_dec, "inc_cont")
    label("inc_cont")
    mov(y, invert(y))

Pin(SLOT_SENSOR_I_PIN, Pin.IN, Pin.PULL_UP)
q_pin = Pin(SLOT_SENSOR_Q_PIN, Pin.IN, Pin.PULL_UP)

sm = rp2.StateMachine(0, _quadrature_encoder_prog, freq=10_000, in_base=q_pin)
sm.active(1)

print("=== ENCODER DIAGNOSTICS ===")
print()

# Test 1: Is the SM pushing anything at all?
sleep_ms(100)
level = sm.rx_fifo()
print(f"1. FIFO level after 100ms: {level}")
if level == 0:
    print("   PROBLEM: SM is not pushing to FIFO. Program may have crashed.")
    print("   Check: does mov(pc, isr) work on this MicroPython version?")
else:
    print("   OK: SM is running and pushing data.")

# Test 2: What raw values are in the FIFO?
vals = []
for _ in range(sm.rx_fifo()):
    vals.append(sm.get())
if vals:
    def s32(v): return v if v < 0x80000000 else v - 0x100000000
    signed = [s32(v) for v in vals]
    print(f"2. Raw FIFO values (last few): {vals[-4:]}")
    print(f"   As signed: {signed[-4:]}")
    if all(v == 0 for v in vals):
        print("   Y is stuck at 0 - SM runs but never inc/dec.")
        print("   Key always lands on 'update'. Check bit layout of ISR.")
    else:
        print("   Y is changing - PIO counting works!")
else:
    print("2. FIFO was empty (drained above or SM not running)")

# Test 3: Read pin states directly
i_val = Pin(SLOT_SENSOR_I_PIN).value()
q_val = Pin(SLOT_SENSOR_Q_PIN).value()
print(f"3. Pin states: I(GP{SLOT_SENSOR_I_PIN})={i_val}, Q(GP{SLOT_SENSOR_Q_PIN})={q_val}")
print(f"   State = (I<<1)|Q = {(i_val<<1)|q_val:02b}")
if i_val == 1 and q_val == 1:
    print("   Both high (pull-ups active, sensor not triggered). Normal at rest.")

# Test 4: Monitor for changes over 3 seconds
print()
print("4. Monitoring for 3 seconds - move the tape now...")
sm.active(0); sm.restart(); sm.active(1)  # reset Y=0
sleep_ms(10)
# drain
while sm.rx_fifo(): sm.get()

prev = None
changes = 0
for _ in range(300):
    sleep_ms(10)
    latest = None
    while sm.rx_fifo():
        latest = sm.get()
    if latest is not None:
        v = latest if latest < 0x80000000 else latest - 0x100000000
        if v != prev:
            print(f"   Y changed: {v}")
            changes += 1
            prev = v
            if changes >= 10:
                break

if changes == 0:
    print("   No changes detected. Either no tape movement or SM not counting.")
    print()
    print("LIKELY CAUSE: mov(pc, isr) may not be supported or works differently.")
    print("Try replacing the PIO program with the simple state-push version.")
else:
    print(f"   Detected {changes} changes. PIO encoder is working!")