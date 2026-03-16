# Run this to check raw pin behavior before the SM claims them
from machine import Pin
from utime import sleep_ms

SLOT_SENSOR_I_PIN = 12
SLOT_SENSOR_Q_PIN = 11

# Configure with pull-ups
i_pin = Pin(SLOT_SENSOR_I_PIN, Pin.IN, Pin.PULL_UP)
q_pin = Pin(SLOT_SENSOR_Q_PIN, Pin.IN, Pin.PULL_UP)

print("Raw pin values (no SM running). Move tape slowly...")
print("Expect 1,1 at rest with pull-ups. Ctrl-C to stop.")
print()
prev = None
for _ in range(500):
    state = (i_pin.value() << 1) | q_pin.value()
    if state != prev:
        print(f"  I={i_pin.value()} Q={q_pin.value()} state={state:02b}")
        prev = state
    sleep_ms(10)