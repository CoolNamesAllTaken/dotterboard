import machine
from machine import Pin, I2C, ADC, PWM
from ssd1306 import SSD1306_I2C
# from enum import Enum # for states
from utime import sleep

VERSION_STRING = "0.1.0"
U16_MAX = 2**16-1

class Bigass7SegmentTester:
    OLED_WIDTH_PIXELS = 128
    OLED_HEIGHT_PIXELS = 64
    OLED_I2C_SDA_PIN = 16
    OLED_I2C_SCL_PIN = 17

    NUM_SEGMENTS = 3
    LED_ON_PINS = [2, 6, 10]
    POWER_ENABLE_PINS = [3, 7, 11]
    PASS_LED_PINS = [0, 4, 8]
    ERROR_LED_PINS = [1, 5, 9]
    FIXTURE_CLOSED_PIN = 12
    CURRENT_SENSE_PINS = [26, 27, 28]
    BUZZER_PIN = 15
    BUZZER_DUTY_U16 = int(U16_MAX / 2) # approximate a square wave

    CURRENT_SENSE_ADC_FULL_SCALE_MA = 396 # Current when the ADC is railed to 3.3V

    led_on_pins = []
    power_enable_pins = []
    pass_led_pins = []
    error_led_pins = []
    current_sense_adcs = []

    class Bigass7SegmentTesterState:
        FIXTURE_OPEN = 0
        TEST_STARTED = 1
        TEST_COMPLETE = 2
    
    fixture_state = Bigass7SegmentTesterState.FIXTURE_OPEN

    class Bigass7SegmentTesterResults:
        TEXT_HEIGHT = 10

        def __init__(self, num_segments):
            self.idle_currents_ma = [0] * num_segments
            self.active_currents_ma = [0] * num_segments
            self.idle_current_ok = [False] * num_segments
            self.active_current_ok = [False] * num_segments
            self.led_segment_ok = [False] * num_segments
        
        def show(self, oled, pass_leds, fail_leds):
            for i in range(len(self.led_segment_ok)):
                if self.idle_current_ok[i] and self.active_current_ok[i]:
                    self.led_segment_ok[i] = True

            oled.fill(0) # clear screen
            if all(self.led_segment_ok):
                oled.text("ALL PASSED", 0, 0)
            else:
                failed_leds = [i for i in range(len(self.led_segment_ok)) if not self.led_segment_ok[i]]
                failed_leds_str = ','.join([f"{i+1}" for i in failed_leds])
                oled.text(f"FAILED: {failed_leds_str}", 0, 0)
            oled.text(f"{'SEG':<3}|{'IDL':<5}|{'ACT':<5}", 0, self.TEXT_HEIGHT)
            for i in range(len(self.led_segment_ok)):
                oled.text(f"{i+1:<3}|{self.idle_currents_ma[i]:5.2f}|{self.active_currents_ma[i]:5.2f}", 0, self.TEXT_HEIGHT*(i+2))
            oled.show()

            for i, ok in enumerate(self.led_segment_ok):
                pass_leds[i].value(ok)
                fail_leds[i].value(not ok)
                

    results = Bigass7SegmentTesterResults(NUM_SEGMENTS)

    def __init__(self):
        i2c = I2C(0, sda=Pin(self.OLED_I2C_SDA_PIN), scl=Pin(self.OLED_I2C_SCL_PIN))
        self.oled = SSD1306_I2C(self.OLED_WIDTH_PIXELS, self.OLED_HEIGHT_PIXELS, i2c)
        self.buzzer_pin = PWM(Pin(self.BUZZER_PIN))

        for i in range(self.NUM_SEGMENTS):
            self.led_on_pins.append(Pin(self.LED_ON_PINS[i], Pin.OUT))
            self.led_on_pins[i].value(0)

            self.power_enable_pins.append(Pin(self.POWER_ENABLE_PINS[i], Pin.OUT))
            self.power_enable_pins[i].value(0)

            self.pass_led_pins.append(Pin(self.PASS_LED_PINS[i], Pin.OUT))
            self.pass_led_pins[i].value(0)

            self.error_led_pins.append(Pin(self.ERROR_LED_PINS[i], Pin.OUT))
            self.error_led_pins[i].value(0)

            self.fixture_closed_pin = Pin(self.FIXTURE_CLOSED_PIN, Pin.IN)

            self.current_sense_adcs.append(ADC(Pin(self.CURRENT_SENSE_PINS[i])))
    
    def reset(self):
        self.oled.fill(0) # clear the screen
        self.oled.text("BIGASS 7 SEGMENT", 0, 0)
        self.oled.text("TESTER", int(self.OLED_WIDTH_PIXELS/3), self.results.TEXT_HEIGHT)
        self.oled.text(f"v{VERSION_STRING}", int(self.OLED_WIDTH_PIXELS/3), 2*self.results.TEXT_HEIGHT)
        self.oled.show()
        self.results = self.Bigass7SegmentTesterResults(self.NUM_SEGMENTS)
    
    
    
    def update(self, skip_on_fail=False):
        # State Transitions
        if self.fixture_state == self.Bigass7SegmentTesterState.FIXTURE_OPEN:
            # previous state: fixture waiting to begin a test
            if self.fixture_closed_pin.value():
                # fixture has been closed, begin a test
                self.fixture_state = self.Bigass7SegmentTesterState.TEST_STARTED
                print("TEST_STARTED")
                if not self.test_idle_current():
                    # idle current test failed, skip active current test
                    self.fixture_state = self.Bigass7SegmentTesterState.TEST_COMPLETE
                    print("TEST_COMPLETE (FAILED EARLY)")
                    self.display_results()
                    if skip_on_fail:
                        return # skip remaining tests
                # idle current test passed, try active current test
                self.test_active_current() # the rest is the same regardless of the active test result
                self.fixture_state = self.Bigass7SegmentTesterState.TEST_COMPLETE
                print("TEST_COMPLETE")
                self.display_results()
                return
        # NOTE: test is blocking, so there is no active testing state
        elif self.fixture_state == self.Bigass7SegmentTesterState.TEST_COMPLETE:
            # previous state: fixture finished running a test
            if not self.fixture_closed_pin.value():
                # fixture has been opened, reset
                self.fixture_state = self.Bigass7SegmentTesterState.FIXTURE_OPEN
                print("FIXTURE_OPEN")
                self.reset()
                return
            # fixture is still closed, we're chillin
    
    def display_results(self):
        self.results.show(self.oled, self.pass_led_pins, self.error_led_pins) # show what failed
        if not all(self.results.led_segment_ok):
            self.sad_beep()
        else:
            self.happy_beep()
        
    def sad_beep(self):
        self.beep_pattern([800, 0, 500, 0, 800, 0, 500], [0.1, 0.05, 0.1, 0.05,  0.1, 0.05, 0.1])
    
    def happy_beep(self):
        self.beep_pattern([500, 0, 800], [0.1, 0.05, 0.1])

    def beep_pattern(self, note_frequencies, note_durations):
        """
        Plays a pattern indicated by a list of frequencies and durations. Silent notes inidicated
        by 0 frequency. Blocking.
        """
        for i in range(len(note_frequencies)):
            if note_frequencies[i] == 0:
                # silent note indicated by 0 frequency
                self.buzzer_pin.duty_u16(0)
            else:
                # play note for duration
                self.buzzer_pin.freq(note_frequencies[i])
                self.buzzer_pin.duty_u16(self.BUZZER_DUTY_U16)
            sleep(note_durations[i])
        self.buzzer_pin.duty_u16(0) # turn off buzzer


    def set_all_power_enable_pins(self, value):
        for i in range(len(self.power_enable_pins)):
            self.power_enable_pins[i].value(value)
    
    def read_segment_current(self, segment):
        return self.current_sense_adcs[segment].read_u16() / U16_MAX * self.CURRENT_SENSE_ADC_FULL_SCALE_MA

    def test_idle_current(self, idle_current_fail_threshold_ma = 10):
        """
        Tests the idle current of all LED segments to see if they draw power when provided with power
        but the LED is supposed to be off.

        Returns:
        True if all LED segments OK, False if any failed.
        """
        # Enable power to the LEDs and see if they have any current.
        self.set_all_power_enable_pins(1) # enable power
        sleep(0.1)
        segment_currents_ma = [self.read_segment_current(i) for i in range(self.NUM_SEGMENTS)]
        self.set_all_power_enable_pins(0) # disable power

        # Record results.
        for i, current in enumerate(segment_currents_ma):
            self.results.idle_currents_ma[i] = current
            if current < idle_current_fail_threshold_ma:
                self.results.idle_current_ok[i] = True

        return all(self.results.idle_current_ok)
    
    def test_active_current(self, active_current_min_threshold_ma=120, active_current_max_threshold_ma=160):
        self.set_all_power_enable_pins(1) # enable power
        sleep(0.1)
        segment_currents_ma = [0] * self.NUM_SEGMENTS
        for i in range(self.NUM_SEGMENTS):
            self.led_on_pins[i].value(1) # turn segment on
            sleep(0.1)
            segment_currents_ma[i] = self.read_segment_current(i)
            self.led_on_pins[i].value(0) # turn segment off
        self.set_all_power_enable_pins(0) # disable power

        # Record results.
        for i, current in enumerate(segment_currents_ma):
            self.results.active_currents_ma[i] = current
            if current < active_current_max_threshold_ma and current > active_current_min_threshold_ma:
                self.results.active_current_ok[i] = True

        return all(self.results.active_current_ok)


        
    