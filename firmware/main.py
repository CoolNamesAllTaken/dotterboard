import machine
from ssd1306 import SSD1306_I2C
# from bigass_7_segment_tester import Bigass7SegmentTester
from dotterboard import DotterBoard

def main():
    dotterboard = DotterBoard()
    while(True):
        dotterboard.update()

if __name__ == "__main__":
    main()