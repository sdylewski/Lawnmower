from machine import UART, Pin
import time

# Set up UART1 with RX on GPIO17
uart = UART(0, baudrate=115200, rx=Pin(17))

def read_ibus():
    if uart.any():
        data = uart.read(32)
        if data and len(data) == 32 and data[0] == 0x20 and data[-1] == 0x04:
            channels = []
            for i in range(14):  # 14 channels max
                ch_val = data[2 + i * 2] | (data[3 + i * 2] << 8)
                channels.append(ch_val)
            return channels
    return None

while True:
    ch = read_ibus()
    if ch:
        print("Channels:", ch)
    time.sleep(0.05)