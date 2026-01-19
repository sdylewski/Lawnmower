from machine import UART, Pin
import time

# Initialize UART0 with RX on GPIO17
uart = UART(0, baudrate=115200, rx=Pin(17))

NUM_CHANNELS = 6
debug=False

def normalize(value, type="default"):
    if type == "dial":
        return (value - 1000) / 10
    else:
        return (value - 1500) / 5
    
def read_ibus():
    if uart.any() >= 32:
        buffer = uart.read(32)

        if not buffer or len(buffer) != 32:
            print("📦 Raw Data:", list(buffer) if buffer else "None")
            print("❌ ERROR: Incomplete or no data")
            return None

        if buffer[0] != 0x20:
            print("📦 Raw Data:", list(buffer))
            print(f"❌ ERROR: Invalid start byte: {buffer[0]}")
            return None

        # Accurate checksum calculation
        checksum = 0xFFFF  # 0xFFFF - start byte 0x20 = 0xffdf
        for i in range(30):         # subtract bytes 0 through 30
            checksum -= buffer[i]
        #checksum &= 0xFFFF             # keep it 16-bit
        # if checksum == (buffer[30] << 8) | buffer[29]:
        received_checksum = (buffer[31] << 8) | buffer[30]  # last 2 bytes, little endian

        if checksum != received_checksum:
            print("📦 Raw Data:", list(buffer))
            print(f"❌ ERROR: Checksum failed. Received: {received_checksum:#06x}, Calculated: {checksum:#06x}")
            return None

        # parse channels
        channels = []
        try:
            for i in range(NUM_CHANNELS):  # 0 to 13
                index = 2 + i * 2
                raw = buffer[index] + (buffer[index + 1] << 8)
                norm_type = "dial" if i >= 4 else "default"
                channels.append(normalize(raw, norm_type))
        except IndexError:
            if debug:
                print("📦 Raw Data:", list(buffer))
                print("❌ ERROR: Channel data incomplete.")
            return None

        if debug:
            print("📦 Raw Data:", list(buffer))
            print(f"✅ Checksum OK: {checksum:#06x}")

        return channels
    else:
        return None  # No data yet

# Main loop
while True:
    channels = read_ibus()
    if channels:
        print("✅ Channels:", channels)
    time.sleep(0.05)
