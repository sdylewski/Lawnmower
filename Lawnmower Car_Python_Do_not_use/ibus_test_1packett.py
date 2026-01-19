from machine import UART
import time

# === Config ===
uart = UART(0, baudrate=115200, tx=machine.Pin(16), rx=machine.Pin(17))
IBUS_BUFFSIZE = 32
IBUS_MAXCHANNELS = 6
debug = True  # Set False to disable debug output

buffer = bytearray(IBUS_BUFFSIZE)
ibusIndex = 0
start_detected = False

def normalize(value, type="default"):
    if type == "dial":
        return (value - 1000) / 10
    else:
        return (value - 1500) / 5

def read_ibus():
    global ibusIndex, start_detected
    while uart.any():
        val = uart.read(1)[0]

        if ibusIndex == 0:
            if val == 0x20:
                buffer[ibusIndex] = val
                ibusIndex += 1
            continue

        elif ibusIndex == 1:
            if val == 0x40:
                buffer[ibusIndex] = val
                ibusIndex += 1
            else:
                ibusIndex = 0
            continue

        else:
            buffer[ibusIndex] = val
            ibusIndex += 1

        if ibusIndex >= IBUS_BUFFSIZE:
            ibusIndex = 0  # Reset for next read

            # === Checksum ===
            checksum = 0xFFFF
            for i in range(30):
                checksum -= buffer[i]
            checksum &= 0xFFFF
            received_checksum = (buffer[31] << 8) | buffer[30]

            if checksum != received_checksum:
                if debug:
                    print("📦 Raw:", list(buffer))
                    print(f"❌ Checksum mismatch: received {received_checksum:#06x}, expected {checksum:#06x}")
                return None

            # === Parse Channels ===
            channels = []
            try:
                high = 3
                low = 2
                for i in range(IBUS_MAXCHANNELS):
                    raw = (buffer[high] << 8) | buffer[low]
                    norm_type = "dial" if i >= 4 else "default"
                    channels.append(normalize(raw, norm_type))
                    high += 2
                    low += 2
            except IndexError:
                if debug:
                    print("❌ ERROR: Channel parsing out of range")
                return None

            if debug:
                print("✅ Valid packet:")
                print("   Raw:", list(buffer))
                print("   Channels:", channels)

            return channels

# === Example usage loop ===
while True:
    result = read_ibus()
    if result:
        print("🎮 Channels:", result)
    time.sleep(0.01)
