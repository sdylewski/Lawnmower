from rp2 import PIO, StateMachine, asm_pio
from machine import Pin
import utime

# Single-pin, single-edge (X1) counter:
# - Watches one GPIO (index 0 relative to in_base)
# - Counts ONLY rising edges on that pin
# - Increments X by +1 per rising edge
@asm_pio(autopush=True, push_thresh=32)  # IN X,32 will auto-push to RX FIFO
def edge_count_x1_single_pin():
    wrap_target()                          # Start of the wrapped loop

    wait(0, pin, 0)                        # Ensure the pin is LOW first -> "arm" for a true rising edge

    label("loop")
    wait(1, pin, 0)                        # ***TRIGGER***: rising edge on the monitored pin

    # --- Increment X by +1 (PIO doesn't have INC; this is the standard idiom) ---
    mov(x, invert(x))                      # X = ~X
    jmp(x_dec, "inc_done")                 # Decrement inverted X by 1 and jump to next line
    label("inc_done")
    mov(x, invert(x))                      # X = ~(~X - 1) => X = X + 1

    wait(0, pin, 0)                        # Re-arm: wait for the pin to go LOW (ignore falling edge)
    jmp("loop")                            # Repeat forever

    wrap()                                 # End of wrapped loop


# ---------------- Python side ----------------
# Choose the encoder channel you want to watch (e.g., GPIO3 = A/CLK).
# No jmp_pin is provided because we are not reading a second pin.
sm1 = StateMachine(1, edge_count_x1_single_pin,
                   freq=125_000_000,      # PIO clock (1 instruction / cycle)
                   in_base=Pin(0))        # "pin,0" in the PIO refers to GPIO3

sm1.active(1)

# Helper to compute signed 32-bit deltas (handles wrap-around)
def i32(n): 
    return (n + 0x80000000) % 0x100000000 - 0x80000000

last_x = 0

while True:
    utime.sleep(1.0)

    # Read the 32-bit X register (autopush pushes it to RX FIFO)
    sm1.exec("in_(x, 32)")
    x = sm1.get()

    # Unsigned-to-signed delta since last read (handles wrap)
    dx = i32(x - last_x)
    last_x = x

    # If you know the commanded motor direction, apply sign here
    # Example: dir_sign = +1 for forward, -1 for reverse (from your driver pins)
    dir_sign = +1
    signed_counts = dir_sign * dx

    print(f"counts={dx} (signed={signed_counts}) total={x}")
