from machine import Pin, PWM, time_pulse_us
from time import sleep, ticks_ms, sleep_us

# RC Channel Input Pins
CH1 = Pin(21, Pin.IN)	# Right side, L-R
CH2 = Pin(20, Pin.IN)	# Right side, up/down
CH3 = Pin(19, Pin.IN)	# left side, throttle up/down
CH4 = Pin(18, Pin.IN)	# left side, L-R
CH5 = Pin(17, Pin.IN) 	#pin 17 is extra green wire, to switch
CH6 = Pin(16, Pin.IN) 	# not connected yet.

# LED
carLED = Pin(25, Pin.OUT)

# TB6612FNG Motor Pins
# Front A
Front_pwmA = PWM(Pin(7))
Front_in1A = Pin(5, Pin.OUT)
Front_in2A = Pin(6, Pin.OUT)

# Front B
Front_pwmB = PWM(Pin(2))
Front_in1B = Pin(4, Pin.OUT)
Front_in2B = Pin(3, Pin.OUT)

# Rear A
Rear_pwmA = PWM(Pin(15))
Rear_in1A = Pin(13, Pin.OUT)
Rear_in2A = Pin(14, Pin.OUT)

# Rear B
Rear_pwmB = PWM(Pin(10))
Rear_in1B = Pin(12, Pin.OUT)
Rear_in2B = Pin(11, Pin.OUT)

# Standby
Front_stby = Pin(1, Pin.OUT)
Rear_stby = Pin(9, Pin.OUT)

# encoder input pins:
Enc_FL = PIN(0, Pin.IN)
Enc_FR = PIN(8, Pin.IN)
Enc_BL = PIN(16, Pin.IN) # use current CH6 blue wire
Enc_BR = PIN(28, Pin.IN)


# Initialize PWM
for pwm in [Front_pwmA, Front_pwmB, Rear_pwmA, Rear_pwmB]:
    pwm.freq(1000)

# Helper: Read RC channel pulse (Microseconds)
def read_channel(pin, min_val=-100, max_val=100, default=0):
    try:
        while pin.value()==1:
            sleep_us(100)
        pulse = time_pulse_us(pin, 1, 30000) # 20ms peak-peak = 50Hz
        if pulse < 1000:
        #    print("pulse < 1000")
        #    return 1000, 1000
            return int(0), int(pulse)
        if pulse > 2000:
        #    print("pulse > 2000")
        #    return 2000, 0
            return int(2000), int(pulse)
        return int((pulse - 1000) * (max_val - min_val) / 1000 + min_val), int(pulse)
    except:
        # return previous value instead?
        print("Failed reading channel")
        return 0, 0

# Helper: Set motor direction and speed
def motor_control(pwm, in1, in2, speed, direction):
    if direction == 0:
        in1.low()
        in2.high()
    else:
        in1.high()
        in2.low()
    duty = int(min(max(speed, 0), 255) * 65535 / 255)
    pwm.duty_u16(duty)

def mControlA(speed, dir):
    motor_control(Front_pwmA, Front_in1A, Front_in2A, speed, dir)
    motor_control(Rear_pwmA, Rear_in1A, Rear_in2A, speed, dir)

def mControlB(speed, dir):
    motor_control(Front_pwmB, Front_in1B, Front_in2B, speed, dir)
    motor_control(Rear_pwmB, Rear_in1B, Rear_in2B, speed, dir)

# Startup LED blink
for _ in range(2):
    carLED.on()
    sleep(0.5)
    carLED.off()
    sleep(0.5)

# Enable motors
Front_stby.high()
Rear_stby.high()

# Main loop
while True:
    rcCH1, rawCH1 = read_channel(CH1, -100, 100, 0)
    rcCH2, rawCH2 = read_channel(CH2, -100, 100, 0)
    rcCH3, rawCH3 = read_channel(CH3, 0, 155, 0)
    rcCH4, rawCH4 = read_channel(CH4, -100, 100, 0)
    rcCH5, rawCH5 = read_channel(CH5, 0, 100, 0) # Switch
    rcCH5 = rcCH5 > 50 # convert to boolean
    rcCH6, rawCH6 = read_channel(CH6, 0, 100, 0) # dial
    #rcCH6 = read_channel(CH6, 0, 100, 0) > 50  # Boolean
    timestamp = ticks_ms()

    MotorSpeedA = rcCH3
    MotorSpeedB = rcCH3

    # use spin mode if velocity = 0 only?
    
#     if rcCH6:  # Spin Mode
#         carLED.on()
#         if rcCH5 >= 0:
#             MotorDirA = 0
#             MotorDirB = 1
#             print("Spin Clockwise")
#         else:
#             MotorDirA = 1
#             MotorDirB = 0
#             print("Spin Counter-Clockwise")
# 
#         spin_speed = abs(rcCH5)
#         MotorSpeedA += spin_speed
#         MotorSpeedB += spin_speed
#     else:  # Normal Mode
    carLED.off()
    if rcCH2 >= 0:
        MotorDirA = 1
        MotorDirB = 1
        #print("Forward")
    else:
        MotorDirA = 0
        MotorDirB = 0
        #print("Backward")

    MotorSpeedA += abs(rcCH2)
    MotorSpeedB += abs(rcCH2)
    MotorSpeedA -= rcCH1
    MotorSpeedB += rcCH1

    # Limit speeds to 0–255
    MotorSpeedA = max(0, min(255, MotorSpeedA))
    MotorSpeedB = max(0, min(255, MotorSpeedB))

    # Drive motors
    mControlA(MotorSpeedA, MotorDirA)
    mControlB(MotorSpeedB, MotorDirB)

    if rcCH2 > 0:
        print(timestamp, "RAW  CH1:", rawCH1, "CH2:", rawCH2, "CH3:", rawCH3, "CH4:", rawCH4, "CH5:", rawCH5, "CH6:", rawCH6)
        print(timestamp, "NORM CH1:", rcCH1, "CH2:", rcCH2, "CH3:", rcCH3, "CH4:", rcCH4, " Motor A Speed:", MotorSpeedA, " B Speed:", MotorSpeedB, " Forward" )
    elif rcCH2 < 0:
        print(timestamp, "RAW  CH1:", rawCH1, "CH2:", rawCH2, "CH3:", rawCH3, "CH4:", rawCH4, "CH5:", rawCH5, "CH6:", rawCH6)
        print(timestamp, "NORM CH1:", rcCH1, "CH2:", rcCH2, "CH3:", rcCH3, "CH4:", rcCH4,  " Motor A Speed:", MotorSpeedA, " B Speed:", MotorSpeedB, " Reverse" )
    else :
        print(timestamp, "RAW  CH1:", rawCH1, "CH2:", rawCH2, "CH3:", rawCH3, "CH4:", rawCH4, "CH5:", rawCH5, "CH6:", rawCH6)
        print(timestamp, "NORM CH1:", rcCH1, "CH2:", rcCH2, "CH3:", rcCH3, "CH4:", rcCH4,  " Motor A Speed:", MotorSpeedA, " B Speed:", MotorSpeedB, " Stopped" )
    
    print("-----")
    carLED.on()
    sleep(0.05)
    carLED.off()


