# Micro:bit Motor Control Program
# Upload this to your micro:bit using the MicroPython editor at https://python.microbit.org

from microbit import *

# Initialize serial communication with Arduino (9600 baud)
uart.init(baudrate=9600)

# Display startup message
display.show(Image.HAPPY)
sleep(500)
display.clear()

while True:
    # Button A: Move all motors FORWARD
    if button_a.is_pressed():
        uart.write('F')
        display.show('F')
        sleep(200)
        display.clear()
        sleep(500)  # Debounce delay
    
    # Button B: STOP all motors
    if button_b.is_pressed():
        uart.write('S')
        display.show('S')
        sleep(200)
        display.clear()
        sleep(500)  # Debounce delay
    
    # Logo touch: BACKWARD (if using V2)
    try:
        if pin_logo.is_touched():
            uart.write('B')
            display.show('B')
            sleep(200)
            display.clear()
            sleep(500)
    except:
        pass
    
    sleep(10)
