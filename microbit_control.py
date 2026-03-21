# Micro:bit Motor Control Program - RADIO (NO PINS NEEDED!)
# Upload this to your micro:bit using the MicroPython editor at https://python.microbit.org

from microbit import *
import radio

# Initialize radio (built-in, no pins, no wires!)
radio.on()

# Display startup message
display.show(Image.HAPPY)
sleep(500)
display.clear()
display.show("READY")

while True:
    # Button A: Move all motors FORWARD
    if button_a.is_pressed():
        radio.send('FORWARD')
        display.show('F')
        sleep(200)
        display.clear()
        sleep(500)  # Debounce delay
    
    # Button B: STOP all motors
    if button_b.is_pressed():
        radio.send('STOP')
        display.show('S')
        sleep(200)
        display.clear()
        sleep(500)  # Debounce delay
    
    # Logo touch: BACKWARD (if using V2)
    try:
        if pin_logo.is_touched():
            radio.send('BACKWARD')
            display.show('B')
            sleep(200)
            display.clear()
            sleep(500)
    except:
        pass
    
    sleep(10)
