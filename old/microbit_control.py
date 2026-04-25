# Micro:bit Motor Control Program - RADIO BRIDGE
# Uses built-in radio to send to another Micro:bit
# That second Micro:bit then sends serial to ESP32

from microbit import *
import radio

# Initialize built-in radio
radio.on()
radio.config(channel=7, power=7, address=0x75626974, group=1)

# Display startup message
display.show(Image.HAPPY)
sleep(500)
display.clear()
display.show("BRIDGE")

while True:
    # Button A: Move all motors FORWARD
    if button_a.is_pressed():
        radio.send('FORWARD')
        display.show('F')
        sleep(200)
        display.clear()
        sleep(500)

    # Button B: STOP all motors
    if button_b.is_pressed():
        radio.send('STOP')
        display.show('S')
        sleep(200)
        display.clear()
        sleep(500)

    # Logo touch: BACKWARD
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
