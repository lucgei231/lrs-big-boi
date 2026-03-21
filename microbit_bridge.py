# Micro:bit BRIDGE Program - Receives Radio, Sends Serial
# Upload this to SECOND Micro:bit using MicroPython editor
# Connect this Micro:bit to ESP32 with serial cable

from microbit import *
import radio

# Initialize radio (same settings as controller)
radio.on()
radio.config(channel=7, power=7)

# Serial setup
uart.init(baudrate=9600)

# Display startup message
display.show(Image.HAPPY)
sleep(500)
display.clear()
display.show("BRIDGE")

while True:
    # Receive radio messages
    incoming = radio.receive()
    if incoming:
        display.show(incoming[0])  # Show first letter

        # Send via serial to ESP32
        if incoming == 'FORWARD':
            uart.write('F\n')
        elif incoming == 'STOP':
            uart.write('S\n')
        elif incoming == 'BACKWARD':
            uart.write('B\n')

        sleep(200)
        display.clear()

    sleep(10)