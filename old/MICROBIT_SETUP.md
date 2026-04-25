# Micro:bit Arduino Motor Control Setup

## Hardware Connections

Connect your Micro:bit to your Arduino using a serial connection:

### Wiring:
- Micro:bit **TX** (pin 1) → Arduino **RX1** (GPIO 9)
- Micro:bit **GND** → Arduino **GND**
- (Optional) Micro:bit **RX** (pin 2) → Arduino **TX1** (GPIO 10)

**Note:** Your ESP32 (Arduino) uses `Serial1` for Micro:bit communication. Serial1 pins are GPIO9 (RX) and GPIO10 (TX).

## Software Setup

### 1. Arduino Code (Already Updated)
Your `hw870.ino` has been updated with:
- `Serial1.begin(9600)` in setup for Micro:bit UART
- `handleMicrobitCommands()` function in loop to process button commands

### 2. Micro:bit Code
Upload the `microbit_control.py` file to your Micro:bit:

1. Go to **https://python.microbit.org**
2. Create a new project
3. Copy the entire contents of `microbit_control.py` into the editor
4. Click **Download** and transfer to your Micro:bit

## Button Controls

| Button | Action | Display |
|--------|--------|---------|
| **A** | Move all motors FORWARD | Shows "F" |
| **B** | STOP all motors | Shows "S" |
| **Logo** (V2 only) | Move all motors BACKWARD | Shows "B" |

## Command Reference

The Micro:bit sends these serial commands:
- `'F'` - All motors forward
- `'S'` - All motors stop
- `'B'` - All motors backward
- `'L'` - Turn left
- `'R'` - Turn right

(You can extend the Micro:bit code to add L/R controls using A+B button combinations.)

## Troubleshooting

**Motors not responding?**
1. Check serial connections (TX/RX to RX1/TX1)
2. Verify baud rate is 9600 on both devices
3. Open Arduino Serial Monitor to see "Micro:bit command:" messages

**Micro:bit not showing output?**
1. Verify USB connection to Micro:bit
2. Watch for error messages when uploading
3. Try restarting the Micro:bit (press RESET button on back)

## Alternative: Use Different Pins

If GPIO 9/10 are in use, modify the Arduino code to use `Serial2` or software serial instead of `Serial1`.
