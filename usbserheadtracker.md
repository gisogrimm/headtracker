# MPU6050 Serial Head Tracker Documentation

## 1. Overview
This firmware is designed for Arduino boards (e.g., Uno, Nano, Mega) equipped with an **MPU6050** IMU sensor. It functions as a high-performance head tracker that streams orientation and rotation data directly to a computer via a USB Serial connection.

Unlike the previous WiFi-based version, this code uses a binary data protocol for low-latency communication, making it ideal for applications requiring high-speed data transfer, such as virtual reality, robotics, or real-time visualization.

### Key Features
*   **High-Speed Serial Output:** Uses binary data transmission instead of ASCII text for maximum speed.
*   **DMP Processing:** Utilizes the MPU6050's Digital Motion Processor (DMP) for accurate orientation calculations.
*   **Integrated Rotation:** Calculates cumulative rotation (yaw, pitch, roll) based on gyroscope data.
*   **Hardware Calibration:** Supports one-touch gyro calibration via a physical button.

---

## 2. Hardware Requirements
*   **Microcontroller:** Arduino (Uno, Nano, Mega, or similar 5V board).
*   **IMU Sensor:** MPU6050 (Gyroscope & Accelerometer).
*   **Connection:** Standard I2C connection (SDA to A4, SCL to A5 on standard Arduinos).
*   **Calibration Button:** A momentary push button connected to **Pin 15** (A1 on standard Arduino boards) and Ground.

---

## 3. Installation & Setup

### Wiring
Connect the MPU6050 to the Arduino using I2C:
*   **VCC** $\rightarrow$ 5V or 3.3V (Check your sensor module specs)
*   **GND** $\rightarrow$ GND
*   **SCL** $\rightarrow$ Pin A5 (or SCL)
*   **SDA** $\rightarrow$ Pin A4 (or SDA)

Connect the Calibration Button:
*   **Pin 15 (A1)** $\rightarrow$ Button $\rightarrow$ GND

### Software
1.  Install the **Arduino IDE**.
2.  Install the **MPU6050** library by Electronic Cats (or the specific `MPU6050_6Axis_MotionApps20.h` library).
3.  Upload the provided code to your Arduino.
4.  Open the **Serial Monitor** or connect your host application (e.g., Python script, Processing sketch) to the COM port.
5.  **Baud Rate:** Set your serial monitor/application to **115200**.

---

## 4. Calibration Procedure
To ensure accurate head tracking, the gyroscope must be calibrated while the sensor is stationary.

1.  Place the Arduino/MPU6050 on a flat, stable surface.
2.  Press and hold the button connected to **Pin 15**.
3.  The Serial Monitor will display **"C1"**, indicating calibration has started.
4.  Keep the device **perfectly still** for approximately 3-5 seconds.
5.  The Serial Monitor will display **"C0"** followed by a line starting with **"O"** and a list of offset values.
6.  Release the button. The device is now calibrated and ready for use.

---

## 5. Data Protocol (Binary Output)
The device sends a continuous stream of binary data packets. This is not human-readable text; it is designed to be parsed by software.

### Packet Structure
Every data packet consists of **40 bytes** and starts with a 4-byte header.

| Byte Offset | Length | Type | Description |
| :--- | :--- | :--- | :--- |
| 0 | 4 | Char | Header: `"TSCH"` (ASCII) |
| 4 | 4 | `uint32` | Timestamp (`rt_uint`) in microseconds |
| 8 | 4 | `int32` | Quaternion W (scaled by $2^{16}$) |
| 12 | 4 | `int32` | Quaternion X (scaled by $2^{16}$) |
| 16 | 4 | `int32` | Quaternion Y (scaled by $2^{16}$) |
| 20 | 4 | `int32` | Quaternion Z (scaled by $2^{16}$) |
| 24 | 4 | `int32` | Rotation X (scaled by $2^{7}$) |
| 28 | 4 | `int32` | Rotation Y (scaled by $2^{7}$) |
| 32 | 4 | `int32` | Rotation Z (scaled by $2^{7}$) |

### Data Conversion
To interpret the data in your software, you must convert the raw integers back to floating-point numbers.

**1. Timestamp:**
The value is in microseconds. To get seconds:
$$ t_{seconds} = \frac{\text{raw\_timestamp}}{1,000,000} $$

**2. Quaternions:**
The raw values are multiplied by $65,536$ ($2^{16}$). To get the normalized quaternion ($w, x, y, z$):
$$ q_{float} = \frac{\text{raw\_q}}{65536.0} $$

**3. Rotation (Integrated Gyro):**
The raw values are multiplied by $128$ ($2^{7}$). To get the rotation value:
$$ rot_{float} = \frac{\text{raw\_rot}}{128.0} $$

---

## 6. Example Pseudocode (Python)
Here is how you might read this data in Python using `pyserial`:

```python
import serial
import struct

ser = serial.Serial('COM3', 115200) # Change COM3 to your port

HEADER = b'TSCH'
PACKET_SIZE = 40

while True:
    # Wait for the header
    if ser.read(4) == HEADER:
        # Read the remaining 36 bytes
        data = ser.read(36)
        
        # Unpack the binary data
        # Format: 1 unsigned long (timestamp), 8 signed longs (q + rot)
        # '<' means little-endian
        unpacked = struct.unpack('<Lllllllll', data)
        
        timestamp = unpacked[0] / 1000000.0
        qw = unpacked[1] / 65536.0
        qx = unpacked[2] / 65536.0
        qy = unpacked[3] / 65536.0
        qz = unpacked[4] / 65536.0
        rotx = unpacked[5] / 128.0
        roty = unpacked[6] / 128.0
        rotz = unpacked[7] / 128.0
        
        print(f"T: {timestamp:.4f} | Q: {qw:.2f}, {qx:.2f}, {qy:.2f}, {qz:.2f} | Rot: {rotx:.1f}, {roty:.1f}, {rotz:.1f}")
```