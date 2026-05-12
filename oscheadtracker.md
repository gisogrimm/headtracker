# ESP8266 Head Tracker & EOG Sensor Documentation

## 1. Overview
This firmware is designed for an ESP8266-based microcontroller (e.g., Wemos D1 R2) equipped with an **MPU6050** IMU sensor and **ADS1115** ADCs. It functions as a wireless head tracker and biosignal acquisition device (EOG/EMG) streaming data via the Open Sound Control (OSC) protocol over WiFi.

### Key Features
*   **Head Tracking:** Streams Quaternion orientation and integrated rotation data from the MPU6050.
*   **Biosignal Acquisition:** Reads analog data via ADS1115 ADCs (suitable for EOG, EMG, or other sensors).
*   **WiFi Connectivity:** Creates a default Access Point (AP) but can connect to an existing WiFi network via OSC commands.
*   **OSC Control:** Fully configurable streaming rates, IP addresses, and ports via OSC messages.

---

## 2. Hardware Requirements
*   **Microcontroller:** ESP8266 (e.g., Wemos D1 Mini, Wemos D1 R2).
*   **IMU Sensor:** MPU6050 (Gyroscope & Accelerometer).
*   **ADC:** One or two ADS1115 modules (16-bit ADC).
*   **Calibration Button:** A button connected to Pin **D8** (GPIO 15) used for gyro calibration.

---

## 3. Initial Setup & Connection

### Default Mode (Access Point)
Upon powering the device, it creates its own WiFi Hotspot:
1.  **SSID:** `head-XXXXXX` (where XXXXXX is the last 6 characters of the MAC address).
2.  **Password:** `headtracker`
3.  **IP Address:** `192.168.100.1`

### Connecting to the Device
1.  Connect your computer or mobile device to the `head-XXXXXX` WiFi network.
2.  Ensure your OSC software (receiver) is listening on UDP port **9999** (or the port you intend to configure).

---

## 4. OSC Commands

The device listens for incoming OSC commands on port **9999** to configure connections and settings.

### WiFi Configuration

#### `/wlan/connect`
Connects the device to an external WiFi router.
*   **Arguments:**
    1.  `String` SSID (Network name)
    2.  `String` Password
    3.  `String` Remote IP (IP address of the computer running your OSC software)
    4.  `Integer` Port (UDP port to send data to)
*   **Example:** `/wlan/connect "MyRouter" "password123" "192.168.1.5" 8000`
*   **Note:** Sending this command with **no arguments** resets the device back to Access Point mode.

### Data Stream Configuration

#### `/connect [port] [path]`
Starts streaming the main Head Tracking data.
*   **Arguments:**
    1.  `Integer` Port (Destination UDP port)
    2.  `String` Path (OSC address to send data to, e.g., `/headtrack`)
*   **Note:** The IP address used is the one of the last device that sent a command to the ESP.

#### `/disconnect`
Stops the Head Tracking data stream.

#### `/raw/connect [port] [path]`
Starts streaming raw sensor data (Accelerometer, Gyroscope, and DMP processed acceleration).
*   **Arguments:**
    1.  `Integer` Port
    2.  `String` Path (Base path for raw data)

#### `/raw/disconnect`
Stops the raw data stream.

#### `/eog/connect [port] [path]`
Starts streaming data from the ADS1115 ADC (EOG/Biosignals).
*   **Arguments:**
    1.  `Integer` Port
    2.  `String` Path

#### `/eog/disconnect`
Stops the ADC data stream.

### Sensor Configuration

#### `/eog/srate [rate]`
Sets the sampling rate of the ADS1115 ADC.
*   **Argument:** `Integer` rate (Samples per second).
*   **Ranges:**
    *   < 16: 8 SPS
    *   < 32: 16 SPS
    *   < 64: 32 SPS
    *   < 128: 64 SPS (Default)
    *   < 250: 128 SPS
    *   < 475: 250 SPS
    *   < 860: 475 SPS
    *   \> 860: 860 SPS

#### `/eog/urange [voltage]`
Sets the voltage range of the ADS1115.
*   **Argument:** `Float` voltage (Max voltage expected).
*   **Ranges:**
    *   < 0.257V: ±0.256V
    *   < 0.513V: ±0.512V
    *   < 1.025V: ±1.024V

#### `/eog/adcmode [mode]`
Sets the ADC input mode.
*   **Argument:** `Integer` mode
    *   `0`: Single Shot mode (Reads channels 0-1 and 2-3 sequentially).
    *   `1`: Continuous mode (Reads channel 0-1).
    *   `2`: Continuous mode (Reads channel 2-3).
    *   `3`: Continuous mode (Reads channel 2-3 on **both** ADC1 and ADC2).

### Calibration

#### `/calib`
Triggers the Gyroscope calibration routine.
*   **Behavior:** Resets the current rotation values and calculates new gyro offsets based on the sensor's current position. Keep the sensor **still** during calibration.

---

## 5. Output Data Format

The device sends data via OSC bundles. The format depends on which stream is active.

### 1. Head Tracking Stream (`/connect`)
*   **Address:** User defined (e.g., `/headtrack`).
*   **Arguments (8 Floats):**
    1.  `rt`: Time in seconds.
    2.  `q.w`: Quaternion W.
    3.  `q.x`: Quaternion X.
    4.  `q.y`: Quaternion Y.
    5.  `q.z`: Quaternion Z.
    6.  `rotx`: Integrated Rotation X.
    7.  `roty`: Integrated Rotation Y.
    8.  `rotz`: Integrated Rotation Z.

### 2. EOG/ADC Stream (`/eog/connect`)
*   **Address:** User defined.
*   **Arguments (3 Floats):**
    1.  `rt`: Time in seconds.
    2.  `U1`: Voltage from ADC Channel 1 (Volts).
    3.  `U2`: Voltage from ADC Channel 2 (Volts).

### 3. Raw Sensor Stream (`/raw/connect`)
This stream sends four separate OSC messages per cycle:

**A. Raw IMU Data**
*   **Address:** User defined.
*   **Arguments (7 Integers/Floats):**
    1.  `rt`: Time.
    2.  `ax`: Accel X.
    3.  `ay`: Accel Y.
    4.  `az`: Accel Z.
    5.  `gx`: Gyro X.
    6.  `gy`: Gyro Y.
    7.  `gz`: Gyro Z.

**B. DMP Acceleration**
*   **Address:** `/dmp/accel`
*   **Arguments:** `rt`, `x`, `y`, `z`

**C. DMP Linear Acceleration**
*   **Address:** `/dmp/accelLinear`
*   **Arguments:** `rt`, `x`, `y`, `z` (Acceleration with gravity removed).

**D. DMP World Linear Acceleration**
*   **Address:** `/dmp/accelLinearInWorld`
*   **Arguments:** `rt`, `x`, `y`, `z` (Linear acceleration in world frame coordinates).

---

## 6. Calibration Procedure
To ensure accurate head tracking, calibrate the gyroscope before use:

1.  Place the device on a flat, stable surface.
2.  Press the button connected to **Pin 15 (D8)**.
3.  The Serial Monitor will print "C1".
4.  Wait approximately 3-5 seconds without moving the device.
5.  The Serial Monitor will print "C0" followed by offset values.
6.  Calibration is now complete.

You can also trigger this remotely via OSC by sending `/calib`.