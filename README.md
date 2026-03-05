# F5L COMPETITION ALTIMETER & MOTOR LIMITER
A lightweight, high-performance altimeter and motor limiter for F5L (RES) model aircraft gliders, based on the Seeed Studio XIAO ESP32-C3 board.

## 🚀 Features
* **Altitude Sensing:** Uses the Bosch BMP280 sensor via I2C.
* **Dual Cutoff Logic:** Automatically "kills" the motor at **90m height** or **30s duration**.
* **Zoom Capture [optional]:** Records the true peak altitude for 10 seconds after the motor stops, just for practicing, not in the F5L rules. By default this is disabled in config file.
* **Ground Drift Tracking:** Smoothly tracks ground pressure before launch to ensure an accurate "zero".
* **Sensor Protection:** the sensor is isulated on the opposite side of the PCB and covered with dark, open-cell foam. This blocks temperature, prop-wash and sunlight (which interferes with the sensor's internal silicon) while allowing air pressure to pass through, providing smooth and accurate readings.

---

## 🔌 Hardware Pinout

| Function | XIAO Pin | GPIO | Notes |
| :--- | :--- | :--- | :--- |
| **PWM Input** | GPIO 2 | Signal from RC Receiver |
| **PWM Output** | GPIO 3 | Signal to ESC |
| **I2C SDA** | GPIO 6 | BMP280 Data |
| **I2C SCL** | GPIO 7 | BMP280 Clock |

---

## 🛠 Operation Guide

### 1. Preparation
* **Installation:** Mount the device in a location with neutral air pressure (inside the fuselage), try to avoid proximity to any big opening or high-heat components like the ESC or battery. The female connector of the device must be attached to the ESC signal cable, and the male to the receiver, following the normal RC servos wiring convention.

### 2. Competition Flight
1. **Power On:** Connect the main battery to the system with the model resting ON THE GROUND. The device begins tracking ground pressure immediately to have a "sharp" zero.
2. **Launch:** Logic arms automatically when it detects throttle PWM > min threshold.
3. **Automatic Cut:** The motor will stop exactly at 90m or 30s after launch.
4. **Anti-Restart:** For safety and competition rules, the motor will not restart until the device is power-cycled by hand.

---

## 🔍 Troubleshooting

### **The motor won't arm or beeps continuously**
* **Signal Range:** Ensure your transmitter's low-throttle position is calibrated below 1100µs. If it's too high, the limiter stays in "killed" mode for safety.

### **Altitude readings are jumpy or incorrect**
* **Light Interference:** If the sensor foam protection is damaged, sunlight can hit the sensor die and cause massive altitude spikes.
* **Foam Type:** In case of attach a new protection, ensure the foam is "open-cell." Closed-cell packing foam will trap pressure and prevent the sensor from working correctly.

---