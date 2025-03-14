# Setting Up PlatformIO for SparkFun Artemis Nano

## Motivation

- Preference for VSCode over the Arduino IDE.
- **PlatformIO significantly compiles faster** than the Arduino IDE, saving development time.

This guide documents the steps taken to configure **PlatformIO** for the **SparkFun Artemis Nano** board, based on discussions from [this GitHub thread](https://github.com/platformio/platformio-core/issues/2709).

## Prerequisites
Before starting, ensure you have:

- **PlatformIO installed** (either through VS Code or CLI).
- The **PlatformIO extension** installed in VS Code (if using the IDE).

### **1️⃣ Install PlatformIO Extension (VS Code)**
If you're using **VS Code**, install the PlatformIO extension:
1. Open **VS Code**.
2. Go to the **Extensions Marketplace** (`Ctrl + Shift + X` / `Cmd + Shift + X` on macOS).
3. Search for **"PlatformIO IDE"**.
4. Click **Install**.

For CLI installation, follow the [official instructions](https://docs.platformio.org/en/latest/installation.html).

---

### **2️⃣ Create and Configure `platformio.ini` for Artemis Nano**
The following `platformio.ini` file ensures compatibility with the **Artemis Nano** and includes the required libraries:

```ini
[env:SparkFun_RedBoard_Artemis_Nano]
platform = apollo3blue
board = SparkFun_Artemis_Nano
framework = arduino
platform_packages = framework-arduinoapollo3@https://github.com/sparkfun/Arduino_Apollo3#v2.2.1

lib_deps = 
    arduino-libraries/ArduinoBLE@^1.3.7
    sparkfun/SparkFun VL53L1X 4m Laser Distance Sensor@1.2.12
```

---

### **3️⃣ Install Required Dependencies**
Once you've created your `platformio.ini` file, install the necessary dependencies by running:
```sh
pio lib install
```
This ensures the correct libraries are available before compiling.

---

### **4️⃣ Build & Upload Code to Artemis Nano**
#### **💡 Build the Project**
To compile your code, run:
```sh
pio run
```

#### **📡 Upload to Artemis Nano**
To upload the firmware:
```sh
pio run --target upload
```

