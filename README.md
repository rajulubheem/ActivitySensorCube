# 🏠 Kitchen Safety Monitoring Cube

![Version](https://img.shields.io/badge/version-2.0.0-blue)
![Platform](https://img.shields.io/badge/platform-ESP32-green)
![License](https://img.shields.io/badge/license-MIT-yellow)

## 🎯 Overview

The **Kitchen Safety Monitoring Cube** is an intelligent IoT device designed to prevent kitchen accidents and enhance cooking safety. Using multiple sensors and BLE connectivity, it monitors environmental conditions in real-time and alerts users to potential hazards like forgotten stoves, gas leaks, or unattended cooking.

## ✨ Key Features

### 🔥 Safety Monitoring
- **Gas/Smoke Detection** - MQ-135 sensor monitors air quality and detects dangerous gas levels
- **Temperature Monitoring** - DHT22 tracks ambient temperature to detect overheating
- **Motion Detection** - MPU9250 9-axis sensor detects presence and activity
- **Light Level Sensing** - BH1750 monitors lighting conditions
- **Inactivity Alert** - Warns when no motion is detected during active cooking

### 📱 Smart Connectivity
- **BLE Communication** - Connect multiple sensor cubes via Bluetooth Low Energy
- **Mobile App Integration** - Real-time monitoring through smartphone
- **Multi-Cube Network** - Deploy multiple units for comprehensive coverage

### ⏲️ Interactive Controls
- **Touch Button Interface**
  - Start/Stop cooking timer
  - Increment/Decrement timer (1-minute steps)
  - Reset timer or silence alarms
- **OLED Display** - Real-time status and sensor readings
- **Audio-Visual Alerts** - Buzzer and LED indicators for warnings

## 🛠️ Hardware Components

### Required Components
| Component | Model | Purpose | Quantity |
|-----------|-------|---------|----------|
| Microcontroller | ESP32 DevKit | Main processing unit with WiFi/BLE | 1 |
| Motion Sensor | MPU9250 | 9-axis accelerometer/gyroscope/magnetometer | 1 |
| Gas Sensor | MQ-135 | Air quality and gas detection | 1 |
| Temp/Humidity | DHT22 | Environmental monitoring | 1 |
| Light Sensor | BH1750 | Ambient light measurement | 1 |
| Display | SSD1306 OLED | 128x64 pixel display | 1 |
| Buzzer | Active 5V | Audio alerts | 1 |
| LED | Status LED | Visual indicator | 1 |
| Touch Buttons | Capacitive | User input | 4 |

### Pin Connections

```
ESP32 Pin Connections:
├── I2C Bus (SDA: GPIO21, SCL: GPIO22)
│   ├── MPU9250 (0x68)
│   ├── BH1750 (0x23)
│   └── SSD1306 OLED (0x3C)
├── Digital Pins
│   ├── Touch Start Button: GPIO2
│   ├── Touch Inc Button: GPIO3
│   ├── Touch Dec Button: GPIO4
│   ├── Touch Reset Button: GPIO5
│   ├── DHT22: GPIO15
│   ├── Buzzer: GPIO13
│   └── Status LED: GPIO12
└── Analog Pins
    └── MQ-135: GPIO34 (ADC1)
```

## 📦 Installation

### Prerequisites
- Arduino IDE 1.8.x or newer
- ESP32 Board Package
- Required Libraries (see below)

### Required Libraries
```bash
# Install via Arduino Library Manager:
- Adafruit GFX Library
- Adafruit SSD1306
- Adafruit Sensor
- DHT sensor library
- MPU9250_asukiaaa
- BH1750
```

### Setup Instructions

1. **Clone the Repository**
   ```bash
   git clone https://github.com/rajulubheem/ActivitySensorCube.git
   cd ActivitySensorCube
   ```

2. **Configure Arduino IDE**
   - Add ESP32 board URL to Preferences
   - Install ESP32 board package
   - Select "ESP32 Dev Module" as board

3. **Install Dependencies**
   - Open Library Manager (Tools → Manage Libraries)
   - Install all required libraries listed above

4. **Upload Code**
   - Open `src/Activity_Sensor_Cube.ino`
   - Select correct COM port
   - Click Upload

## 🔧 Configuration

### Threshold Settings
Edit these values in the code to customize sensitivity:

```cpp
#define GAS_THRESHOLD_WARNING   1500   // PPM for warning
#define GAS_THRESHOLD_DANGER    2500   // PPM for danger
#define TEMP_THRESHOLD_WARNING  45.0   // Celsius
#define TEMP_THRESHOLD_DANGER   60.0   // Celsius
#define INACTIVITY_TIMEOUT      300000 // 5 minutes
```

### BLE Configuration
```cpp
#define BLE_SERVER_NAME "KitchenSafetyCube"
// Change UUID if deploying multiple cubes
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
```

## 📱 Mobile App Integration

### BLE Services & Characteristics

| Service | UUID | Description |
|---------|------|-------------|
| Main Service | 4fafc201-1fb5-459e-8fcc-c5c9c331914b | Primary service |
| Status Char | beb5483e-36e1-4688-b7f5-ea07361b26a8 | System status |
| Alert Char | 1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e | Alert notifications |
| Sensor Char | a3c87cd8-7f1a-4b3e-9f9e-2b5d0f0e5a6d | Sensor data JSON |

### Data Format
Sensor data is transmitted as JSON:
```json
{
  "temp": 25.5,
  "hum": 45,
  "gas": 350,
  "light": 250,
  "motion": true
}
```

## 🚨 Alert System

### Alert Levels
1. **Normal** - All parameters within safe ranges
2. **Warning** - One or more parameters approaching threshold
3. **Danger** - Immediate attention required

### Alert Types
- `ALERT_GAS` - High gas/smoke levels detected
- `ALERT_TEMP` - Temperature exceeds safe threshold
- `ALERT_INACTIVITY` - No motion during active cooking
- `ALERT_TIMER` - Cooking timer expired

## 🏗️ Project Structure

```
ActivitySensorCube/
├── src/
│   └── Activity_Sensor_Cube.ino   # Main Arduino code
├── PCB/
│   ├── Base/                      # PCB design files
│   ├── Bottom_3d_view.PNG         # PCB 3D view
│   └── top_3d_view.PNG           # PCB top view
├── docs/
│   └── ESP32_Cube_Safety_Monitor_Board_0.2.docx
├── README.md                       # This file
└── LICENSE                        # MIT License
```

## 🔮 Future Enhancements

- [ ] WiFi connectivity for cloud logging
- [ ] Machine learning for cooking pattern recognition
- [ ] Voice alerts using text-to-speech
- [ ] Integration with smart home systems (Alexa, Google Home)
- [ ] Mobile app with historical data visualization
- [ ] Multi-language support
- [ ] Recipe-based timer presets
- [ ] Smoke vs steam differentiation

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👨‍💻 Author

**Bheema Rajulu**
- GitHub: [@rajulubheem](https://github.com/rajulubheem)

## 🙏 Acknowledgments

- Thanks to the Arduino and ESP32 communities
- Adafruit for excellent sensor libraries
- All contributors who help improve kitchen safety

## 📞 Support

For support, please open an issue in the GitHub repository or contact the author.

---

**⚠️ Safety Notice:** This device is a supplementary safety tool and should not replace standard kitchen safety practices. Always follow proper cooking safety guidelines and never leave cooking unattended for extended periods.