# Wiring Guide - Kitchen Safety Monitoring Cube

## Circuit Diagram

```
                             ESP32 DevKit
                    ┌─────────────────────────────┐
                    │                             │
    MPU9250    ────►│ SDA(21)          (2)GPIO2  │◄──── Touch Start Button
    BH1750     ────►│ SCL(22)          (3)GPIO3  │◄──── Touch Inc Button
    SSD1306    ────►│                  (4)GPIO4  │◄──── Touch Dec Button
                    │                  (5)GPIO5  │◄──── Touch Reset Button
                    │                             │
    MQ-135     ────►│ (34)ADC1_CH6    (13)GPIO13 │────► Buzzer
                    │                  (12)GPIO12 │────► Status LED
    DHT22      ────►│ (15)GPIO15                 │
                    │                             │
                    │ 3.3V                    GND │
                    └─────────────────────────────┘
```

## Component Connections

### I2C Bus Connections (3.3V Logic)
All I2C devices share the same SDA and SCL lines with pull-up resistors.

| Device | SDA | SCL | VCC | GND | I2C Address |
|--------|-----|-----|-----|-----|-------------|
| MPU9250 | GPIO21 | GPIO22 | 3.3V | GND | 0x68 |
| BH1750 | GPIO21 | GPIO22 | 3.3V | GND | 0x23 |
| SSD1306 OLED | GPIO21 | GPIO22 | 3.3V | GND | 0x3C |

**Note:** Add 4.7kΩ pull-up resistors on SDA and SCL lines to 3.3V.

### Sensor Connections

#### MQ-135 Gas Sensor
```
MQ-135 Module
┌─────────┐
│ VCC ────┼──► 5V (ESP32 VIN)
│ GND ────┼──► GND
│ AOUT ───┼──► GPIO34 (ADC1_CH6)
│ DOUT    │    (Not connected)
└─────────┘
```

#### DHT22 Temperature & Humidity Sensor
```
DHT22 (Front View)
┌─────────┐
│ 1  2  3 4│
└─────────┘
Pin 1: VCC ──► 3.3V
Pin 2: DATA ─► GPIO15 (with 10kΩ pull-up to 3.3V)
Pin 3: NC (Not connected)
Pin 4: GND ──► GND
```

### User Interface Connections

#### Touch Buttons (Capacitive or Tactile)
```
Each button connection:
┌──────────┐
│  Button  │
│    SW    │
└─┬──────┬─┘
  │      │
GPIO    GND

Internal pull-up enabled in code
```

| Button Function | GPIO Pin | Pull-up |
|-----------------|----------|---------|
| Start/Stop Timer | GPIO2 | Internal |
| Increment (+1 min) | GPIO3 | Internal |
| Decrement (-1 min) | GPIO4 | Internal |
| Reset/Alarm Off | GPIO5 | Internal |

#### Status Indicators

**Buzzer Connection:**
```
GPIO13 ──┬──[220Ω]──┬── Buzzer+
         │           │
        GND      Buzzer-
```

**LED Connection:**
```
GPIO12 ──[330Ω]──┬── LED Anode(+)
                  │
                 LED Cathode(-) ── GND
```

## Power Supply

### Power Requirements
- ESP32: 3.3V logic, 5V input (via USB or VIN)
- Current consumption: ~200-300mA typical, 500mA peak

### Power Distribution
```
USB 5V ──┬── ESP32 VIN
         ├── MQ-135 VCC
         └── 3.3V Regulator ──┬── MPU9250 VCC
                               ├── BH1750 VCC
                               ├── OLED VCC
                               └── DHT22 VCC
```

## PCB Design Considerations

### Layer Stack
- 2-layer PCB recommended
- Top layer: Components and signal routing
- Bottom layer: Ground plane

### Component Placement
1. Keep I2C devices close to minimize trace length
2. Place MQ-135 away from heat sources
3. DHT22 should be exposed to ambient air
4. OLED display on front panel
5. Touch buttons accessible on top/front

### Design Rules
- Trace width: 0.25mm minimum for signals
- Via size: 0.3mm drill, 0.6mm pad
- Clearance: 0.2mm minimum
- Power traces: 0.5mm minimum

## Assembly Instructions

1. **Prepare the PCB or Breadboard**
   - If using breadboard, use quality jumper wires
   - For PCB, check for shorts before assembly

2. **Install Pull-up Resistors**
   - 4.7kΩ resistors on SDA and SCL to 3.3V
   - 10kΩ resistor on DHT22 data pin to 3.3V

3. **Connect I2C Devices**
   - Wire all I2C devices in parallel
   - Keep wires short to reduce interference

4. **Connect Sensors**
   - MQ-135: Ensure proper ventilation
   - DHT22: Mount away from heat sources

5. **User Interface**
   - Mount OLED display at eye level
   - Position buttons for easy access
   - Test buzzer volume before final assembly

6. **Power Connection**
   - Use USB cable or 5V power adapter
   - Add capacitors (100µF, 0.1µF) near ESP32 for stability

## Testing Procedure

1. **Power Test**
   - Check 3.3V and 5V rails
   - Verify no components getting hot

2. **I2C Scan**
   - Run I2C scanner sketch
   - Verify addresses: 0x68, 0x23, 0x3C

3. **Sensor Tests**
   - MQ-135: Read analog values
   - DHT22: Check temp/humidity readings
   - MPU9250: Verify motion detection
   - BH1750: Test light readings

4. **Interface Test**
   - Test each button function
   - Verify OLED display
   - Test buzzer and LED

## Troubleshooting

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| OLED not working | Wrong I2C address | Try 0x3D instead of 0x3C |
| No I2C devices found | Missing pull-ups | Add 4.7kΩ pull-up resistors |
| Erratic sensor readings | Power issues | Add decoupling capacitors |
| Buttons not responsive | Wrong pin mode | Check INPUT_PULLUP setting |
| BLE not connecting | Name conflict | Change BLE_SERVER_NAME |

## Safety Notes

⚠️ **Important Safety Considerations:**
- MQ-135 sensor heater consumes ~800mW
- Ensure proper ventilation for gas sensor
- Use appropriate enclosure (non-flammable)
- Keep device away from water/moisture
- This is a monitoring device, not a certified safety system

## Bill of Materials (BOM)

| Item | Quantity | Part Number | Approximate Cost |
|------|----------|-------------|------------------|
| ESP32 DevKit | 1 | ESP32-DEVKITC-32D | $10 |
| MPU9250 Module | 1 | GY-91 | $8 |
| MQ-135 Module | 1 | MQ-135 | $5 |
| BH1750 Module | 1 | GY-302 | $3 |
| DHT22 | 1 | AM2302 | $5 |
| OLED Display | 1 | SSD1306 128x64 | $5 |
| Buzzer | 1 | Active 5V | $1 |
| LED | 1 | 5mm Red | $0.10 |
| Touch Buttons | 4 | TTP223 | $2 |
| Resistors | Various | Kit | $2 |
| Jumper Wires | 1 set | M-F, M-M | $3 |
| **Total** | | | **~$44** |

## Enclosure Design

### Recommended Dimensions
- Size: 100mm x 80mm x 40mm
- Material: ABS plastic or 3D printed PLA
- Ventilation holes for gas sensor
- Clear window for OLED display
- Access holes for buttons

### 3D Printing Files
- STL files available in `/enclosure` directory
- Print settings: 0.2mm layer height, 20% infill
- Support material needed for button holes