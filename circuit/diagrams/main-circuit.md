![image failed](circuit/diagrams/EV_circuit_diagram.png)
![image failed](images/IMG_4255.png)

# Circuit Specification

PROJECT: SCIOLY EV Circuit Diagram
DOCUMENTATION: Main Circuit Netlist and Pin Specifications
REVISION: 1.0
DATE: 2026-04-13
COMPANY/TEAM: Savannah Country Day School
DRAWN BY: Yisun Yoo
CREATED BY: Yisun Yoo
SOFTWARE: EasyEDA

-------------------------------------------------------------------
# 1. SYSTEM SPECIFICATIONS & HARDWARE NOTES
-------------------------------------------------------------------
* Microcontroller: Arduino Nano ESP32
* Power Source: 12V Battery (powers the Arduino VIN and motor drivers)
* Motor Driver Configuration: 
  - M1 = Left Motor Driver (SimpleFOCMini V1)
  - M2 = Right Motor Driver (SimpleFOCMini V1)
* I2C Multiplexer: TCA9548A (Address: 0x70)
  - All address pins (A0, A1, A2) are tied to GND.
* Motor: 4015 gimbal motor

-------------------------------------------------------------------
# 2. POWER DISTRIBUTION & BUS RAILS
-------------------------------------------------------------------
* 12V Main Rail:
  - Battery (+) ---> Arduino Nano ESP32 (Pin 30: VIN)
  - Battery (+) ---> M1 SimpleFOCMini V1 (PWR+)
  - Battery (+) ---> M2 SimpleFOCMini V1 (PWR+)

* 5V Logic Rail (VBUS):
  - Arduino Nano ESP32 (Pin 28: VBUS) ---> TCA9548A (Pin 24: VCC)
  - Arduino Nano ESP32 (Pin 28: VBUS) ---> M1_CH AS5600 Encoder (Pin 1: VDD5V)
  - Arduino Nano ESP32 (Pin 28: VBUS) ---> M2_CH AS5600 Encoder (Pin 1: VDD5V)

* 3.3V Logic Rail:
  - Arduino Nano ESP32 (Pin 17: 3V3) ---> GY-BNO08X IMU (Pin 1: VCC)
  - Arduino Nano ESP32 (Pin 17: 3V3) ---> M1 SimpleFOCMini V1 (3V3)
  - Arduino Nano ESP32 (Pin 17: 3V3) ---> M2 SimpleFOCMini V1 (3V3)

* Common Ground (GND):
  - Linked across Battery (-),
  - Arduino Nano ESP32 (Pin 29: GND),
  - TCA9548A (Pin 12: GND),
  - TCA9548A (Pins 1, 2, 3: A0/A1/A2),
  - GY-BNO08X (Pin 2: GND),
  - M1_CH AS5600 (Pin 5: GND),
  - M2_CH AS5600 (Pin 5: GND),
  - M1 SimpleFOCMini (GND),
  - M2 SimpleFOCMini (GND),
  - Arduino-Button (Pin 4).

-------------------------------------------------------------------
# 3. DETAILED PIN CONNECTIONS BY COMPONENT
-------------------------------------------------------------------

[A] ARDUINO NANO ESP32
-------------------------------------------------------------------
* Pin 1  (D1/TX)   ---> TCA9548A (Pin 22: SCL)
* Pin 2  (D0/RX)   ---> TCA9548A (Pin 23: SDA)
* Pin 5  (D2)      ---> Arduino-Button (Pin 3)
* Pin 6  (D3)      ---> Tied to BOTH M1 (nRT) and M2 (nRT)
* Pin 7  (D4)      ---> M2 SimpleFOCMini V1 (nSP)
* Pin 8  (D5)      ---> M2 SimpleFOCMini V1 (nFT)
* Pin 9  (D6)      ---> M2 SimpleFOCMini V1 (EN)
* Pin 10 (D7)      ---> M2 SimpleFOCMini V1 (M3)
* Pin 11 (D8)      ---> M2 SimpleFOCMini V1 (M2)
* Pin 12 (D9)      ---> M2 SimpleFOCMini V1 (M1)
* Pin 13 (D10)     ---> M2 SimpleFOCMini V1 (IN3)
* Pin 14 (D11)     ---> M2 SimpleFOCMini V1 (IN2)
* Pin 15 (D12)     ---> M2 SimpleFOCMini V1 (IN1)
* Pin 16 (D13)     ---> M1 SimpleFOCMini V1 (IN1)
* Pin 17 (3V3)     ---> 3.3V Power Rail
* Pin 19 (A0)      ---> M1 SimpleFOCMini V1 (IN2)
* Pin 20 (A1)      ---> M1 SimpleFOCMini V1 (IN3)
* Pin 21 (A2)      ---> M1 SimpleFOCMini V1 (M1)
* Pin 22 (A3)      ---> M1 SimpleFOCMini V1 (M2)
* Pin 23 (A4)      ---> M1 SimpleFOCMini V1 (M3)
* Pin 24 (A5)      ---> M1 SimpleFOCMini V1 (EN)
* Pin 25 (A6)      ---> M1 SimpleFOCMini V1 (nFT)
* Pin 26 (A7)      ---> M1 SimpleFOCMini V1 (nSP)
* Pin 28 (VBUS)    ---> 5V Power Rail
* Pin 29 (GND)     ---> Common Ground Rail
* Pin 30 (VIN)     ---> 12V Battery (+)

[B] TCA9548A I2C MULTIPLEXER (Addr: 0x70)
-------------------------------------------------------------------
* Pin 1  (A0)      ---> GND
* Pin 2  (A1)      ---> GND
* Pin 3  (A2)      ---> GND
* Pin 5  (SD0)     ---> GY-BNO08X IMU (Pin 4: SDA/MISO/TX)
* Pin 6  (SC0)     ---> GY-BNO08X IMU (Pin 3: SCL/SCK/RX)
* Pin 7  (SD1)     ---> M1_CH AS5600 Left Encoder (Pin 4: SDA)
* Pin 8  (SC1)     ---> M1_CH AS5600 Left Encoder (Pin 3: SCL)
* Pin 9  (SD2)     ---> M2_CH AS5600 Right Encoder (Pin 4: SDA)
* Pin 10 (SC2)     ---> M2_CH AS5600 Right Encoder (Pin 3: SCL)
* Pin 12 (GND)     ---> GND
* Pin 22 (SCL)     ---> Arduino Nano ESP32 (Pin 1: D1/TX)
* Pin 23 (SDA)     ---> Arduino Nano ESP32 (Pin 2: D0/RX)
* Pin 24 (VCC)     ---> 5V Rail (VBUS)

[C] GY-BNO08X IMU SENSOR (Channel 0)
-------------------------------------------------------------------
* Pin 1 (VCC)        ---> 3.3V Rail
* Pin 2 (GND)        ---> GND
* Pin 3 (SCL/SCK/RX) ---> TCA9548A (Pin 6: SC0)
* Pin 4 (SDA/MISO/TX)---> TCA9548A (Pin 5: SD0)
* **Pins 5-10 (ADDR, CS, INT, RST, PS1, PS0) are UNCONNECTED.**

[D] M1_CH AS5600 ENCODER - LEFT MOTOR (Channel 1)
-------------------------------------------------------------------
* Pin 1 (VDD5V)  ---> 5V Rail (VBUS)
* Pin 3 (SCL)    ---> TCA9548A (Pin 8: SC1)
* Pin 4 (SDA)    ---> TCA9548A (Pin 7: SD1)
* Pin 5 (GND)    ---> GND
* **Pins 2 (VDD3V3), 6 (PGO), 7 (DIR) are UNCONNECTED.**

[E] M2_CH AS5600 ENCODER - RIGHT MOTOR (Channel 2)
-------------------------------------------------------------------
* Pin 1 (VDD5V)  ---> 5V Rail (VBUS)
* Pin 3 (SCL)    ---> TCA9548A (Pin 10: SC2)
* Pin 4 (SDA)    ---> TCA9548A (Pin 9: SD2)
* Pin 5 (GND)    ---> GND
* **Pins 2 (VDD3V3), 6 (PGO), 7 (DIR) are UNCONNECTED.**

[F] M1 SIMPLEFOCMINI V1 - LEFT MOTOR DRIVER
-------------------------------------------------------------------
* PWR+          ---> 12V Battery (+)
* PWR- / GND    ---> GND
* 3V3           ---> 3.3V Rail
* nRT           ---> Arduino Nano ESP32 (Pin 6: D3)
* nSP           ---> Arduino Nano ESP32 (Pin 26: A7)
* nFT           ---> Arduino Nano ESP32 (Pin 25: A6)
* EN            ---> Arduino Nano ESP32 (Pin 24: A5)
* M3            ---> Arduino Nano ESP32 (Pin 23: A4)
* M2            ---> Arduino Nano ESP32 (Pin 22: A3)
* M1            ---> Arduino Nano ESP32 (Pin 21: A2)
* IN3           ---> Arduino Nano ESP32 (Pin 20: A1)
* IN2           ---> Arduino Nano ESP32 (Pin 19: A0)
* IN1           ---> Arduino Nano ESP32 (Pin 16: D13)
* Motor1 u,v,w  ---> 3-Phase outputs to Left Motor

[G] M2 SIMPLEFOCMINI V1 - RIGHT MOTOR DRIVER
-------------------------------------------------------------------
* PWR+          ---> 12V Battery (+)
* PWR- / GND    ---> GND
* 3V3           ---> 3.3V Rail
* nRT           ---> Arduino Nano ESP32 (Pin 6: D3)
* nSP           ---> Arduino Nano ESP32 (Pin 7: D4)
* nFT           ---> Arduino Nano ESP32 (Pin 8: D5)
* EN            ---> Arduino Nano ESP32 (Pin 9: D6)
* M3            ---> Arduino Nano ESP32 (Pin 10: D7)
* M2            ---> Arduino Nano ESP32 (Pin 11: D8)
* M1            ---> Arduino Nano ESP32 (Pin 12: D9)
* IN3           ---> Arduino Nano ESP32 (Pin 13: D10)
* IN2           ---> Arduino Nano ESP32 (Pin 14: D11)
* IN1           ---> Arduino Nano ESP32 (Pin 15: D12)
* Motor2 u,v,w  ---> 3-Phase outputs to Right Motor

[H] ARDUINO-BUTTON (System Input)
-------------------------------------------------------------------
* Pin 3 ---> Arduino Nano ESP32 (Pin 5: D2)
* Pin 4 ---> GND


