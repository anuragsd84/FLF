# Fastest Line Follower Robot – 16 Sensor QTR + Dual ESC Drive

This repository contains the firmware for a **high-speed line follower robot** using:

- ✅ **16-sensor QTR analog array**
- ✅ **Dual ESC-based drive motors**
- ✅ **PID line-following control**
- ✅ **Calibration + On/Off hardware buttons**
- ✅ **Differential motor speed mixing**

The robot is designed for **high-speed track racing** with reliable center estimation using `readLineBlack()` and smooth ESC control ranging **1000–2000 µs**.

## ✅ Features

### 🔹 1. 16-Sensor QTR Line Array
- Analog mode
- Auto-calibration (500 cycles)
- Weighted average position output (0–15000)
- Noise-resistant line reading

### 🔹 2. ESC Motor Control
- Left ESC → Pin **3**
- Right ESC → Pin **5**
- Soft-start during setup
- Safe idle mode (1000 µs)

### 🔹 3. PID Controller
- `Kp = 0.125`
- `Ki = 0.00`
- `Kd = 0.00`

Formula:

```
error = (7500 − position)
output = Kp*P + Ki*I + Kd*D
```

Motor mixing:

```
left  = base + PID
right = base − PID
```

### 🔹 4. Hardware Buttons

| Button | Pin | Function |
|--------|------|-----------|
| Calibration | 31 | Start QTR calibration |
| Start/Stop | 32 | Toggle PID loop |

### 🔹 5. Safety
- Debounced toggle switch  
- Auto-idle on stop  
- ESC protection  
- Constrained speeds  

---

## ✅ Hardware Connections

### **QTR-16 Analog Sensor Pins**
```
A0, A1, A2, A3, A4, A5, A6, A7,
A8, A9, A12, A13, A14, A15, A16, A17
```

### **ESC Pins**
| ESC | Pin |
|-----|------|
| Left | 3 |
| Right | 5 |

### **Buttons**
| Purpose | Pin | Mode |
|----------|------|--------|
| Calibration | 31 | INPUT_PULLUP |
| On/Off | 32 | INPUT_PULLUP |

---

## ✅ Operation

### ✅ 1. Startup
- System powers ON  
- Waits for **calibration button press**
- Rotates for **400 calibration cycles**
- Waits for **Start button press**

### ✅ 2. Running
- Reads 16 sensors  
- Computes line position  
- Runs PID  
- Updates ESCs

### ✅ 3. Stop Mode
- ESC pulses set to **1000 µs**  
- Waits for next toggle  

---

## ✅ Speed Settings

```
baseSpeed = 1300 µs
maxSpeed  = 1450 µs
```

Race recommendation:

```
baseSpeed = 1450–1550
maxSpeed  = 1700–1800
```

---

## ✅ Folder Structure

```
line_follower_16sensor/
│── README.md
│── line_follower.ino
```

---

## ✅ Author
**Anurag Deshmukh**  
Embedded Systems • Robotics • Automation

## ✅ License
Open-source for education & competitions.
