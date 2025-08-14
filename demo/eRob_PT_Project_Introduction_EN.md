# eRob_PT.cpp EtherCAT Master Program - Project Introduction

> **Language Switch**: [中文版](eRob_PT_功能说明文档.md) | English Version

## Project Overview

**eRob_PT** is a high-performance EtherCAT master control system developed based on the SOEM (Simple Open EtherCAT Master) library, specifically designed for precise servo motor control and system identification. This project integrates advanced motor control algorithms, friction compensation, gravity compensation, and system identification capabilities, providing a complete solution for robotic joint control, precision positioning systems, and other applications.

### Core Features

- **Real-time EtherCAT Communication**: Complete EtherCAT protocol stack implementation based on SOEM library, supporting multi-slave device management and real-time data exchange
- **Multi-mode Motor Control**: Supports various control modes including current loop and speed loop, with flexible switching based on application requirements
- **Intelligent Compensation Algorithms**: Integrated friction compensation and gravity compensation algorithms, significantly improving system control precision
- **System Identification Functions**: Built-in sinusoidal frequency sweep and step response testing, supporting automatic transfer function identification and parameter optimization
- **Real-time Performance Optimization**: CPU isolation optimization for embedded platforms like Raspberry Pi, ensuring communication real-time performance

### Technical Architecture

- **Communication Protocol**: EtherCAT real-time Ethernet protocol
- **Control Platform**: Linux systems (supports Raspberry Pi, PC, etc.)
- **Development Language**: C++
- **Real-time Performance**: Microsecond-level communication cycles
- **Scalability**: Supports multi-slave device cascading

## Usage Instructions

### Compilation and Execution
```bash
git clone git@github.com:ZeroErrControl/eRob_Compensation_Controller.git
cd eRob_Compensation_Controller
mkdir build && cd build
cmake ..
make
sudo ./demo/eRob_PT
```

### Control Operations
- **P key**: Switch control modes
- **W/S keys**: Control speed changes in speed mode
  - W: Increase speed
  - S: Decrease speed

## Disclaimer

⚠️ **Important Safety Reminder**

This program is for learning and research purposes only. All experiments must implement proper safety protection measures. **Please do not use this demo program directly in actual production scenarios**.

### Safety Requirements
- Ensure motors and joints are securely fastened to prevent unexpected movement or tipping
- Avoid dangerous operations in torque mode and parameter identification mode
- Check all connections and protection measures before operation
- Maintain safe distance to avoid injury from high-speed rotation

### Disclaimer Terms
Users are responsible for all risks associated with using this program. Developers are not liable for any losses, injuries, or accidents caused by using this program. Please fully understand the associated risks and take necessary safety measures before use.

> **Related Resources**: For more detailed operation demonstrations and tutorial videos, please refer to: [Douyin@翼之道男](https://www.douyin.com/root/search/%E7%BF%BC%E4%B9%8B%E9%81%93%E7%94%B7?aid=162e4cd5-cc26-4b15-91bf-8564670a63ce&modal_id=7537173963715300634&type=general)

# Software Reproduction Steps

## Step 1: Hardware Connection and Safety Preparation
1. Connect motor, Raspberry Pi, power supply, and communication cables
2. **Safety Precautions**:
   - Motor must be securely mounted on the test bench to prevent tipping
   - No obstructions in motor operating range, keep away from personnel to avoid injury from high-speed rotation

---

## Step 2: Code Download and Compilation
- Obtain project code and complete compilation

---

## Step 3: Raspberry Pi Platform Runtime Preparation (Raspberry Pi 3B only)
- Isolate CPU cores 2 and 3, dedicated to program execution to ensure communication real-time performance

---

## Step 4: Communication Line Testing
1. Ensure motor runs without load, no connections to any load
2. Run the program:
   - Press **P** key to switch control modes, observe if motor rotates normally
   - Switch to speed mode (P key), use **W/S** keys to control motor speed, confirm normal operation
   - Press **B** key to stop the motor

---

## Step 5: Friction Identification
1. Ensure motor runs without load
2. Press **P** key to switch to friction identification mode:
   - Motor accelerates from 0 to 29 rpm, collecting average speed and current data
   - After completing forward rotation, stop and perform reverse rotation test
3. Export `data.txt` file from runtime directory to MATLAB, plot speed-current curve and verify correctness
4. Take average of forward and reverse current for each speed, fill into friction parameter table

---

## Step 6: Verify Friction Compensation Algorithm
1. Install connecting rod
2. Start program, press **P** key to switch to friction compensation mode
3. Manually rotate the connecting rod, observe compensation effect
4. If compensation is insufficient, adjust friction compensation amplification coefficient

---

## Step 7: Gravity Compensation
1. Add load iron plates to the connecting rod
2. Start program and enter speed mode (P key switch)
3. Use **W/S** keys to adjust speed, let connecting rod stop at horizontal position
4. Read current value from logs, fill into gravity compensation parameters
5. **Note**: Need to zero encoder in advance, ensure motor angle is 90° when connecting rod is horizontal

---

## Step 8: Verify Gravity Compensation Effect
1. Run program, press **P** key to switch to friction + gravity compensation mode
2. Manually rotate load and connecting rod, observe compensation effect
3. If compensation is insufficient, manually fine-tune gain coefficients

---

## Step 9: Speed Loop Model Identification
1. Run program, press **P** key to switch to sinusoidal frequency sweep mode
2. Motor drives load for sinusoidal vibration, frequency gradually increases until stopping
3. Export `data.txt` to MATLAB, use system identification toolbox to identify transfer function

---

## Step 10: Model Verification and Parameter Writing
1. Discretize transfer function through c2d method, obtain coefficients and write into program
2. Run program, press **P** key to switch to step test mode
3. After testing, export data to MATLAB, verify model following effect 