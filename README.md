# eRob_Compensation_Controller

> **Language Switch**: [中文版功能说明文档](demo/eRob_PT_功能说明文档.md) | [English Function Documentation](demo/eRob_PT_Project_Introduction_EN.md)

---

## ⚠️ Safety Notice

This program is intended for **learning and research purposes only**.  
Do **not** use this demo program directly in production systems.  

Before operation, always:

- Ensure motors and joints are securely fastened  
- Avoid dangerous operations in torque mode and parameter identification mode  
- Check all connections and safety protection measures  
- Maintain a safe distance to avoid injury from high-speed rotation  

---

## Project Overview

**eRob_Compensation_Controller** is a high-performance EtherCAT master control system developed with the SOEM (Simple Open EtherCAT Master) library, specifically designed for **ZeroErr eRob robotic rotary actuators**.  

The system integrates intelligent parameter identification, friction compensation, and gravity compensation algorithms for **high-precision actuator control**.

---

## Core Features

- **Real-time EtherCAT Communication**  
  Full SOEM-based EtherCAT master with multi-slave management and real-time data exchange  

- **Multi-mode Motor Control**  
  Supports current loop and speed loop with flexible switching  

- **Intelligent Compensation Algorithms**  
  Integrated friction and gravity compensation for enhanced precision  

- **System Identification Functions**  
  Built-in sinusoidal frequency sweep and step response testing for parameter optimization  

- **Real-time Performance Optimization**  
  CPU isolation tuning for embedded Linux (e.g. Raspberry Pi)  

---

## Technical Architecture

- **Protocol**: EtherCAT (real-time Ethernet)  
- **Control Platform**: Linux (PC, Raspberry Pi, etc.)  
- **Language**: C++  
- **Performance**: Microsecond-level communication cycles  
- **Scalability**: Multi-slave cascading supported  

---

## Quick Start

### 1. Clone and Build
```bash
git clone git@github.com:ZeroErrControl/eRob_Compensation_Controller.git
cd eRob_Compensation_Controller
mkdir build && cd build
cmake ..
make
````

### 2. Prepare Network Interface

Replace `enp0s31f6` with the NIC connected to the EtherCAT bus:

```bash
sudo ip link set enp0s31f6 down
sudo ip addr flush dev enp0s31f6
sudo ip link set enp0s31f6 up
sudo ethtool -K enp0s31f6 tso off gso off gro off lro off rx off tx off sg off
```

### 3. Run the Demo

Run with an environment variable:

```bash
sudo env EC_IFACE=enp0s31f6 ./demo/eRob_PT
```

Or pass the NIC as argument (if supported):

```bash
sudo ./demo/eRob_PT enp0s31f6
```

If no interface is specified, the default is **`enp5s0`**.

### 4. Verify Actuator Connection

Expected output on startup:

```
__________STEP 1__________________
EtherCAT master initialized successfully on: enp0s31f6
3 slaves found and configured.
_________________________________
All slaves in OPERATIONAL. Actuators are ready.
```

### 5. Control Operations

* **P** → Switch control modes
* **W** → Increase speed (in speed mode)
* **S** → Decrease speed (in speed mode)

⚠️ **Safety Reminder**: Keep a safe distance from actuators during operation.

---

## Control Modes

Switch modes using the **P** key:

* **MC_MODE_OFF** → Idle
* **MC_MODE_COMP_FRIC_PT** → Current loop + friction compensation
* **MC_MODE_COMP_GRAVITY_PT** → Current loop + gravity compensation
* **MC_MODE_COMP_PT** → Current loop + friction + gravity compensation
* **MC_MODE_COMP_PV** → Speed loop + friction + gravity compensation
* **MC_MODE_FRIC_IDEN** → Friction identification mode
* **MC_MODE_COMP_PV_IDEN_SIN** → Speed loop identification (sinusoidal)
* **MC_MODE_COMP_PV_IDEN_SQUARE** → Speed loop identification (square wave)

---

## Raspberry Pi Notes

### CPU Isolation (Raspberry Pi 3B)

For improved real-time EtherCAT performance:

1. Edit `/boot/cmdline.txt`, add `isolcpus=2,3`
2. Reboot
3. Confirm CPU 2/3 idle in `htop`
4. Run with CPU affinity:

   ```bash
   sudo taskset -c 2,3 ./demo/eRob_PT
   ```

### Network Card

Some builds expect the NIC to be named `eth0`.
If using another name, set `EC_IFACE` or adjust the code.

---

## Related Resources

* Tutorial videos and demos: [Douyin@翼之道男](https://www.douyin.com/root/search/%E7%BF%BC%E4%B9%8B%E9%81%93%E7%94%B7?aid=162e4cd5-cc26-4b15-91bf-8564670a63ce&modal_id=7537173963715300634&type=general)

---

## License

This project is licensed under the terms specified in the [LICENSE](LICENSE) file.

---

**Disclaimer**: This is a **reference implementation** for EtherCAT master control. Users are responsible for all risks associated with use.