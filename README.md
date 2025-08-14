# eRob_Compensation_Controller

> **Language Switch**: [中文版功能说明文档](demo/eRob_PT_功能说明文档.md) | [English Function Documentation](demo/eRob_PT_Project_Introduction_EN.md)

## Safety Notice

⚠️ **Important Safety Reminder**

This program is for learning and research purposes only. All experiments must implement proper safety protection measures. **Please do not use this demo program directly in actual production scenarios**.

- Ensure motors and joints are securely fastened
- Avoid dangerous operations in torque mode and parameter identification mode
- Check all connections and protection measures before operation
- Maintain safe distance to avoid injury from high-speed rotation

## Project Overview

**eRob_Compensation_Controller** is a high-performance EtherCAT master control system developed based on the SOEM (Simple Open EtherCAT Master) library, specifically designed for ZeroErr eRob robotic rotary actuators. This project integrates intelligent parameter identification, friction compensation, and gravity compensation algorithms for high-precision actuator control.

## Core Features

- **Real-time EtherCAT Communication**: Complete EtherCAT protocol stack implementation based on SOEM library, supporting multi-slave device management and real-time data exchange
- **Multi-mode Motor Control**: Supports various control modes including current loop and speed loop, with flexible switching based on application requirements
- **Intelligent Compensation Algorithms**: Integrated friction compensation and gravity compensation algorithms, significantly improving system control precision
- **System Identification Functions**: Built-in sinusoidal frequency sweep and step response testing, supporting automatic transfer function identification and parameter optimization
- **Real-time Performance Optimization**: CPU isolation optimization for embedded platforms like Raspberry Pi, ensuring communication real-time performance

## Technical Architecture

- **Communication Protocol**: EtherCAT real-time Ethernet protocol
- **Control Platform**: Linux systems (supports Raspberry Pi, PC, etc.)
- **Development Language**: C++
- **Real-time Performance**: Microsecond-level communication cycles
- **Scalability**: Supports multi-slave device cascading

## Quick Start

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

## Control Modes

Press P key after startup to switch control modes:

- **MC_MODE_OFF**: Idle mode
- **MC_MODE_COMP_FRIC_PT**: Current loop mode with friction compensation
- **MC_MODE_COMP_GRAVITY_PT**: Current loop mode with gravity compensation
- **MC_MODE_COMP_PT**: Current loop mode with friction and gravity compensation
- **MC_MODE_COMP_PV**: Speed loop mode with friction and gravity compensation
- **MC_MODE_FRIC_IDEN**: Friction identification mode
- **MC_MODE_COMP_PV_IDEN_SIN**: Speed loop identification with compensation (sinusoidal mode)
- **MC_MODE_COMP_PV_IDEN_SQUARE**: Speed loop identification with compensation (square wave mode)

## Raspberry Pi Configuration

### CPU Isolation (Raspberry Pi 3B only)
To ensure EtherCAT communication real-time performance:

1. Edit `/boot/cmdline.txt`, add `isolcpus=2,3` at the end
2. Restart system
3. Check CPU 2,3 usage with `htop` (should be 0)
4. Run: `sudo taskset -c 2,3 ./eRob_PT`

### Network Card
Network card name must be eth0, otherwise modify `ec_init("eth0")` in code.

## Related Resources

For more detailed operation demonstrations and tutorial videos, please refer to: [Douyin@翼之道男](https://www.douyin.com/root/search/%E7%BF%BC%E4%B9%8B%E9%81%93%E7%94%B7?aid=162e4cd5-cc26-4b15-91bf-8564670a63ce&modal_id=7537173963715300634&type=general)

## License

This project is licensed under the terms specified in the LICENSE file.

---

**Note**: This is a reference implementation for EtherCAT master control systems. Users are responsible for all risks associated with using this program.