
# 编译和运行
```bash
cd eRob_SOEM_linux
mkdir build
cd build
cmake ..
make all
``` 

# 运行
```bash
sudo taskset -c 2,3 ./eRob_PT
``` 

# EtherCAt通信注意事项
树莓派3B需要隔离两个CPU单独运行程序，以保证ethercat通信的实时性
（1）编辑 /boot/cmdline.txt 文件，在文件末尾添加 isolcpus=2,3
（2）重启
（3）htop 看CPU 2,3占用率应该为0
（4）使用这两个CPU运行程序 sudo taskset -c 2,3 ./eRob_PT
（5）网卡的名称必须为eth0，否则需要修改ec_init("eth0")为对应的名称



# 电机控制
启动后按P键切换控制模式
MC_MODE_OFF = 0X00,    // 空闲
MC_MODE_COMP_FRIC_PT,      // 摩擦力补偿后的电流环模式
MC_MODE_COMP_GRAVITY_PT,      // 重力补偿后的电流环模式
MC_MODE_COMP_PT,      // 重力和摩擦力补偿后的电流环模式
MC_MODE_COMP_PV,      // 重力和摩擦力补偿后的速度环模式
MC_MODE_FRIC_IDEN,    // 摩檫力辨识
MC_MODE_COMP_PV_IDEN_SIN, // 重力和摩擦力补偿后的速度环辨识 正弦模式
MC_MODE_COMP_PV_IDEN_SQUARE, // 重力和摩擦力补偿后的速度环辨识 方波模式
MC_MODE_COMP_PV_IDEN_SIN_2, // 重力和摩擦力补偿后的速度环辨识 正弦模式 加速度限幅
MC_MODE_COMP_PV_IDEN_SQUARE_2, // 重力和摩擦力补偿后的速度环辨识 方波模式 加速度限幅
MC_MODE_NUM,          // 模式的数量

## 其他控制指令
press key to control motor
p: mode change
w: ref     + 10, spd_ref   +1
s: ref     - 10, spd_ref   -1
e: offset  + 10
d: offset  - 10
j: fric_gain    + 0.01
k: fric_gain    - 0.01
n: gravity_gain + 0.01
m: gravity_gain - 0.01
b: stop
q: exit
f: en_fric_comp
c: en_fric_c