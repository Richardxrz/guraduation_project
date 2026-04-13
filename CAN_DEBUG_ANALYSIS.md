# 机械臂CAN通信问题诊断报告

## 问题现象
- 只有J3能通过遥控器控制
- J4, J5, J6能接收数据（有反馈）但无法控制
- temp值不变，J6的curr值稳定在1000左右

## 配置信息

### 电机配置（robot_param_engineer.h）
```
J3: ID=3, CAN2, DJI_M6020
J4: ID=5, CAN2, DJI_M3508
J5: ID=6, CAN2, DJI_M2006
J6: ID=8, CAN2, DJI_M2006
```

### DJI CAN协议
- **0x1FF控制帧格式：**
  - 字节0-1 → 控制ID=5的电机
  - 字节2-3 → 控制ID=6的电机
  - 字节4-5 → 控制ID=7的电机
  - 字节6-7 → 控制ID=8的电机

- **接收ID映射：**
  - 0x201 → CAN_DJI_MEASURE[0] → ID 1
  - 0x202 → CAN_DJI_MEASURE[1] → ID 2
  - 0x203 → CAN_DJI_MEASURE[2] → ID 3
  - ...
  - 0x205 → CAN_DJI_MEASURE[4] → ID 5
  - 0x206 → CAN_DJI_MEASURE[5] → ID 6
  - 0x207 → CAN_DJI_MEASURE[6] → ID 7
  - 0x208 → CAN_DJI_MEASURE[7] → ID 8

## 核心问题分析

### 问题1：J3的ID映射不一致

**在GetMotorMeasure函数中（CAN_receive.c:511-514）：**
```c
case DJI_M6020: {
    const DjiMotorMeasure_t * p_dji_motor_measure =
        GetDjiMotorMeasurePoint(p_motor->can, p_motor->id + 3);
    // J3: id=3, 索引 = 3+3 = 6, 读取 CAN2_DJI_MEASURE[6]
    // 但CAN2_DJI_MEASURE[6]对应的是接收ID 0x207（硬件ID=7）！
```

**如果J3的硬件ID真的是3：**
- 接收：硬件发送0x203 → 存储到CAN2_DJI_MEASURE[2]
- 读取：代码从CAN2_DJI_MEASURE[6]读数据（读的是ID=7的数据）
- **结果：读取的数据与实际电机不匹配！**

**如果J3能正常工作，说明：**
1. 硬件ID实际是7（不是配置的3）
2. 或者CAN2上恰好有一个ID=7的电机连接到J3

### 问题2：发送命令的参数顺序

**ArmSendCmdDebug (mechanical_arm_engineer.c:1015-1021)：**
```c
CanCmdDjiMotor(
    ARM_DM_CAN, 0x1FF,
    MA.joint_motor[J4].set.value,  // 参数1 → 发给ID 5
    MA.joint_motor[J5].set.value,  // 参数2 → 发给ID 6
    MA.joint_motor[J3].set.value,  // 参数3 → 发给ID 7
    MA.joint_motor[J6].set.value); // 参数4 → 发给ID 8
```

**如果硬件ID配置正确（J3=3, J4=5, J5=6, J6=8）：**
- J3的控制量发送到ID 7 → **错误！应该发给ID 3**
- J4的控制量发送到ID 5 → 正确 ✓
- J5的控制量发送到ID 6 → 正确 ✓
- J6的控制量发送到ID 8 → 正确 ✓

但ID 3应该使用0x200控制帧，不是0x1FF：
- 0x200控制ID 1-4
- 0x1FF控制ID 5-8

## 诊断步骤

### 步骤1：确认实际硬件ID
使用Robomaster调试助手连接CAN2，查看实际响应的电机ID：
1. 发送0x1FF控制帧，看哪些ID的电机有响应
2. 记录J3, J4, J5, J6实际对应的硬件ID

### 步骤2：检查CAN接收数据
在UpdateMotorStatus函数后添加断点，查看：
```c
MECHANICAL_ARM.joint_motor[J3].fdb.pos
MECHANICAL_ARM.joint_motor[J3].fdb.temp
MECHANICAL_ARM.joint_motor[J3].can  // 应该是2
MECHANICAL_ARM.joint_motor[J3].id   // 应该是3
```

### 步骤3：检查CAN2的数据数组
在CAN接收中断后查看CAN2_DJI_MEASURE数组：
```c
CAN2_DJI_MEASURE[2]  // 对应ID 3的电机数据
CAN2_DJI_MEASURE[4]  // 对应ID 5的电机数据
CAN2_DJI_MEASURE[5]  // 对应ID 6的电机数据
CAN2_DJI_MEASURE[6]  // 对应ID 7的电机数据
CAN2_DJI_MEASURE[7]  // 对应ID 8的电机数据
```
看哪些位置有实际数据更新。

## 可能的修复方案

### 方案A：如果J3硬件ID确实是7
**修改robot_param_engineer.h:133：**
```c
// 修改前
#define JOINT_MOTOR_3_ID 3  // DJi 相当于2006或者3508的7

// 修改后
#define JOINT_MOTOR_3_ID 7  // DJI M6020, 实际硬件ID
```

**同时修改CAN_receive.c:511-514的6020映射：**
```c
// 修改前
case DJI_M6020: {
    const DjiMotorMeasure_t * p_dji_motor_measure =
        GetDjiMotorMeasurePoint(p_motor->can, p_motor->id + 3);

// 修改后（与其他DJI电机一致）
case DJI_M6020: {
    const DjiMotorMeasure_t * p_dji_motor_measure =
        GetDjiMotorMeasurePoint(p_motor->can, p_motor->id - 1);
```

### 方案B：如果J3硬件ID确实是3
**需要同时修改：**

1. **robot_param_engineer.h** - 保持J3 ID=3

2. **CAN_receive.c:511-514** - 修改6020的映射逻辑
```c
case DJI_M6020: {
    const DjiMotorMeasure_t * p_dji_motor_measure =
        GetDjiMotorMeasurePoint(p_motor->can, p_motor->id - 1);  // 改为id-1
```

3. **mechanical_arm_engineer.c** - 修改发送命令，分两次发送
```c
void ArmSendCmdDebug(void)
{
    // DM电机控制 (J1, J2)
    DmMitCtrlVelocity(&MECHANICAL_ARM.joint_motor[J1], J1_KD_FOLLOW);
    DmMitCtrlVelocity(&MECHANICAL_ARM.joint_motor[J2], J2_KD_FOLLOW);
    delay_us(DM_DELAY);

    // DJI电机控制 - 分两次发送
    // 第一次：0x200控制ID 1-4（J3在第3个位置，对应ID 3）
    CanCmdDjiMotor(
        ARM_DM_CAN, 0x200,
        0,
        0,
        MA.joint_motor[J3].set.value,  // 参数3 → ID 3
        0);

    delay_us(10);  // 两次发送间隔

    // 第二次：0x1FF控制ID 5-8
    CanCmdDjiMotor(
        ARM_DM_CAN, 0x1FF,
        MA.joint_motor[J4].set.value,  // 参数1 → ID 5
        MA.joint_motor[J5].set.value,  // 参数2 → ID 6
        0,                              // 参数3 → ID 7（未使用）
        MA.joint_motor[J6].set.value); // 参数4 → ID 8
}
```

## 建议的诊断代码

在mechanical_arm_engineer.c的UpdateMotorStatus函数后添加：

```c
// 临时诊断代码
static uint32_t debug_count = 0;
if (++debug_count % 1000 == 0) {  // 每秒打印一次
    // 打印电机反馈数据
    printf("J3: id=%d, can=%d, temp=%.1f, pos=%.3f, offline=%d\n",
           MA.joint_motor[J3].id,
           MA.joint_motor[J3].can,
           MA.joint_motor[J3].fdb.temp,
           MA.joint_motor[J3].fdb.pos,
           MA.joint_motor[J3].offline);

    printf("J4: id=%d, can=%d, temp=%.1f, pos=%.3f, offline=%d\n",
           MA.joint_motor[J4].id,
           MA.joint_motor[J4].can,
           MA.joint_motor[J4].fdb.temp,
           MA.joint_motor[J4].fdb.pos,
           MA.joint_motor[J4].offline);

    printf("J5: id=%d, can=%d, temp=%.1f, pos=%.3f, offline=%d\n",
           MA.joint_motor[J5].id,
           MA.joint_motor[J5].can,
           MA.joint_motor[J5].fdb.temp,
           MA.joint_motor[J5].fdb.pos,
           MA.joint_motor[J5].offline);

    printf("J6: id=%d, can=%d, temp=%.1f, curr=%d, offline=%d\n",
           MA.joint_motor[J6].id,
           MA.joint_motor[J6].can,
           MA.joint_motor[J6].fdb.temp,
           MA.joint_motor[J6].fdb.curr,
           MA.joint_motor[J6].offline);

    // 打印控制量
    printf("Control: J3=%d, J4=%d, J5=%d, J6=%d\n",
           MA.joint_motor[J3].set.value,
           MA.joint_motor[J4].set.value,
           MA.joint_motor[J5].set.value,
           MA.joint_motor[J6].set.value);
}
```

## 下一步行动
1. 先用调试助手确认实际硬件ID
2. 根据实际ID选择对应的修复方案
3. 验证修复后的效果