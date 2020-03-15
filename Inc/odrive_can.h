#ifndef __ODRIVE_CAN_H
#define __ODRIVE_CAN_H

typedef enum
{
    AXIS_0 = 0,
    AXIS_1 = 1
} Axis_t;

typedef enum
{
    CTRL_MODE_VOLTAGE_CONTROL = 0,
    CTRL_MODE_CURRENT_CONTROL = 1,
    CTRL_MODE_VELOCITY_CONTROL = 2,
    CTRL_MODE_POSITION_CONTROL = 3
} ControlMode_t;

typedef enum
{
    INPUT_MODE_INACTIVE,
    INPUT_MODE_PASSTHROUGH,
    INPUT_MODE_VEL_RAMP,
    INPUT_MODE_POS_FILTER,
    INPUT_MODE_MIX_CHANNELS,
    INPUT_MODE_TRAP_TRAJ,
    INPUT_MODE_CURRENT_RAMP,
    INPUT_MODE_MIRROR,
} InputMode_t;

typedef enum
{
    MSG_CO_NMT_CTRL = 0x000, // CANOpen NMT Message REC
    MSG_ODRIVE_HEARTBEAT,
    MSG_ODRIVE_ESTOP,
    MSG_GET_MOTOR_ERROR, // Errors
    MSG_GET_ENCODER_ERROR,
    MSG_GET_SENSORLESS_ERROR,
    MSG_SET_AXIS_NODE_ID,
    MSG_SET_AXIS_REQUESTED_STATE,
    MSG_SET_AXIS_STARTUP_CONFIG,
    MSG_GET_ENCODER_ESTIMATES,
    MSG_GET_ENCODER_COUNT,
    MSG_SET_CONTROLLER_MODES,
    MSG_SET_INPUT_POS,
    MSG_SET_INPUT_VEL,
    MSG_SET_INPUT_CURRENT,
    MSG_SET_VEL_LIMIT,
    MSG_START_ANTICOGGING,
    MSG_SET_TRAJ_VEL_LIMIT,
    MSG_SET_TRAJ_ACCEL_LIMITS,
    MSG_SET_TRAJ_A_PER_CSS,
    MSG_GET_IQ,
    MSG_GET_SENSORLESS_ESTIMATES,
    MSG_RESET_ODRIVE,
    MSG_GET_VBUS_VOLTAGE,
    MSG_CLEAR_ERRORS,
    MSG_CO_HEARTBEAT_CMD = 0x700, // CANOpen NMT Heartbeat  SEND
} OdriveMsg_t;

typedef struct
{
    uint16_t id; // 11-bit max is 0x7ff, 29-bit max is 0x1FFFFFFF
    uint8_t rtr;
    uint8_t len;
    uint8_t buf[8];
} CanMessage_t;

typedef union {
    float f;
    uint32_t a;
} FloatUnion_t;

FloatUnion_t vbus_voltage;

// struct Config_t
// {
//     ControlMode_t control_mode = CTRL_MODE_POSITION_CONTROL; //see: ControlMode_t
//     InputMode_t input_mode = INPUT_MODE_PASSTHROUGH;         //see: InputMode_t
// } _odrive_can;

uint8_t odrive_can_init(Axis_t axis);
uint8_t odrive_handle_msg(CanMessage_t *msg);
uint8_t odrive_can_send(Axis_t axis, OdriveMsg_t msg);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

// void odrive_can_update_config(Axis_t axis, float pos);
// void odrive_can_enable_control(Axis_t axis, ControlMode_t mode);
// void odrive_can_send_position(Axis_t axis, float pos);

#endif /* __ODRIVE_CAN_H */