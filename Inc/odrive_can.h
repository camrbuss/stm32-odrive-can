#ifndef __ODRIVE_CAN_H
#define __ODRIVE_CAN_H

enum Axis_t
{
    AXIS_0 = 0,
    AXIS_1 = 1
};

// enum ControlMode_t
// {
//     CTRL_MODE_VOLTAGE_CONTROL = 0,
//     CTRL_MODE_CURRENT_CONTROL = 1,
//     CTRL_MODE_VELOCITY_CONTROL = 2,
//     CTRL_MODE_POSITION_CONTROL = 3
// };

// enum InputMode_t
// {
//     INPUT_MODE_INACTIVE,
//     INPUT_MODE_PASSTHROUGH,
//     INPUT_MODE_VEL_RAMP,
//     INPUT_MODE_POS_FILTER,
//     INPUT_MODE_MIX_CHANNELS,
//     INPUT_MODE_TRAP_TRAJ,
//     INPUT_MODE_CURRENT_RAMP,
//     INPUT_MODE_MIRROR,
// };

// struct Config_t
// {
//     ControlMode_t control_mode = CTRL_MODE_POSITION_CONTROL; //see: ControlMode_t
//     InputMode_t input_mode = INPUT_MODE_PASSTHROUGH;         //see: InputMode_t
// } _odrive_can;


void odrive_can_init(uint8_t axis);
void odrive_can_send(uint8_t axis);

// void odrive_can_update_config(Axis_t axis, float pos);
// void odrive_can_enable_control(Axis_t axis, ControlMode_t mode);
// void odrive_can_send_position(Axis_t axis, float pos);

#endif /* __ODRIVE_CAN_H */