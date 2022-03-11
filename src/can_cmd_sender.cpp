/*
 *  Copyright (c) 2017, Acer Inc.
 *  All rights reserved.
 */

#include <autoware_can_msgs/CANInfo.h>
#include <ros/ros.h>

// CAN_DEV_ID_SOCKETCAN
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>


#include <netinet/in.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "vehicle.h"

static void setVehicleGear(void);

vehicle_cmd_t v_cmd;


xgene_can_cmd_t xgeneCan_cmd;

void xgeneCan_cmd_reset()
{
    xgeneCan_cmd.shift = XGENE_GEAR_N;
    xgeneCan_cmd.accel_stroke = 0;
    xgeneCan_cmd.brake_stroke = 0;
    xgeneCan_cmd.steering_angle = XGENE_CAN_STEERING_OFFSET;
    xgeneCan_cmd.light = 0;
}


void cmd_reset()
{
    v_cmd.linear_x = 0;
    v_cmd.angular_z = 0;
    v_cmd.shift = GEAR_N;
    v_cmd.accel_stroke = 0;
    v_cmd.brake_stroke = 0;
    v_cmd.steering_angle = 0;
    v_cmd.light = 0;
}

void modeCMDCallback(const tablet_socket_msgs::mode_cmd &mode)
{
    if (mode.mode == -1 || mode.mode == 0) {
        cmd_reset();
    }

    v_cmd.modeValue = mode.mode;
    printf("[ROS CMD] mode cmd: %d\n", v_cmd.modeValue);
}

void gearCMDCallback(const tablet_socket_msgs::gear_cmd &gear)
{
    v_cmd.shift = gear.gear;
    printf("[ROS CMD] gear cmd: %d\n", v_cmd.shift);
}

void twistCMDCallback(const geometry_msgs::TwistStamped &msg)
{
    v_cmd.linear_x = msg.twist.linear.x;
    v_cmd.angular_z = msg.twist.angular.z;
    // printf("[ROS CMD] twist cmd, x:%f,z:%f\n", v_cmd.linear_x, v_cmd.angular_z);
}

void steerCMDCallback(const autoware_msgs::SteerCmd &steer)
{
    v_cmd.steering_angle = steer.steer;
    printf("[ROS CMD] steering_angle cmd: %d\n", v_cmd.steering_angle);
}

void accellCMDCallback(const autoware_msgs::AccelCmd &accell)
{
    v_cmd.accel_stroke = accell.accel;
    printf("[ROS CMD] accel_stroke cmd: %d\n", v_cmd.accel_stroke);
}

void brakeCMDCallback(const autoware_msgs::BrakeCmd &brake)
{
    v_cmd.brake_stroke = brake.brake;
    printf("[ROS CMD] brake_stroke cmd: %d Time=%lld\n", v_cmd.brake_stroke, getTime());
}

static bool auto_setVehicleGear(void)
{
    static bool change_gear_flag = false;
    static int count_down = 0;
    float target_speed = v_cmd.linear_x / 2.4; // Fix the wrong velocity 2022/03/06 by Kenny
    char current_gear = get_vInfo_gear();
    float current_speed = get_vInfo_velocity();
    float cmd_speed = target_speed;

    if ((current_speed != 0.0) && ((current_gear == GEAR_F && cmd_speed < 0.0) || (current_gear == GEAR_R && cmd_speed > 0.0))) {
        /*
          Auto stop for the case :Forward -> reverse or reverse -> forward
         */

        // setup flags
        change_gear_flag = true;
        if (count_down == 0)
            count_down = CHANGE_GEAR_COUNTDOWN;

        // slow down than stop the car
        if (current_speed > 1.0)
            cmd_speed = current_speed / 2;
        else
            cmd_speed = 0.0;
        PedalControl(current_speed, cmd_speed);
    }

    if (change_gear_flag) {
        if (current_speed == 0.0 && count_down > 0) {
            // wait for 1 sec, before change gear.
            v_cmd.shift = GEAR_N;
            setVehicleGear();
            count_down--;
        } else if (current_speed == 0.0 && count_down <= 0) {
            // clear flags, gear will be changed in next round.
            count_down = 0;
            change_gear_flag = false;
        }
        return true;
    }

    if (target_speed < 0.0) {
        v_cmd.shift = GEAR_R;
    } else if (target_speed == 0.0) {
        v_cmd.shift = GEAR_N;
    } else {
        v_cmd.shift = GEAR_F;
    }
    return false;
    // printf("[ROS CMD] auto_setVehicleGear speed: %f, gear: %d\n", speed, v_cmd.shift);
}

static void setVehicleGear()
{
    char current_gear = get_vInfo_gear();
    char cmd_gear = v_cmd.shift;
    float speed = get_vInfo_velocity();
    static uint8_t msg[1];

    if (cmd_gear == current_gear && v_config.vendor_ID != VENDOR_ID_XGENE) {
        msg[0] = current_gear;
        canbus_write(CAN_ID_CMD_SHIFT, (char *) msg, sizeof(msg));
    } else if ((cmd_gear != current_gear) && (speed == 0.0) && (v_config.vendor_ID != VENDOR_ID_XGENE)) {
        // never change the gear when driving!
        // set Acer golf CAN gear
        msg[0] = cmd_gear;
        canbus_write(CAN_ID_CMD_SHIFT, (char *) msg, sizeof(msg));
    } else if ((cmd_gear != current_gear) && (speed == 0.0) && (v_config.vendor_ID == VENDOR_ID_XGENE)) {
        // never change the gear when driving!
        // set xgene mule car CAN gear
        switch (cmd_gear) {
        case GEAR_N:
            xgeneCan_cmd.shift = XGENE_GEAR_N;
            break;
        case GEAR_R:
            xgeneCan_cmd.shift = XGENE_GEAR_R;
            break;
        case GEAR_F:
            xgeneCan_cmd.shift = XGENE_GEAR_D;
            break;
        }
    }
}

static void SteeringControl(float cmd_steering_angle)
{
    // TODO TODO TODO FIXME: positive angle == left
    int16_t int16_angle = (int16_t) cmd_steering_angle;
    static char msg[2];

    if (v_config.vendor_ID == VENDOR_ID_XGENE) {
        xgeneCan_cmd.steering_angle = cmd_steering_angle + XGENE_CAN_STEERING_OFFSET;
    } else {
        // Byte order: Big-endian
        // high byte
        msg[0] = (int16_angle >> 8) & 0x00ff;

        // low byte
        msg[1] = int16_angle & 0x00ff;
        canbus_write(CAN_ID_CMD_ANGLE, msg, sizeof(msg));
    }
}

static void setVehicleDrv_control()
{
#define WHEEL_TO_STEERING (v_config.STEERING_ANGLE_MAX / v_config.WHEEL_ANGLE_MAX)
    static float pre_cmd_steering_angle = 0.0;
    float cmd_velocity = v_cmd.linear_x / 2.4; // Fix the wrong velocity 2022/03/06 by Kenny
    float cmd_steering_angle;
    float current_velocity = get_vInfo_velocity();

    if (fabs(v_cmd.linear_x) < 0.1) {  // just avoid divided by zero.
        // cmd_steering_angle = get_vInfo_steering_angle();
        cmd_steering_angle = pre_cmd_steering_angle;
    } else {
        double phi_angle_pi = (v_cmd.angular_z / v_cmd.linear_x);
        double wheel_angle_pi = phi_angle_pi * v_config.WHEEL_BASE;
        double wheel_angle = (wheel_angle_pi / M_PI) * 180.0;
        cmd_steering_angle = wheel_angle * WHEEL_TO_STEERING;


        // Limit the steering angle
        if (cmd_steering_angle < -v_config.STEERING_ANGLE_MAX)
            cmd_steering_angle = -v_config.STEERING_ANGLE_MAX;
        if (cmd_steering_angle > v_config.STEERING_ANGLE_MAX)
            cmd_steering_angle = v_config.STEERING_ANGLE_MAX;

        pre_cmd_steering_angle = cmd_steering_angle;
    }


    PedalControl((double) current_velocity, (double) cmd_velocity);

    SteeringControl(cmd_steering_angle);

    printf("cmd_velocity=%f, cmd_steering_angle=%f\n", cmd_velocity, cmd_steering_angle);
}

static void setDirect_control(void)
{
    float cmd_steering_angle = v_cmd.steering_angle;

    set_drv_stroke((double) v_cmd.accel_stroke);
    set_brake_stroke((double) v_cmd.brake_stroke);
    SteeringControl(cmd_steering_angle);
}

#ifdef USE_CANLIB
canStatus Kvaser_canbus_write(long id, char msg[], int len)
{
    static canHandle hnd = -1;
    canStatus stat;

    if (hnd < 0) {
        /* Open channels, parameters and go on bus */
        hnd = canOpenChannel(CAN_CHANNEL, 0);
        if (hnd < 0) {
            printf("ERROR canOpenChannel %d\n", CAN_CHANNEL);
            return canERR_INVHANDLE;
        }

        canSetBusParams(hnd, CAN_BITRATE, 0, 0, 0, 0, 0);
        canSetBusOutputControl(hnd, canDRIVER_NORMAL);
        canBusOn(hnd);
    }

    stat = canWrite(hnd, id, msg, len, CAN_CHANNEL);
    // TODO: check if stat is error or not
    stat = canWriteSync(hnd, 25);
    return stat;
}
#endif
// Using socketCAN
int SocketCAN_canbus_write(long id, char msg[], int len)
{
    struct can_frame frame;
    struct ifreq ifr;
    struct sockaddr_can addr;
    static int can_socket = -1;
    int ret;
    if (can_socket == -1) {
        if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            perror("Cannot open socket");
            return 1;
        }

        strcpy(ifr.ifr_name, "can0");

        if (ioctl(can_socket, SIOCGIFINDEX, &ifr)) {
            perror("ioctl");
            return 1;
        }

        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_socket, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
            perror("bind");
            return 1;
        }
    }

    frame.can_dlc = len;
    frame.can_id = id;
    frame.can_id &= CAN_SFF_MASK;
    memcpy(frame.data, msg, frame.can_dlc);
    ret = write(can_socket, &frame, sizeof(frame));

    // FIXME: Close socket!

    return ret;
}

void canbus_write(long id, char msg[], int len)
{
#ifdef USE_CANLIB
    if (v_config.CAN_dev_ID == CAN_DEV_ID_KVASER) {
        Kvaser_canbus_write(id, msg, len);
    } else
#endif
    {
        SocketCAN_canbus_write(id, msg, len);
    }
}

void send_xgeneCan_cmd(void)
{
    int16_t int16_angle = (int16_t) xgeneCan_cmd.steering_angle;
    static char msg[6];

    msg[0] = xgeneCan_cmd.brake_stroke;
    msg[1] = xgeneCan_cmd.accel_stroke;
    msg[4] = xgeneCan_cmd.shift;
    msg[5] = xgeneCan_cmd.light;
    msg[5] = 0x04;

    // Byte order: Big-endian
    // high byte
    msg[2] = (int16_angle >> 8) & 0x00ff;
    // low byte
    msg[3] = int16_angle & 0x00ff;

    canbus_write(CAN_ID_CMD_XGENE_040, (char *) msg, sizeof(msg));
}


void *canCMDsender(void *unused)
{
    long long int current_time, last_cmdSend_time, remain_time;

    printf("INTO acer_vehicle_control::CAN_CMD_Sender thread.\n");

    // init cmd value
    cmd_reset();

    last_cmdSend_time = getTime();

    printf("[acer_vehicle_control::CAN_CMD_Sender] can open done.\n");
    while (ros::ok() && !willExit) {
        current_time = getTime();
        if ((current_time - last_cmdSend_time) >= CAN_CMD_INTERVAL) {
            // autoware control
            switch (v_cmd.modeValue) {
            case 1:  // auto pilot
                if (auto_setVehicleGear())
                    break;  // break if during auto stop for changing gear
            case 2:         // UI twist control
                setVehicleGear();
                setVehicleDrv_control();
                // printf("[CAN CMD] mode auto/UI twist\n");
                break;
            case 3:  // UI direct control
                setVehicleGear();
                setDirect_control();
                // printf("[CAN CMD] mode UI direct\n");
                break;
            // case 0: //manual mode
            //   break
            default:
                break;
            }

            if (v_cmd.modeValue == 0) {  // == manual mode
                cmd_reset();
                xgeneCan_cmd_reset();
            } else if (v_config.vendor_ID == VENDOR_ID_XGENE) {
                send_xgeneCan_cmd();
            }

            last_cmdSend_time = current_time;
        }

        remain_time = CAN_CMD_INTERVAL + last_cmdSend_time - getTime();
        if (remain_time > 0)
            usleep(remain_time * 1000);
    }

    printf("EXIT acer_vehicle_control::CAN_CMD_Sender thread.\n");

    return nullptr;
}
