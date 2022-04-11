/*
 *  Copyright (c) 2017, Acer Inc.
 *  All rights reserved.
 */

#include <autoware_can_msgs/CANInfo.h>
#include <ros/ros.h>
#include <tf/tf.h>

// SocketCAN
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>


#include <math.h>
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

extern ros::Publisher can_pub;
extern ros::Publisher mode_pub;

vehicle_info_t v_info;

float lidar_speed, RTK_speed;
double current_pose_x;
double current_pose_y;
double AEB_dp_obj_distance = 0.0;

using namespace std::chrono;

int get_vInfo_accel_stroke()
{
    // FIXME:: critical section
    return v_info.accel_stroke;
}

int get_vInfo_brake_stroke()
{
    // FIXME:: critical section
    return v_info.brake_stroke;
}

float get_vInfo_velocity()
{
    // FIXME:: critical section
    return v_info.velocity;
}

char get_vInfo_gear()
{
    // FIXME:: critical section
    return v_info.shift;
}

int get_vInfo_steering_angle()
{
    return v_info.steering_angle;
}

static bool parseCanValue(long id, unsigned char msg[], unsigned int dlc)
{
    // static int counter_wheel = 0;
    static float wheel_speed_sensor = 0;
    static float motor_speed = 0;

    // FIXME:: critical section
    switch (id) {
    case CAN_ID_INFO_XGENE_060:
        if (dlc == 6) {
            v_info.brake_stroke = msg[0];
            v_info.accel_stroke = msg[1];
            v_info.steering_angle = (float) ((int16_t)(((msg[2] << 8) & 0xff00) + msg[3]) - XGENE_CAN_STEERING_OFFSET);
            v_info.light = msg[5];

            // xgene CAN gear
            switch (msg[4]) {
            case XGENE_GEAR_N:
                v_info.shift = GEAR_N;
                break;
            case XGENE_GEAR_R:
                v_info.shift = GEAR_R;
                break;
            case XGENE_GEAR_D:
                v_info.shift = GEAR_F;
                break;
            case XGENE_GEAR_P:
                v_info.shift = GEAR_P;
                break;
            }
        } else {
            std::cout << "Warning: CAN_ID_INFO_XGENE_060 error length!" << std::endl;
        }
        break;
    case CAN_ID_INFO_XGENE_061:
        if (dlc == 5) {
            // driver op info
            v_info.drv_op_info = msg[0];

            // xgene CAN control mode
            switch (msg[1]) {
            case XGENE_CMD_MODE_MANUAL:
                v_info.control_mode = CMD_MODE_MANUAL;
                break;
            case XGENE_CMD_MODE_PROGRAM:
                v_info.control_mode = CMD_MODE_PROGRAM;
                break;
            case XGENE_CMD_MODE_OVERRIDE:
                v_info.control_mode = CMD_MODE_MANUAL;
                break;
            case XGENE_CMD_MODE_INIT:
                v_info.control_mode = CMD_MODE_MANUAL;
                break;
            }
#if 1
            // xgene CAN speed
            // Byte order: Big-endian
            uint16_t speed = 0;
            speed = (((uint16_t) msg[2]) << 8) & 0xFF00;  // speed high byte
            speed += msg[3];                              // speed low byte
            motor_speed = ((float) speed) / 10;
            // for the case: wheel encoder failed
            if (wheel_speed_sensor == 0)
                v_info.velocity = ((float) motor_speed) / 10;  // offset /10
                                                               // long long int current_time = getTime();
                                                               // printf("[speed INFO] Time=%lld,lidar_velocity:%f, vehicle_velocity:%f\n",
                                                               //                   current_time, lidar_speed, v_info.velocity);
#endif
            // msg[4] is SOC(battery)
            v_info.SOC_battery = msg[4];

        } else {
            std::cout << "Warning: CAN_ID_INFO_XGENE_061 error length!" << std::endl;
        }
        break;
#if 1
    case CAN_ID_INFO_XGENE_WHEEL_AVG_VELOC:
        if (dlc == 6) {
            // xgene CAN speed
            // Byte order: Big-endian
            uint16_t speed = 0;
            uint16_t brake_sensor1 = 0;
            uint16_t brake_sensor2 = 0;
            speed = (((uint16_t) msg[0]) << 8) & 0xFF00;  // speed high byte
            speed += msg[1];                              // speed low byte
            wheel_speed_sensor = ((float) speed) / 100;
            v_info.velocity = ((float) speed) / 100;  // offset /100

            brake_sensor1 = (((uint16_t) msg[2]) << 8) & 0xFF00;  // brake_sensor1 high byte
            brake_sensor1 += msg[3];                              // brake_sensor1 low byte
            brake_sensor2 = (((uint16_t) msg[4]) << 8) & 0xFF00;  // brake_sensor2 high byte
            brake_sensor2 += msg[5];                              // brake_sensor2 low byte
            v_info.brake_sensor1 = brake_sensor1;
            v_info.brake_sensor2 = brake_sensor2;

        } else {
            std::cout << "Warning: CAN_ID_INFO_XGENE_WHEEL_AVG_VELOC error length!" << std::endl;
        }
        break;
#endif
    case CAN_ID_INFO_DRV_OP:
        if (dlc == 1)
            v_info.drv_op_info = msg[0];
        else
            std::cout << "Warning: CAN_ID_INFO_BRAKE error length!" << std::endl;
        break;
    case CAN_ID_INFO_BRAKE:
        if (dlc == 2)
            v_info.brake_stroke = msg[1];
        else if (dlc == 3)  // NTUT T-KNG
            v_info.brake_stroke = msg[0];
        else
            std::cout << "Warning: CAN_ID_INFO_BRAKE error length!" << std::endl;
        break;
    case CAN_ID_INFO_ANGLE_TORQUE:
        if (dlc == 4) {
            // Byte order: Big-endian
            v_info.steering_angle = (float) (int16_t)(((msg[0] << 8) & 0xff00) + msg[1]);
            v_info.steering_torque = (float) (((msg[2] << 8) & 0xff00) + msg[3]);
        } else {
            std::cout << "Warning: CAN_ID_INFO_ANGLE_TORQUE error length!" << std::endl;
        }
        break;
    case CAN_ID_INFO_VELOC:
        if (dlc >= 2) {
            // Byte order: Big-endian
            uint16_t speed = 0;
            speed = (((uint16_t) msg[0]) << 8) & 0xFF00;  // speed high byte
            speed += msg[1];                              // speed low byte
            v_info.velocity = ((float) speed) / 10;       // offset /10
        } else {
            std::cout << "Warning: CAN_ID_INFO_VELOC error length!" << std::endl;
        }
        break;
    case CAN_ID_INFO_MODE:
        if (dlc == 1)
            v_info.control_mode = msg[0];
        else
            std::cout << "Warning: CAN_ID_INFO_MODE error length!" << std::endl;
        break;
    case CAN_ID_INFO_ACCEL:
        if (dlc == 1)
            v_info.accel_stroke = msg[0];
        else
            std::cout << "Warning: CAN_ID_INFO_ACCEL error length!" << std::endl;
        break;
    case CAN_ID_INFO_SHIFT:
        if (dlc == 1)
            v_info.shift = msg[0];
        else
            std::cout << "Warning: CAN_ID_INFO_SHIFT error length!" << std::endl;
        break;
    default:
        std::cout << "Warning: recv unknown CAN ID : " << id << std::endl;
        return false;
    }
    // v_info.velocity *= 2.6;
    return true;
}

void lidarSpeedcallback(const geometry_msgs::TwistStamped &msg)
{
    long long int current_time = getTime();
    lidar_speed = msg.twist.linear.x * 3.6;

    printf("[speed INFO] Time=%lld,lidar_velocity:%f, vehicle_velocity:%f, diff:%f\n", current_time, lidar_speed, v_info.velocity,
           abs(lidar_speed - v_info.velocity));
}

void RTK_Speedcallback(const geometry_msgs::TwistStamped &msg)
{
    long long int current_time = getTime();
    RTK_speed = msg.twist.linear.z * 3.6;

    printf("[speed INFO] Time=%lld,RTK_velocity:%f, vehicle_velocity:%f, diff:%f\n", current_time, RTK_speed, v_info.velocity,
           abs(RTK_speed - v_info.velocity));
}


void Current_Pose_callback(const geometry_msgs::PoseStampedConstPtr &pose)
{
    current_pose_x = (double) pose->pose.position.x;
    current_pose_y = (double) pose->pose.position.y;
}


void IMU_callback(const sensor_msgs::ImuConstPtr &IMU_ptr) {}

static void *publish_can_msg(void)
{
    autoware_can_msgs::CANInfo can_msg;

    can_msg.header.frame_id = "/can";
    can_msg.header.stamp = ros::Time::now();
    can_msg.brakepedal = v_info.brake_stroke;
    can_msg.angle = v_info.steering_angle;
    can_msg.torque = v_info.steering_torque;
    can_msg.speed = v_info.velocity * v_config._BIAS_VEL;
    can_msg.drivepedal = v_info.accel_stroke;
    can_msg.driveshift = v_info.shift;
    can_pub.publish(can_msg);

    tablet_socket_msgs::mode_info mode_msg;
    mode_msg.header.frame_id = "/mode";
    mode_msg.header.stamp = ros::Time::now();
    mode_msg.mode = v_info.control_mode;
    mode_pub.publish(mode_msg);

    return nullptr;
}

void *canReceiver(void *unused)
{
#ifdef USE_CANLIB
    canHandle hnd = -1;
    canStatus stat;

    unsigned char echo_status = 0;
    unsigned int flag;
    unsigned long time;
#endif

    long id;

    // Use socketCAN (for PX2/CANalbe.io)
    struct can_frame frame_rd;
    struct ifreq ifr;
    struct sockaddr_can addr;
    static int can_socket = -1;
    // int ret;
    int recvbytes;
    // Use socketCAN END(for PX2/CANalbe.io)

    unsigned char msg[8];
    unsigned int dlc;
    bool update = false, update_print = false;
    long long int current_time, last_pub_time, last_print_time;

    current_time = last_print_time = last_pub_time = getTime();
    printf("INTO acer_vehicle_control CAN Info Receiver thread.\n");

#ifdef USE_CANLIB
    if (v_config.CAN_dev_ID == CAN_DEV_ID_KVASER) {
        /* Open channels, parameters and go on bus */
        hnd = canOpenChannel(CAN_CHANNEL, 0);
        printf("can info recv: canOpenChannel %d\n", CAN_CHANNEL);
        if (hnd < 0) {
            printf("canOpenChannel %d", CAN_CHANNEL);
            // check_can("", hnd);
            return nullptr;
        }
        if ((stat = canIoCtl(hnd, canIOCTL_SET_LOCAL_TXECHO, &echo_status, 1)) != canOK) {
            printf("ERROR canIoCtl(canIOCTL_SET_LOCAL_TXECHO) FAILED, \n");
        }
        canSetBusParams(hnd, CAN_BITRATE, 0, 0, 0, 0, 0);
        // canSetBusOutputControl(hnd, canDRIVER_NORMAL);
        canBusOn(hnd);
    } else  // v_config.CAN_dev_ID == CAN_DEV_ID_KVASER
#endif

    {  // v_config.CAN_dev_ID == CAN_DEV_ID_SOCKETCAN
        printf("can info recv: Socket CAN\n");
        // for socketCAN, on PX2
        if (can_socket == -1) {
            if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
                perror("Cannot open socket");
                return nullptr;
            }

            strcpy(ifr.ifr_name, "can0");

            if (ioctl(can_socket, SIOCGIFINDEX, &ifr)) {
                perror("ioctl");
                return nullptr;
            }

            addr.can_ifindex = ifr.ifr_ifindex;

            fcntl(can_socket, F_SETFL, O_NONBLOCK);

            if (bind(can_socket, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
                perror("bind");
                return nullptr;
            }
        }
    }  // v_config.CAN_dev_ID == CAN_DEV_ID_SOCKETCAN


    printf("[acer_vehicle_control::CAN_Info_Receiver] can open done.\n");

    while (ros::ok() && !willExit) {
#ifdef USE_CANLIB
        if (v_config.CAN_dev_ID == CAN_DEV_ID_KVASER) {
            stat = canReadWait(hnd, &id, &msg, &dlc, &flag, &time, CAN_INFO_READ_TIMEOUT_INTERVAL);  // timeout 2ms

            if (stat == canOK &&
                ((v_config.vendor_ID == VENDOR_ID_XGENE && (id >= 0x060 || id == 31)) || (v_config.vendor_ID != VENDOR_ID_XGENE && id >= 20))) {
                // try to parsing
                if (parseCanValue(id, msg, dlc) == true) {
                    update = true;
                    update_print = true;
                }
            }
        } else  // v_config.CAN_dev_ID == CAN_DEV_ID_KVASER
#endif
        {  // v_config.CAN_dev_ID == CAN_DEV_ID_SOCKETCAN
            recvbytes = read(can_socket, &frame_rd, sizeof(struct can_frame));
            if (recvbytes > 0) {
                id = frame_rd.can_id & CAN_SFF_MASK;
            }

            if (recvbytes > 0 &&
                ((v_config.vendor_ID == VENDOR_ID_XGENE && (id >= 0x060 || id == 31)) || (v_config.vendor_ID != VENDOR_ID_XGENE && id >= 20))) {
                // try to parsing
                dlc = frame_rd.can_dlc;
                memset(msg, 0, 8);
                memcpy(msg, frame_rd.data, dlc);
                if (parseCanValue(id, msg, dlc) == true) {
                    update = true;
                    update_print = true;
                }
            }
        }  // v_config.CAN_dev_ID == CAN_DEV_ID_SOCKETCAN


        current_time = getTime();

        if (update && ((current_time - last_pub_time) >= CAN_PUBLISH_INTERVAL)) {
            //
            publish_can_msg();
            last_pub_time = current_time;
            update = false;
        }

        current_time = getTime();
        if (update_print && ((current_time - last_print_time) >= MSG_PRINT_INTERVAL)) {
            //
#if 1
            printf(
                "[CAN INFO] Time=%lld, drv_op:%d, accel_stroke:%d, brake_stroke:%d,\n"
                "           steering_torque=%f, steering_angle:%f, velocity:%f,\n"
                "           battery=%d, vcu control_mode: %d, shift:%d \n",
                current_time, v_info.drv_op_info, v_info.accel_stroke, v_info.brake_stroke, v_info.steering_torque, v_info.steering_angle,
                v_info.velocity * v_config._BIAS_VEL, v_info.SOC_battery, v_info.control_mode, v_info.shift);
#endif
            last_print_time = current_time;
            update_print = false;
        }

        // v_config.CAN_dev_ID == CAN_DEV_ID_SOCKETCAN
        if (v_config.CAN_dev_ID == CAN_DEV_ID_SOCKETCAN)
            // sleep 1ms
            usleep(1000);
    }
    printf("EXIT acer_vehicle_control CAN Info Receiver thread.\n");

    return nullptr;
}
