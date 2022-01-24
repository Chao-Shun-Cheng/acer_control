/*
 *  Copyright (c) 2017, Acer Inc.
 *  All rights reserved.
 */

#include <autoware_msgs/CanInfo.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>

#include <netinet/in.h>
#include <pthread.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "vehicle.h"


ros::Publisher can_pub;
ros::Publisher mode_pub;
ros::Publisher obj_pose_pub;


int willExit = 0;
vehicle_config_t v_config;

// int can_publish_interval = 10; // ms
// int channel = 0;
// int bitrate = BAUD_500K;
// unsigned long can_read_timeout = 2; //ms

int loading_vehicle_config()
{
    ros::NodeHandle _nh;

    v_config.is_valid = false;

    if (!_nh.hasParam("/vehicle_config/vendor_ID") || !_nh.hasParam("/vehicle_config/car_ID") || !_nh.hasParam("/vehicle_config/CAN_DEV_ID") ||

        !_nh.hasParam("/vehicle_config/STEERING_ANGLE_MAX") || !_nh.hasParam("/vehicle_config/WHEEL_BASE") ||
        !_nh.hasParam("/vehicle_config/WHEEL_ANGLE_MAX") ||

        !_nh.hasParam("/vehicle_config/SPEED_LIMIT") ||

        !_nh.hasParam("/vehicle_config/_K_ACCEL_P_UNTIL20") || !_nh.hasParam("/vehicle_config/_K_ACCEL_I_UNTIL20") ||
        !_nh.hasParam("/vehicle_config/_K_ACCEL_D_UNTIL20") || !_nh.hasParam("/vehicle_config/_K_ACCEL_P_UNTIL10") ||
        !_nh.hasParam("/vehicle_config/_K_ACCEL_I_UNTIL10") || !_nh.hasParam("/vehicle_config/_K_ACCEL_D_UNTIL10") ||

        !_nh.hasParam("/vehicle_config/_K_ACCEL_I_GAIN") || !_nh.hasParam("/vehicle_config/_K_ACCEL_OFFSET") ||
        !_nh.hasParam("/vehicle_config/_ACCEL_MAX_I") || !_nh.hasParam("/vehicle_config/_ACCEL_PEDAL_MAX") ||
        !_nh.hasParam("/vehicle_config/_ACCEL_PEDAL_OFFSET") ||

        !_nh.hasParam("/vehicle_config/_K_BRAKE_P") || !_nh.hasParam("/vehicle_config/_K_BRAKE_I") || !_nh.hasParam("/vehicle_config/_K_BRAKE_D") ||
        !_nh.hasParam("/vehicle_config/_K_BRAKE_I_CYCLES") || !_nh.hasParam("/vehicle_config/_BRAKE_PEDAL_MAX") ||
        !_nh.hasParam("/vehicle_config/_BRAKE_PEDAL_STOPPING_MAX") || !_nh.hasParam("/vehicle_config/_BRAKE_PEDAL_STOPPING_MED") ||
        !_nh.hasParam("/vehicle_config/_BRAKE_PEDAL_OFFSET")) {
        // missing param
        ROS_ERROR("Vehicle config param does NOT load!");
        ROS_ERROR("Please check vehicle config!");
        return 0;
    }

    // loading vendor id, car id, CAN dev Type
    _nh.getParam("/vehicle_config/vendor_ID", v_config.vendor_ID);
    _nh.getParam("/vehicle_config/car_ID", v_config.car_ID);
    _nh.getParam("/vehicle_config/CAN_DEV_ID", v_config.CAN_dev_ID);


    // steering
    _nh.getParam("/vehicle_config/STEERING_ANGLE_MAX", v_config.STEERING_ANGLE_MAX);
    _nh.getParam("/vehicle_config/TURNING_RADIUS_MIN", v_config.TURNING_RADIUS_MIN);
    _nh.getParam("/vehicle_config/WHEEL_BASE", v_config.WHEEL_BASE);
    _nh.getParam("/vehicle_config/WHEEL_ANGLE_MAX", v_config.WHEEL_ANGLE_MAX);

    _nh.getParam("/vehicle_config/SPEED_LIMIT", v_config.SPEED_LIMIT);

    // accel
    _nh.getParam("/vehicle_config/_K_ACCEL_P_UNTIL20", v_config._K_ACCEL_P_UNTIL20);
    _nh.getParam("/vehicle_config/_K_ACCEL_I_UNTIL20", v_config._K_ACCEL_I_UNTIL20);
    _nh.getParam("/vehicle_config/_K_ACCEL_D_UNTIL20", v_config._K_ACCEL_D_UNTIL20);
    _nh.getParam("/vehicle_config/_K_ACCEL_P_UNTIL10", v_config._K_ACCEL_P_UNTIL10);
    _nh.getParam("/vehicle_config/_K_ACCEL_I_UNTIL10", v_config._K_ACCEL_I_UNTIL10);
    _nh.getParam("/vehicle_config/_K_ACCEL_D_UNTIL10", v_config._K_ACCEL_D_UNTIL10);

    _nh.getParam("/vehicle_config/_K_ACCEL_I_GAIN", v_config._K_ACCEL_I_GAIN);
    _nh.getParam("/vehicle_config/_K_ACCEL_OFFSET", v_config._K_ACCEL_OFFSET);
    _nh.getParam("/vehicle_config/_ACCEL_MAX_I", v_config._ACCEL_MAX_I);
    _nh.getParam("/vehicle_config/_ACCEL_PEDAL_MAX", v_config._ACCEL_PEDAL_MAX);
    _nh.getParam("/vehicle_config/_ACCEL_PEDAL_OFFSET", v_config._ACCEL_PEDAL_OFFSET);

    // brake
    _nh.getParam("/vehicle_config/_K_BRAKE_P", v_config._K_BRAKE_P);
    _nh.getParam("/vehicle_config/_K_BRAKE_I", v_config._K_BRAKE_I);
    _nh.getParam("/vehicle_config/_K_BRAKE_D", v_config._K_BRAKE_D);

    _nh.getParam("/vehicle_config/_K_BRAKE_I_CYCLES", v_config._K_BRAKE_I_CYCLES);
    if (v_config._K_BRAKE_I_CYCLES > MAX_K_BRAKE_I_CYCLES)
        v_config._K_BRAKE_I_CYCLES = MAX_K_BRAKE_I_CYCLES;
    _nh.getParam("/vehicle_config/_BRAKE_MAX_I", v_config._BRAKE_MAX_I);
    _nh.getParam("/vehicle_config/_BRAKE_PEDAL_MAX", v_config._BRAKE_PEDAL_MAX);
    _nh.getParam("/vehicle_config/_BRAKE_PEDAL_STOPPING_MAX", v_config._BRAKE_PEDAL_STOPPING_MAX);
    _nh.getParam("/vehicle_config/_BRAKE_PEDAL_STOPPING_MED", v_config._BRAKE_PEDAL_STOPPING_MED);
    _nh.getParam("/vehicle_config/_BRAKE_PEDAL_OFFSET", v_config._BRAKE_PEDAL_OFFSET);
    v_config.is_valid = true;


    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "acer_vehicle_control");
    ros::NodeHandle nh;
    ros::Rate exitCheck_rate(1);  // 1 hz

    /* Allow signals to interrupt syscalls */
    signal(SIGINT, sighand);
    siginterrupt(SIGINT, 1);


    ros::Subscriber sub[7];
    sub[0] = nh.subscribe("/twist_cmd", 1, twistCMDCallback);
    sub[1] = nh.subscribe("/mode_cmd", 1, modeCMDCallback);
    sub[2] = nh.subscribe("/gear_cmd", 1, gearCMDCallback);
    sub[3] = nh.subscribe("/accel_cmd", 1, accellCMDCallback);
    sub[4] = nh.subscribe("/steer_cmd", 1, steerCMDCallback);
    sub[5] = nh.subscribe("/brake_cmd", 1, brakeCMDCallback);
    // sub[6] = nh.subscribe("/enable_cmds",  1, enableCMDCallback);

    loading_vehicle_config();

    if (!v_config.is_valid) {
        ROS_ERROR("Exit acer_vehicle_contorl!");
        return 0;
    }

    if (v_config.vendor_ID == VENDOR_ID_XGENE)  // X-GENE
        std::cout << "xgene vehicle controller" << std::endl;
    else  // Acer Golf
        std::cout << "acer vehicle controller" << std::endl;

    can_pub = nh.advertise<autoware_msgs::CanInfo>("can_info", 100);
    mode_pub = nh.advertise<tablet_socket_msgs::mode_info>("mode_info", 100);


#if 0
   // lidar vel.
   ros::Subscriber lidar_vel_subcscriber;
   lidar_vel_subcscriber = nh.subscribe(
            "estimate_twist", 10,
            lidarSpeedcallback);
   // RTK vel.
   ros::Subscriber RTK_vel_subcscriber;
   RTK_vel_subcscriber = nh.subscribe(
            "vel", 1,
             RTK_Speedcallback);
#endif

    // IMU data.
    ros::Subscriber IMU_subcscriber;
    IMU_subcscriber = nh.subscribe("/imu/data", 5, IMU_callback);

    // RTK pose.
    /*
     ros::Subscriber RTK_pose_subcscriber;
     RTK_pose_subcscriber = nh.subscribe(
              "gnss_pose", 1,
               RTK_Pose_callback);
     */
    ros::Subscriber Current_pose_subcscriber;
    Current_pose_subcscriber = nh.subscribe("/current_pose", 1, Current_Pose_callback);

    int ret;

    pthread_t vehicle_info_thread;
    ret = pthread_create(&vehicle_info_thread, nullptr, canReceiver, nullptr);
    if (ret != 0) {
        std::perror("pthread_create");
        std::exit(1);
    }

    pthread_t vehicle_cmd_thread;
    ret = pthread_create(&vehicle_cmd_thread, nullptr, canCMDsender, nullptr);
    if (ret != 0) {
        std::perror("pthread_create");
        std::exit(1);
    }

    // ret = pthread_join(vehicle_info_thread, NULL);
    ret = pthread_detach(vehicle_info_thread);
    if (ret != 0) {
        std::perror("pthread_detach");
        std::exit(1);
    }

    // ret = pthread_join(vehicle_cmd_thread, NULL);
    ret = pthread_detach(vehicle_cmd_thread);
    if (ret != 0) {
        std::perror("pthread_detach");
        std::exit(1);
    }

    ros::AsyncSpinner spinner(4);  // Use 4 threads
    spinner.start();

    while (!willExit)
        ros::waitForShutdown();
    //   exitCheck_rate.sleep();

    return 0;
}
