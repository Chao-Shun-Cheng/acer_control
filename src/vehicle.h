#include <autoware_msgs/CanInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <tablet_socket_msgs/mode_info.h>
#include <string>

#ifdef USE_CANLIB
#include <canlib.h>
#endif

#include <signal.h>

#include <autoware_msgs/AccelCmd.h>
#include <autoware_msgs/BrakeCmd.h>
#include <autoware_msgs/SteerCmd.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tablet_socket_msgs/gear_cmd.h>
#include <tablet_socket_msgs/mode_cmd.h>

#include <netinet/in.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>


#define CAN_CHANNEL 0
#define CAN_BITRATE BAUD_500K

#define CMD_MODE_MANUAL 0
#define CMD_MODE_PROGRAM 1
#define GEAR_N 0
#define GEAR_R 1
#define GEAR_F 2
#define GEAR_P 3

#define CAN_ID_CMD_BRAKE (10)
#define CAN_ID_CMD_ANGLE (11)
#define CAN_ID_CMD_ACCEL (12)
#define CAN_ID_CMD_SHIFT (13)
#define CAN_ID_CMD_LIGHT (14)
#define CAN_ID_CMD_XGENE_040 (0x040)

#define CAN_ID_INFO_DRV_OP (20)
#define CAN_ID_INFO_BRAKE (21)
#define CAN_ID_INFO_ANGLE_TORQUE (22)
#define CAN_ID_INFO_VELOC (23)
#define CAN_ID_INFO_MODE (24)
#define CAN_ID_INFO_ACCEL (25)
#define CAN_ID_INFO_SHIFT (26)
#define CAN_ID_INFO_XGENE_060 (0x060)
#define CAN_ID_INFO_XGENE_061 (0x061)
#define CAN_ID_INFO_XGENE_WHEEL_AVG_VELOC (31)

// can MSG_PRINT_INTERVAL = 1000ms
#define MSG_PRINT_INTERVAL 500
// can cmd tx interval = 100ms
#define CAN_CMD_INTERVAL 50
// can cmd info publish interval = 10ms
#define CAN_PUBLISH_INTERVAL 10
// can cmd tx interval = 100ms
#define CAN_INFO_READ_TIMEOUT_INTERVAL 2
// wait for 3000ms
#define CHANGE_GEAR_COUNTDOWN (3000 / CAN_CMD_INTERVAL)
#define MILLISECOND 1000

using namespace std;

#define VENDOR_ID_ACER 1
#define VENDOR_ID_XGENE 2
#define VENDOR_ID_NTUT 3

#define CAN_DEV_ID_KVASER 1
#define CAN_DEV_ID_SOCKETCAN 2

#define MAX_K_BRAKE_I_CYCLES 1000

typedef struct vehicle_config {
    bool is_valid;
    int vendor_ID;
    int car_ID;
    int CAN_dev_ID;
    double SPEED_LIMIT;

    // steering
    double STEERING_ANGLE_MAX;
    double TURNING_RADIUS_MIN;
    double WHEEL_BASE;
    double WHEEL_ANGLE_MAX;

    // accel
    double _K_ACCEL_P_UNTIL20;
    double _K_ACCEL_I_UNTIL20;
    double _K_ACCEL_D_UNTIL20;
    double _K_ACCEL_P_UNTIL10;
    double _K_ACCEL_I_UNTIL10;
    double _K_ACCEL_D_UNTIL10;

    double _K_ACCEL_I_GAIN;
    double _K_ACCEL_OFFSET;
    double _ACCEL_MAX_I;
    double _ACCEL_PEDAL_MAX;
    double _ACCEL_PEDAL_OFFSET;

    // anti wand-up and lpf
    double _T_sample;
    double _tau_lpf;

    // brake
    double _K_BRAKE_P;
    double _K_BRAKE_I;
    double _K_BRAKE_D;
    int _K_BRAKE_I_CYCLES;
    double _BRAKE_MAX_I;
    double _BRAKE_PEDAL_MAX;
    double _BRAKE_PEDAL_STOPPING_MAX;
    double _BRAKE_PEDAL_STOPPING_MED;
    double _BRAKE_PEDAL_OFFSET;

} vehicle_config_t;

typedef struct vehicle_info {
    char drv_op_info;
    int accel_stroke;
    int brake_stroke;
    float steering_torque;
    float steering_angle;
    float velocity;
    int control_mode;
    char shift;
    char light;
    int SOC_battery;
    uint16_t brake_sensor1;
    uint16_t brake_sensor2;
} vehicle_info_t;

typedef struct vehicle_cmd {
    double linear_x;
    double angular_z;
    int modeValue;  // 0:manual, 1:auto pilot, 2:UI twist control, 3:UI direct control
    int shift;
    int accel_stroke;
    int brake_stroke;
    int steering_angle;
    char light;
    //   vehicle_enable_cmds_t enablesCMDs;
} vehicle_cmd_t;


typedef struct xgene_can_cmd {
    //   double linear_x;
    //   double angular_z;
    int shift;
    int accel_stroke;
    int brake_stroke;
    int steering_angle;
    char light;
} xgene_can_cmd_t;



#define XGENE_CMD_MODE_MANUAL 1
#define XGENE_CMD_MODE_PROGRAM 2
#define XGENE_CMD_MODE_OVERRIDE 3
#define XGENE_CMD_MODE_INIT 4

#define XGENE_GEAR_P 1
#define XGENE_GEAR_R 2
#define XGENE_GEAR_N 3
#define XGENE_GEAR_D 4

#define XGENE_CAN_STEERING_OFFSET 0x400

/*
class PlanningStopID
{
   public:
      static const int NOSTOP = 0;
      static const int OBSTACLE =1;
      static const int TRAFFICLIGHT = 2;
      static const int STOPSIGN = 3;
      static const int END=4;
};
*/

struct PID_valueSet {
    float P;
    float I;
    float D;
};

struct struct_PID_controller {
    struct PID_valueSet accel;
    struct PID_valueSet brake;
    struct PID_valueSet steer;
};

const std::string SHM_SEED_PATH = "/tmp/PID_controller";

extern vehicle_cmd_t v_cmd;
extern vehicle_info_t v_info;
extern vehicle_config_t v_config;
extern xgene_can_cmd_t xgeneCan_cmd;

extern int can_tx_interval;  // ms
extern int cmd_rx_interval;  // ms
extern std::string ros_ip_address;
extern double estimate_accel;
extern double cycle_time;
extern struct struct_PID_controller *shm_ptr;
extern int willExit;


void twistCMDCallback(const geometry_msgs::TwistStamped &msg);
void modeCMDCallback(const tablet_socket_msgs::mode_cmd &mode);
void gearCMDCallback(const tablet_socket_msgs::gear_cmd &gear);
void accellCMDCallback(const autoware_msgs::AccelCmd &accell);
void steerCMDCallback(const autoware_msgs::SteerCmd &steer);
void brakeCMDCallback(const autoware_msgs::BrakeCmd &brake);
void lidarSpeedcallback(const geometry_msgs::TwistStamped &msg);
void RTK_Speedcallback(const geometry_msgs::TwistStamped &msg);
void IMU_callback(const sensor_msgs::ImuConstPtr &IMU_ptr);
void Current_Pose_callback(const geometry_msgs::PoseStampedConstPtr &pose);

void set_drv_stroke(double accel_stroke);
void set_brake_stroke(double brake_stroke);

void *canReceiver(void *unused);
void *canCMDsender(void *unused);
void send_xgeneCan_cmd(void);

void PedalControl(double current_velocity, double cmd_velocity);


void canbus_write(long id, char msg[], int len);
int SocketCAN_canbus_write(long id, char msg[], int len);
#ifdef USE_CANLIB
canStatus Kvaser_canbus_write(long id, char msg[], int len);
#endif

// get can info
int get_vInfo_accel_stroke();
int get_vInfo_brake_stroke();
float get_vInfo_velocity();
char get_vInfo_gear();
int get_vInfo_steering_angle();

// convert km/h to m/s
static inline double KmhToMs(double v)
{
    return (v * 1000.0 / (60.0 * 60.0));
}

// get current time
static inline long long int getTime(void)
{
    // returns time in milliseconds
    struct timeval current_time;
    struct timezone ttz;
    double t;
    // lock();
    gettimeofday(&current_time, &ttz);
    t = ((current_time.tv_sec * 1000.0) + (current_time.tv_usec) / 1000.0);
    // unlock();
    return static_cast<long long int>(t);
}

static inline void sighand(int sig)
{
    // static unsigned int last;

    switch (sig) {
    case SIGINT:
        willExit = 1;
        ros::shutdown();
        break;
    case SIGALRM:
#if 0
    if (msgCounter - last) {
      printf("msg/s = %d, total=%d\n",
             (msgCounter - last) / ALARM_INTERVAL_IN_S, msgCounter);
    }   
    last = msgCounter;
    alarm(ALARM_INTERVAL_IN_S);
#endif
        break;
    }
}
