#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <tf/transform_listener.h>


#include <vector>
// #include "matplotlibcpp.h"

using namespace std;
// namespace plt = matplotlibcpp;

bool base_waypoint = false, closest_waypoint = false, acer = false;
int waypoint_number = 122;
int current_waypoint_id = 0;
int a = 12.26279;
int b = -1;
int c = 513.419318;
vector<float> path_x(waypoint_number), path_y(waypoint_number), path_z(waypoint_number);

geometry_msgs::Vector3 path;
geometry_msgs::Pose current_pose;
geometry_msgs::Pose acer_pose;
tf::StampedTransform lidar2map;
autoware_msgs::DetectedObjectArray euclidean_object;

void getWaypoint(const autoware_msgs::Lane& waypoint){
    base_waypoint = true;
    for(int i = 0;i < waypoint_number;i++){
        path_x[i] = waypoint.waypoints[i].pose.pose.position.x;
        path_y[i] = waypoint.waypoints[i].pose.pose.position.y;
        path_z[i] = waypoint.waypoints[i].pose.pose.position.z;
    }
    return;
}

// void getPose(const geometry_msgs::PoseStamped& input){
//     current_pose = input.pose;
//     cout << "current position: x = " <<  current_pose.position.x << ", y = " <<  current_pose.position.y << ", z = " << current_pose.position.z << endl;
//     return;
// }

void getClosestWaypoint(const std_msgs::Int32& current_waypoint){
    closest_waypoint = true;
    current_waypoint_id = current_waypoint.data;
    return;
}

void getObject(const autoware_msgs::DetectedObjectArray& object){
    acer = true;
    euclidean_object = object;
    return;
}

bool updateTransform(const string &input, tf::StampedTransform &local2global){
    bool success = true;
    tf::TransformListener tf_listener;
    try {
        tf_listener.waitForTransform("map", input, ros::Time(0), ros::Duration(1.0));
        tf_listener.lookupTransform("map", input, ros::Time(0), local2global);
    } catch (tf::TransformException ex)  
    {
        ROS_ERROR("%s", ex.what());
        success = false;
    }
    return success;
}

geometry_msgs::Pose getTransformedPose(const geometry_msgs::Pose &in_pose, const tf::StampedTransform &tf_stamp){
    tf::Transform transform;
    geometry_msgs::PoseStamped out_pose;
    transform.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
    transform.setRotation(tf::Quaternion(in_pose.orientation.x, in_pose.orientation.y, in_pose.orientation.z, in_pose.orientation.w));
    tf::poseTFToMsg(tf_stamp * transform, out_pose.pose);
    return out_pose.pose;
}

void getDistance(const autoware_msgs::DetectedObjectArray& object){
    int id = 0;
    int number = 0;
    float offset = 0;
    float min_offset = 1000;

    bool transfer_stat = updateTransform(object.header.frame_id, lidar2map);
    if(transfer_stat == false){
        cout << "******frame transfer failed******" << endl;
    }

    for(int i = 0;i < object.objects.size();i++){
        geometry_msgs::Pose obj_pose = object.objects[i].pose;
        geometry_msgs::Pose tf_obj_pose = getTransformedPose(obj_pose, lidar2map);
        offset = abs(a * tf_obj_pose.position.x + b * tf_obj_pose.position.y + c) / sqrt(a * a + b * b);
        if(min_offset > offset){
            min_offset = offset;
            id = i;
            acer_pose = tf_obj_pose;
        }
    }
    // float distance = pow(pow((acer_pose.position.x - current_pose.position.x), 2) + pow((acer_pose.position.y - current_pose.position.y), 2), 0.5);
    float distance = pow(pow((acer_pose.position.x - path_x[current_waypoint_id + 1]), 2) + pow((acer_pose.position.y - path_y[current_waypoint_id + 1]), 2), 0.5);
    cout << "Current waypoint position: x = " <<  path_x[current_waypoint_id + 1] << ", y = " << path_y[current_waypoint_id + 1] << endl;
    cout << "Acer position: x = " <<  acer_pose.position.x << ", y = " <<  acer_pose.position.y << endl;
    cout << "Acer id = " << id  << ", distance = " << distance << endl;
    cout << "---------------------------------" << endl;
}

int main(int argc, char **argv)
{
    printf("pid: %d\n", getpid());
    ros::init(argc, argv, "front_position");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/base_waypoints", 1, getWaypoint);
    // ros::Subscriber sub2 = n.subscribe("/ndt_pose", 1, getPose);
    ros::Subscriber sub2 = n.subscribe("/closest_waypoint", 1, getClosestWaypoint);
    ros::Subscriber sub3 = n.subscribe("/detection/lidar_detector/objects", 1, getObject);
    ros::Rate loop_rate(10);

    while(ros::ok()){
        ros::spinOnce();
        if(base_waypoint){
            if(closest_waypoint && acer)getDistance(euclidean_object);
        }
        loop_rate.sleep();
        euclidean_object.objects.clear(); 
    }
    return 0;
}
