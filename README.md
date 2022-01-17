# Acer EV Controller

## **Prepare**
### ***Step one***  
Download "Kvaser Linux Drivers and SDK" from https://www.kvaser.com/download/. Please read the README in the driver first. And follow the instructions to install it.  
<font color=red>***YOU NEED A KVASER USB CANBUS HARDWARE TO CONTROL VEHICLE!***</font>  
### ***Step two***  
Copy acer_vehicle_control to your folder:  
```bash=
cp vehicle_socket Autoware/ros/src/socket/packages/vehicle_socket
```

### ***Step three***  
Copy vehicle_config to your autoware folder  
```bash=
cp vehicle_config Autoware/ros/src/vehicle_config
```

Compile ROS nodes
```bash=
 cd ~/Autoware/ros/src
 catkin_init_workspace
 cd ../
 ./catkin_make_release
```
## **Execute**
### ***Step four***  
Setup env
```bash=
  ~/Autoware/ros$ source devel/setup.sh
```
### ***Step five***  
Loading vehicle parameters to ros
```bash=
cd ~/Autoware/ros/src/vehicle_config
~/Autoware/ros/src/vehicle_config$ roslaunch setup_vehicle_info<RENAME CAR MODEL HERE>.launch
```
### ***Step six***  
Launch Acer vehicle controller
```bash=
rosrun vehicle_socket acer_vehicle_control
```
### ***Step seven***  
Sending an autopilot(or manual) mode cmd to "/mode_cmd" topic to Acer vehicle controller

## Notice
If you are ready to have fun with ***auto driving***,
just switch the mode button to autopilot at the dashboard of the vehicle.
The vehicle will be controlled by ROS.
   
***If you want to turn off autopilot***, switch the button off (or hit the brakes).
The vehicle will be in manual mode. You can take back control immediately.
   

***Topic Name*** : mode_cmd  
***case 1*** : auto pilot  
***case 0*** : manual mode

