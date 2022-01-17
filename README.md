# Acer EV Controller

## **Prepare**
### ***Step one***  
Download "Kvaser Linux Drivers and SDK" from https://www.kvaser.com/download/. Please read the README in the driver first. And follow the instructions to install it.  
<font color=red>***YOU NEED A KVASER USB CANBUS HARDWARE TO CONTROL VEHICLE!***</font>  
### ***Step two***  
Compile ROS nodes
```bash=
 cd ~/catkin_ws
 catkin_make
```
## **Execute**
### ***Step three***  
Setup env
```bash=
  ~/catkin_ws$ source devel/setup.sh
```
### ***Step four***  
Loading vehicle parameters to ros
```bash=
roslaunch setup_vehicle_info<RENAME CAR MODEL HERE>.launch
```
### ***Step five***  
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

