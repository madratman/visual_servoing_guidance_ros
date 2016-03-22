## Set up ##
- Setup [dji_launch](https://bitbucket.org/castacks/dji_launch) using the instructions in link.

- git clone this repo inside the dji_launch workspace. `catkin_make` again

- `roslaunch dji_launch sdk_demo.launch`   
   (In case of error, please ensure the key is correct.)

- `roslauch GuidanceRos GuidanceNodeCalibration.launch`

- `rosrun GuidanceRos GuidanceNodeCalibration`

- `rosrun GuidanceRos real_life_node`

One Guidance sensor is mounted on upper face.  
Check [flashy video](https://www.youtube.com/watch?v=5hpSSbL3lYI) of demo.    
Check not so [flashy video](https://www.youtube.com/watch?v=_Wie7HQW1x8).
TODO: Add gripper and latch on line   

### Setting up odroid (most probably you have already done this, but anyway)
On odroid, add these lines to `/etc/rc.local` to bring up an ad-hoc on bootup each time. (Or use your favourite way - dhcp/static/etc). 

Please check what's the N in `wlanN` below by a quick `dmesg|grep wlan`. Change N from 2 to 1 or 0 or whatever it is.
```
 ifconfig wlan2 down
 iwconfig wlan2 mode ad-hoc
 ifconfig wlan2 up
 iwconfig wlan2 essid "odroid_adhoc"
 ifconfig wlan2 192.168.1.1 netmask 255.255.255.0
 
 chmod o+rw /dev/ttyUSB0 

 exit 0
```
Assign maybe `192.168.1.2` to your lappie


## What project generated this code?
Power line inspection