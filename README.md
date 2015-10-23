# Gudiance-SDK-ROS
The official ROS package of Guidance SDK for 32/64 bit Ubuntu and XU3.

- We write the CMakeLists.txt file so that it automatically detects your operating system and choose the appropriate library file.
- We suppose the users are using USB for Guidance SDK on ROS. To use UART for Guidance SDK, plese reference [uart_example](https://github.com/dji-sdk/GuidanceSDK/tree/master/examples/uart_example).

# How to use
1. Setup USB devide rules so that no root privilege is required when using Guidance SDK via USB.
		
		sudo sh -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"fff0\", ATTR{idProduct}==\"d009\", MODE=\"0666\"" > /etc/udev/rules.d/51-guidance.rules'
2. Clone the repo to the catkin workspace source directory `catkin_ws/src` and then 
	
		cd ~/catkin_ws
		catkin_make
		rosrun GuidanceRos GuidanceNode
		rosrun GuidanceRos GuidanceNodeTest

# Documentation
To reduce the size of this package, we omit all documents. 

- For getting started, please refer to [Developer Guide](https://github.com/dji-sdk/Guidance-SDK/blob/master/doc/Guides/Developer_Guide/en/DeveloperGuide_en.md).
- For detailed API documentation, please refer to [Guidance_SDK_API](https://github.com/dji-sdk/Guidance-SDK/blob/master/doc/Guidance_SDK_API/Guidance_SDK_API.md).

# Calibration
     rosrun GuidanceRos GuidanceNodeCalibration

     rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 right:=/guidance/right/image_raw left:=/guidance/left/image_raw right_camera:=/guidance/right left_camera:=/guidance/left --no-service-check
(change --size param accordingly)

# Viewing pointcloud, disparity, changing stereo params
    rosrun GuidanceRos GuidanceNodeCalibration
    ROS_NAMESPACE=guidance rosrun stereo_image_proc stereo_image_proc _approximate_sync:=True
    rosrun image_view stereo_view stereo:=guidance image:=image_rect_color
    rosrun rqt_reconfigure rqt_reconfigure 
    rosrun rviz rviz. Change frame to "guidance". Add pc2. 
(GuidanceNodeCalibration publishes hardcoded params which are got from its API first. To improve on this later by either using cam_info_manager or a YAML parser)
In rqt_reconfigure, play around to improve disparity (http://wiki.ros.org/stereo_image_proc/Tutorials/ChoosingGoodStereoParameters)