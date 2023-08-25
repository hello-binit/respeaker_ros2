respeaker_ros2
=============

A ROS 2 Package for Respeaker Mic Array


## Supported Devices

- [Respeaker Mic Array v2.0](http://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/)

    ![Respeaker Mic Array v2.0](https://github.com/SeeedDocument/ReSpeaker_Mic_Array_V2/raw/master/img/Hardware%20Overview.png)

## Preparation

1. Install this package

    ```bash
    mkdir -p ~/ament_ws/src && ~/ament_ws/src
    git clone https://github.com/hello-chintan/respeaker_ros2.git
    git clone https://github.com/hello-chintan/audio_common.git
    cd ~/ament_ws
    source /opt/ros/iron/setup.bash
    rosdep install --from-paths src --ignore-src -i -r -n -y
    colcon build
    source install/setup.bash
    ```

1. Register respeaker udev rules

    Normally, we cannot access USB device without permission from user space.
    Using `udev`, we can give the right permission on only respeaker device automatically.

    Please run the command as followings to install setting file:

    ```bash
    cd ~/ament_ws/src/respeaker_ros2
    sudo cp -f ~/ament_ws/src/respeaker_ros2/config/60-respeaker.rules /etc/udev/rules.d/60-respeaker.rules
    sudo systemctl restart udev
    ```

    And then re-connect the device.

1. Install python requirements

    ```bash
    cd ~/ament_ws/src/respeaker_ros2
    sudo pip install -r requirements.txt
    ```

1. Update firmware

    ```bash
    git clone https://github.com/respeaker/usb_4_mic_array.git
    cd usb_4_mic_array
    sudo python dfu.py --download 6_channels_firmware.bin  # The 6 channels version 
    ```

## How to use

1. Run executables

    ```bash
    ros2 launch respeaker_ros2 respeaker.launch
    ros2 topic echo /sound_direction     # Result of DoA
    ros2 topic echo /sound_localization  # Result of DoA as Pose
    ros2 topic echo /is_speeching        # Result of VAD
    ros2 topic echo /audio               # Raw audio
    ros2 topic echo /speech_audio        # Audio data while speeching
    ```

    To set LED color, publish desired color:

    ```bash
    ros2 topic pub /status_led std_msgs/msg/ColorRGBA "r: 0.0
    g: 0.0
    b: 1.0
    a: 0.3"
    ```

## Use cases

### Voice Recognition

- [ros_speech_recognition](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/ros_speech_recognition)
- [julius_ros](http://wiki.ros.org/julius_ros)

## Notes

The configuration file for `dynamic_reconfigure` in this package is created automatically by reading the parameters from devices.
Though it will be rare case, the configuration file can be updated as followings:

1. Connect the device to the computer.
1. Run the generator script.

    ```bash
    rosrun  respeaker_ros respeaker_gencfg.py
    ```
1. You will see the updated configuration file at `$(rospack find respeaker_ros)/cfg/Respeaker.cfg`.


## Author

Yuki Furuta <<furushchev@jsk.imi.i.u-tokyo.ac.jp>>

## License

[Apache License](LICENSE)
