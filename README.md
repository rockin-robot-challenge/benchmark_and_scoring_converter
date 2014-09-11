RoCKIn Converter
================

Tool to convert RoCKIn YAML files to ROS bags.

## Dependencies

You need to use at least ROS Hydro (for the audio messages).

```
sudo apt-get install ros-${ROS_DISTRO}-audio-common-msgs libyaml-dev
```

## Compiling

Compile as a normal ROS package.

## Running

```
rosrun rockin_converter rockin_converter FILE1.yaml FILE2.yaml ...
```

There are two utils to check that base 64 conversion is being done correctly:

```
rosrun rockin_converter base64_encode < INPUT_BINARY_FILE > OUTPUT_BASE64_FILE
rosrun rockin_converter base64_decode < INPUT_BASE64_FILE > OUTPUT_BINARY_FILE
```
