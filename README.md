# ros2monitor
**ros2monitor** is a monitoring software developed in Python, which allows to detect threats in messages published in ROS 2 topics using YARA rules. It also works with ROS 2 security active, complementing current security mechanisms and facilitating threat detection.   
The project consists of a ROS 2 workspace made up of two packages:  
+ **ros2monitor**: tool package.  
+ **examples**: package with examples for testing the tool.

## Installation and deployment
``` clone
git clone https://github.com/sr6y/ros2monitor
```
``` cd
cd ros2monitor
```
``` colcon
colcon build
```
``` source1
source /opt/ros/humble/setup.bash
```
``` source2
source install/local_setup.bash
```
``` run
ros2 run ros2monitor ros2monitor
```
_*Requires yara-python to be installed before running it for the first time_
``` yara-python
pip install yara-python
```
## YARA rules configuration
Go to **/src/ros2monitor/ros2monitor/rules/rules.yar** and add your own YARA rules in the file to detect threats.

## Usage
```usage
ros2 run ros2monitor ros2monitor

Type help or ? to list commands.
Commands:
  help  logs  monitoredtopics  multiple  quit  select  stop  topiclist
  
Type "help command" to find out more about the command "command".
Type "Ctrl+L" to clear-screen.

```
## Examples
To test the tool, two examples of threats have been created.  
To run them:
```example_1
ros2 run examples image_publisher_payload 
```
```example_2
ros2 run examples object_publisher_payload 
```
Once launched, we execute the tool
```monitor
ros2 run ros2monitor ros2monitor
```
We can enter the **multiple** command to try monitoring all available topics and view the logs of the detected threats with the **logs** command.   
_*The first example requires opencv-python to be installed before running it for the first time_
``` opencv-python
pip install opencv-python
```

## License
_This project is licensed under the GNU General Public License v3.0.  
See [LICENSE.md](LICENSE.md) for more details._
