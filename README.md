# Ground-Vehicle-Manager
This repository consists of software packages for managing higher-level tasks of an autonomous ground vehicle. It is implemented and tested in ROS Noetic and Ubuntu 20.04. There are two main ROS packages: one for diagnostics and another for the robot_state_machine.

## Major Requirements Considered:
The codebase is structured with the following in mind:

- Easy to choose the type of path-tracking controller.
- Easier to port to a different framework like ROS2.
- Easier to add more states to the controller.
- Real-time programming.
- Easier to deploy on a new platform.

## Feature Requirements:

### Mock sensor data and monitoring system with the following sensors:

- Battery
- Temperature
- GPS accuracy
- Internet signal strength
- Emergency stop

### Navigation Task Planner:
- A higher-level task planning system that handles decision-making and control of the vehicle/robot.

## System Design:

The system is designed and implemented by considering the given requirements. The entire system is divided into two packages: diagnostics and robot_state_machine. Each package has exactly one process associated with it. The design of the entire system at the highest level is shown below.

![System Overview](docs/images/overall.png)

System running in an Ubuntu/ROS Noetic environment. Both diagnostics and state machine processes are started as ROS nodes, but the inter-process communication (IPC) between both processes does not use the ROS1 pub/sub or the service client model.

One of the important design requirements is real-time execution. ROS1 is not suitable for this, which is why ZeroMQ (https://zeromq.org/) was used, as it can handle soft real-time deadlines. An ideal choice could have been a standalone DDS protocol, but due to time and other limitations, ZeroMQ was selected as the IPC mechanism between both processes.

Real-time Publishers (http://wiki.ros.org/realtime_tools) are used for non-critical data sharing. One example is the robot_state topic from robot_state_machine.

Since multithreading is easier to handle in making the system deterministic, this implementation uses multiple threads to avoid unnecessary multiprocessing. All threads are implemented with higher priority and in a lock-free manner. This setup requires a Linux real-time kernel and root permission for execution.

Google Protobuf is used as the message structure in the diagnostics system to make it easier to port to other frameworks. Details of the diagnostics design are explained separately. A serialized string is passed as the data between the processes.
