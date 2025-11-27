# ROS2 Humble Workspace

This repository contains a ROS 2 Humble workspace used to experiment with core ROS concepts such as nodes, topics, publishers, and subscribers using both **C++ (rclcpp)** and **Python (rclpy)**.

The goal of this project is to understand how ROS 2 applications communicate and how different nodes interact within the ROS graph.

---
## Mandatory Setup

In every new terminal session, the workspace environment must be sourced before running any node:

```bash
cd ~/Public/ros2-humble-workspace/ros2_ws
source install/setup.bash
```

After modifying any C++ code or package configuration, rebuild the workspace:

```bash
colcon build
source install/setup.bash
```

---

## Core ROS2 Concepts

### Nodes

A node is a single executable that performs a specific task. Each node runs independently and communicates with other nodes through the ROS middleware. Examples in this project include:

* cpp_talker
* cpp_listener
* py_talker
* py_listener

Nodes form a distributed system and are discovered automatically by ROS2 once running.

### Topics

A topic is a named communication channel over which nodes exchange messages. Topics follow a publish/subscribe model:

* A publisher sends messages to a topic
* A subscriber receives messages from that topic

Multiple publishers and subscribers can exist on the same topic. Communication happens asynchronously.

For communication to work, nodes must agree on:

* Topic name (string)
* Message type (e.g. std_msgs/msg/String)

### Publishers

A publisher is created by a node to send data to a topic. It defines:

* Topic name
* Message type
* Quality of Service (QoS)

### Subscribers

A subscriber listens to a topic and executes a callback function each time a new message arrives.

---

## How This Project Works

This workspace demonstrates two independent communication systems:

| Language | Talker Node | Listener Node | Topic        |
| -------- | ----------- | ------------- | ------------ |
| C++      | cpp_talker  | cpp_listener  | /cpp_chatter |
| Python   | py_talker   | py_listener   | /chatter     |

Each talker periodically publishes text messages and each listener prints the received data to the terminal.

Cross-language communication is possible by simply using the same topic name and message type in both languages.

---

## Running Nodes

### C++ Nodes

```bash
ros2 run my_cpp_pkg cpp_talker
ros2 run my_cpp_pkg cpp_listener
```

### Python Nodes

```bash
ros2 run my_py_pkg py_talker
ros2 run my_py_pkg py_listener
```

---

## Debugging and Inspection Commands

### List running nodes

```bash
ros2 node list
```

### List active topics

```bash
ros2 topic list
```

### Inspect a topic

```bash
ros2 topic info /topic_name
ros2 topic info /topic_name --verbose
```

Displays:

* Publishers
* Subscribers
* Node names
* Message type
* QoS configuration

### View topic messages

```bash
ros2 topic echo /topic_name
```

### List executables in a package

```bash
ros2 pkg executables <package_name>
```

---

## ROS2 Communication Model

ROS2 uses DDS (Data Distribution Service) as its underlying transport layer. Nodes discover each other automatically and create peer-to-peer connections. There is no central message broker.

The ROS graph represents the full system and includes:

* Nodes
* Topics
* Services
* Actions

This graph can be visualized using:

```bash
rqt_graph
```

---

## Common Problems

* Workspace not sourced
* Executables run from unsourced terminal
* Topic name mismatch
* Message type mismatch
* C++ code modified without rebuilding
* Python package missing **init**.py

---

## Cleaning the Workspace

If unexpected behavior occurs:

```bash
rm -rf build install log
colcon build
source install/setup.bash
```

---

## Quick Command Reference

```bash
# Build
colcon build

# Source
source install/setup.bash

# Run nodes
ros2 run my_cpp_pkg cpp_talker
ros2 run my_py_pkg py_talker

# System inspection
ros2 node list
ros2 topic list
ros2 topic info /chatter --verbose

# Monitor messages
ros2 topic echo /chatter
```

---

## Summary

* ROS2 applications are built using nodes that communicate over topics
* Topics act as message channels and follow publisher/subscriber semantics
* Nodes are language-agnostic as long as topic names and message types match
* Proper sourcing and rebuilding are critical for correct operation

This README serves as an operational reference for working with ROS2 Humble inside this workspace.
