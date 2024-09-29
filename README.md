# Homeworks_ROB_311_Gianluca_Baghino_Gomez

## Homework 1: Telephone Game in ROS

This repository contains a ROS package called `telephone_game` that implements a simple communication system using four nodes: Node A, Node B, Node C, and Node D. The nodes pass messages sequentially, demonstrating basic ROS message passing and node communication.

### Step 1: Set Up the ROS Package

1. **Access the Docker Container**
   
Open a terminal and enter the Docker environment where ROS is installed:
   ```bash
   sudo docker exec -it ros-noetic-base-ros-basic-noetic-1 /bin/bash
  ```

2. **Source ROS and the Workspace**

In the Docker environment, source both ROS and your workspace:
   ```bash
   source /opt/ros/noetic/setup.bash
   source /catkin_ws/devel/setup.bash
  ```

3. **Create the Package**

Create a new package called telephone_game where all the nodes will be placed:
   ```bash
   cd /catkin_ws/src
   catkin_create_pkg telephone_game rospy std_msgs
  ```
This creates a package that depends on rospy (for Python ROS nodes) and std_msgs (for sending and receiving messages).

4. **Build the Package**

Go back to the workspace root and build the package:
  ```bash
  cd /catkin_ws
  catkin_make
  ```

5. **Source the Workspace Again**

Update your environment with the new package:
  ```bash
  source /catkin_ws/devel/setup.bash
  ```

### Step 2: Create the Nodes

Create the four nodes: node_A, node_B, node_C, and node_D. These nodes will communicate by passing messages.

1. **Navigate to the Package’s src Folder**

  ```bash
  cd /catkin_ws/src/telephone_game/src
  ```

2. **Create Python Node Scripts**

  ```bash
  touch node_A.py node_B.py node_C.py node_D.py
  chmod +x node_A.py node_B.py node_C.py node_D.py
  ```

### Step 3: Write Code for Each Node

Each node will have a publisher and a subscriber.

#### Node A: node_A.py

1. **Open the Node A File**

  ```bash
  nano node_A.py
  ```

2. **Write the Code**

  ```bash
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import threading

def callback(data):
    rospy.loginfo("Node A received: %s", data.data)

def talker():
    pub = rospy.Publisher('outgoing_A', String, queue_size=10)
    while not rospy.is_shutdown():
        message = "Hello from A"
        rospy.loginfo("Node A sending: %s", message)
        pub.publish(message)
        rospy.sleep(2)

def listener():
    rospy.Subscriber('incoming_A', String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('node_A', anonymous=True)
        talker_thread = threading.Thread(target=talker)
        listener_thread = threading.Thread(target=listener)

        talker_thread.start()
        listener_thread.start()

        talker_thread.join()
        listener_thread.join()
    except rospy.ROSInterruptException:
        pass
```

#### Node B: node_B.py

1. **Open the Node B File**

  ```bash
  nano node_B.py
  ```

2. **Write the Code**

  ```bash
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Node B received: %s", data.data)
    new_message = data.data + " -> B"
    pub.publish(new_message)
    rospy.loginfo("Node B sending: %s", new_message)

def listener():
    rospy.init_node('node_B', anonymous=True)
    rospy.Subscriber('outgoing_A', String, callback)
    global pub
    pub = rospy.Publisher('outgoing_B', String, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
```

#### Node C: node_C.py

1. **Open the Node C File**

  ```bash
  nano node_C.py
  ```

2. **Write the Code**

  ```bash
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Node C received: %s", data.data)
    new_message = data.data + " -> C"
    pub.publish(new_message)
    rospy.loginfo("Node C sending: %s", new_message)

def listener():
    rospy.init_node('node_C', anonymous=True)
    rospy.Subscriber('outgoing_B', String, callback)
    global pub
    pub = rospy.Publisher('outgoing_C', String, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
```

#### Node D: node_D.py

1. **Open the Node D File**

  ```bash
  nano node_D.py
  ```

2. **Write the Code**

  ```bash
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Node D received: %s", data.data)
    new_message = data.data + " -> D"
    pub.publish(new_message)
    rospy.loginfo("Node D sending: %s", new_message)

def listener():
    rospy.init_node('node_D', anonymous=True)
    rospy.Subscriber('outgoing_C', String, callback)
    global pub
    pub = rospy.Publisher('incoming_A', String, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
```

### Step 4: Modify CMakeLists.txt

Modify the CMakeLists.txt file to install these scripts.

1. **Open CMakeLists.txt**

  ```bash
cd /catkin_ws/src/telephone_game
nano CMakeLists.txt
```

2. **Add the Following Line Under catkin_install_python**

  ```bash
catkin_install_python(PROGRAMS src/node_A.py src/node_B.py src/node_C.py src/node_D.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### Step 5: Build and Source the Package

After editing the scripts and CMakeLists.txt, build the package again:

  ```bash
cd /catkin_ws
catkin_make
source /catkin_ws/devel/setup.bash
```

### Step 6: Start roscore

Start roscore before running any ROS nodes, as it acts as the central communication hub for all ROS processes.

1. **Open a New Terminal**

  ```bash
sudo docker exec -it ros-noetic-base-ros-basic-noetic-1 /bin/bash
```

2. **Source ROS**

  ```bash
source /opt/ros/noetic/setup.bash
```

3. **Start roscore**

  ```bash
roscore
```

### Step 7: Launch the Nodes in Sequence

Now that you've built the nodes and started roscore, run each of the nodes one by one.

1. **Node B**

  ```bash
sudo docker exec -it ros-noetic-base-ros-basic-noetic-1 /bin/bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
rosrun telephone_game node_B.py
```

#### Output Node B:

```less
[INFO] [1727623039.750214]: Node B received: Hello from A
[INFO] [1727623039.751779]: Node B sending: Hello from A -> B
[INFO] [1727623041.753088]: Node B received: Hello from A
[INFO] [1727623041.754432]: Node B sending: Hello from A -> B
[INFO] [1727623043.756811]: Node B received: Hello from A
[INFO] [1727623043.758154]: Node B sending: Hello from A -> B
[INFO] [1727623045.760683]: Node B received: Hello from A
[INFO] [1727623045.761931]: Node B sending: Hello from A -> B
[INFO] [1727623047.764627]: Node B received: Hello from A
```

2. **Node C**

  ```bash
sudo docker exec -it ros-noetic-base-ros-basic-noetic-1 /bin/bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
rosrun telephone_game node_C.py
```

#### Output Node C:

```less
[INFO] [1727623039.752215]: Node C received: Hello from A -> B
[INFO] [1727623039.753434]: Node C sending: Hello from A -> B -> C
[INFO] [1727623041.754870]: Node C received: Hello from A -> B
[INFO] [1727623041.756342]: Node C sending: Hello from A -> B -> C
[INFO] [1727623043.758541]: Node C received: Hello from A -> B
[INFO] [1727623043.759572]: Node C sending: Hello from A -> B -> C
[INFO] [1727623045.762357]: Node C received: Hello from A -> B
[INFO] [1727623045.763861]: Node C sending: Hello from A -> B -> C
[INFO] [1727623047.766231]: Node C received: Hello from A -> B
```

3. **Node D**

  ```bash
sudo docker exec -it ros-noetic-base-ros-basic-noetic-1 /bin/bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
rosrun telephone_game node_D.py
```

#### Output Node D:

```less
[INFO] [1727623039.754038]: Node D received: Hello from A -> B -> C
[INFO] [1727623039.755280]: Node D sending: Hello from A -> B -> C -> D
[INFO] [1727623041.756833]: Node D received: Hello from A -> B -> C
[INFO] [1727623041.758291]: Node D sending: Hello from A -> B -> C -> D
[INFO] [1727623043.759936]: Node D received: Hello from A -> B -> C
[INFO] [1727623043.761075]: Node D sending: Hello from A -> B -> C -> D
[INFO] [1727623045.764294]: Node D received: Hello from A -> B -> C
[INFO] [1727623045.765685]: Node D sending: Hello from A -> B -> C -> D
[INFO] [1727623047.767751]: Node D received: Hello from A -> B -> C
```

4. **Node A**

  ```bash
sudo docker exec -it ros-noetic-base-ros-basic-noetic-1 /bin/bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
rosrun telephone_game node_A.py
```

#### Output Node A:

```less
[INFO] [1727623037.744359]: Node A sending: Hello from A
[INFO] [1727623039.748193]: Node A sending: Hello from A
[INFO] [1727623039.755621]: Node A received: Hello from A -> B -> C -> D
[INFO] [1727623041.751289]: Node A sending: Hello from A
[INFO] [1727623041.758772]: Node A received: Hello from A -> B -> C -> D
[INFO] [1727623043.754945]: Node A sending: Hello from A
[INFO] [1727623043.761401]: Node A received: Hello from A -> B -> C -> D
[INFO] [1727623045.758790]: Node A sending: Hello from A
[INFO] [1727623045.766083]: Node A received: Hello from A -> B -> C -> D
[INFO] [1727623047.762750]: Node A sending: Hello from A
```

### Step 8: Monitoring Topics

1. **Open a New Terminal**

To enter the container, run:

  ```bash
sudo docker exec -it ros-noetic-base-ros-basic-noetic-1 /bin/bash
```

2. **Source ROS and Your Workspace**

  ```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
```

3. **Check Running Nodes**

Ensure all nodes are running:

  ```bash
rosnode list
```

4. **Check Published Topics**

Verify all the active topics that are currently being published or subscribed to in the ROS network:

  ```bash
rostopic list
```

## Conclusion Homework 1: Telephone Game in ROS

This README provides a step-by-step guide to setting up and running the telephone_game ROS package.






