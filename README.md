# Ancube-Kit

Ancube kit for education perpose.

for Raspberry Pi 3 (Ubuntu Mate 16.04) (ROS Kinetic)

```bash
sudo apt install ros-kinetic-urdf
sudo apt install ros-kinetic-joint-state-publisher
sudo apt install ros-kinetic-robot-state-publisher
```

```
cd ~/catkin_ws/src
git clone https://github.com/EAIBOT/ydlidar.git
```

rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y