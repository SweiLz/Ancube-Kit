# Ancube-Kit

Ancube kit for education perpose.

for Raspberry Pi 3 (Ubuntu Mate 16.04) (ROS Kinetic)

```bash
sudo apt install ros-kinetic-urdf
sudo apt install ros-kinetic-joint-state-publisher
sudo apt install ros-kinetic-robot-state-publisher
```

```bash
cd ~/catkin_ws/src
git clone https://github.com/EAIBOT/ydlidar.git
```

```bash
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

### Time sync

__At the first time__
```
Server$ sudo vi /etc/chrony/chrony.conf
>> allow 11.11.11
>> allow 10/8
Server$ sudo invoke-rc.d chrony restart

Client$ sudo vi /etc/chrony/chrony.conf
>> server 11.11.11.2
Client$ sudo invoke-rc.d chrony restart


```