#include <mbed.h>
#include <ros.h>
#include <ros/time.h>
#include <mpu9250.h>
#include <BMP180.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>

geometry_msgs::Twist cmd_msg;
sensor_msgs::Imu imu_msg;
std_msgs::Float32MultiArray sonar_msg;

MPU9250 imu(PB_9, PB_8);

void cmdvel_cb(const geometry_msgs::Twist &twist_msg);

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char log_msg[255];
char base_footprint[] = "/base_footprint";
char odom[] = "/odom";

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", &cmdvel_cb);
ros::Publisher pub_imu("/attitude", &imu_msg);
ros::Publisher pub_sonar("/sonar", &sonar_msg);

ros::NodeHandle nh;

float lin_vel = 0, ang_vel = 0;

int main()
{
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    broadcaster.init(nh);

    sonar_msg.layout.dim = (std_msgs::MultiArrayDimension *);
    malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    sonar_msg.layout.dim[0].label = "echo";
    sonar_msg.layout.dim[0].size = 4;
    sonar_msg.layout.dim[0].stride = 1;
    sonar_msg.layout.data_offset = 0;
    // sonar_msg.data = (int *)malloc(sizeof(int) * 8);
    sonar_msg.data_length = 4;
    nh.advertise(pub_sonar);

    nh.subscribe(sub_cmd_vel);
    nh.advertise(pub_imu);

    int count = 0;

    Timer tim[5];
    tim[0].start(); //Debug
    tim[1].start(); //IMU
    while (1)
    {
        imu.updateData();

        if (tim[0].read_ms() > 1000)
        {
            sprintf(log_msg, "Hello %d, %f, %f", count++, lin_vel, ang_vel);
            nh.loginfo(log_msg);

            sonar_msg.data[0] = 1;
            sonar_msg.data[1] = 2;
            sonar_msg.data[2] = 3;
            sonar_msg.data[3] = 4;
            pub_sonar.publish(&sonar_msg);
            tim[0].reset();
        }
        if (tim[1].read_ms() > 100)
        {
            imu_msg.header.stamp = nh.now();
            imu_msg.header.frame_id = "imu_link";
            imu_msg.angular_velocity.x = imu.gyroData[0];
            imu_msg.angular_velocity.y = imu.gyroData[1];
            imu_msg.angular_velocity.z = imu.gyroData[2];
            imu_msg.linear_acceleration.x = imu.accelData[0];
            imu_msg.linear_acceleration.y = imu.accelData[1];
            imu_msg.linear_acceleration.z = imu.accelData[2];
            imu_msg.orientation.x = imu.q[1];
            imu_msg.orientation.y = imu.q[2];
            imu_msg.orientation.z = imu.q[3];
            imu_msg.orientation.w = imu.q[0];
            pub_imu.publish(&imu_msg);

            tim[1].reset();
        }

        nh.spinOnce();
    }
}

void cmdvel_cb(const geometry_msgs::Twist &twist_msg)
{
    lin_vel = twist_msg.linear.x;
    ang_vel = twist_msg.angular.z;
}