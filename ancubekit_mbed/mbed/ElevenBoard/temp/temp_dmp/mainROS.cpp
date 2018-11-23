#include <mbed.h>
#include <BMP180.h>
#include <SparkFunMPU9250-DMP.h>
#include <mdcompat.h>

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>

// #include <sensor_msgs/FluidPressure.h>
// #include <sensor_msgs/Temperature.h>

#define G_2_MPSS 9.80665
#define DPS_2_RPS 0.0174533
#define uT_2_T 1000000

Serial pc(USBTX, USBRX);

I2C i2c(PB_9, PB_8);
BMP180 bmp180(&i2c);
MPU9250_DMP imu;

sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
std_msgs::Float64 temp_msg;
std_msgs::Float64 pressure_msg;
geometry_msgs::Vector3 rpy_msg;

ros::Publisher pub_imu("imu", &imu_msg);
ros::Publisher pub_mag("magnetometer", &mag_msg);
ros::Publisher pub_temp("temperature", &temp_msg);
ros::Publisher pub_pressure("pressure", &pressure_msg);
ros::Publisher pub_rpy("rpy", &rpy_msg);

char log_msg[255];

ros::NodeHandle nh;

int main(int argc, char **argv)
{
    nh.getHardware()->setBaud(250000);
    nh.initNode();
    nh.advertise(pub_imu);
    nh.advertise(pub_mag);
    nh.advertise(pub_rpy);
    nh.advertise(pub_pressure);
    nh.advertise(pub_temp);

    imu_init();
    stamper_init();

    while (true)
    {
        if (bmp180.init() != 0)
        {
            nh.logerror("Error communicating with BMP180");
        }
        else
        {
            nh.loginfo("Initialized BMP180");
        }
        if (imu.begin() != INV_SUCCESS)
        {
            nh.logerror("Unable to communicate with MPU-9250");
            nh.logerror("Check connections, and try again.");
        }
        else
        {
            nh.loginfo("imu.begin() suceeded");
            imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
            imu.setGyroFSR(250);
            imu.setAccelFSR(2);
            imu.setLPF(5);
            imu.setSampleRate(10);
            imu.setCompassSampleRate(10);

            if (imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL |    // Send accelerometer data
                                 DMP_FEATURE_GYRO_CAL |      // Calibrate the gyro data
                                 DMP_FEATURE_SEND_CAL_GYRO | // Send calibrated gyro data
                                 DMP_FEATURE_6X_LP_QUAT,     // Calculate quat's with accel/gyro, // Use gyro calibration
                             10) == INV_ERROR)
            {                                            // Set DMP FIFO rate to 150 Hz
                nh.logerror("imu.dmpBegin have failed"); //dmpLoad function under it fails which is caused by memcmp(firmware+ii, cur, this_write) (it is located at row 2871 of inv_mpu.c)
            }
            else
            {
                nh.loginfo("imu.dmpBegin() suceeded\n");
                break;
            }
        }
        wait(1);
    }

    Timer tim[5];
    tim[0].start(); //Debug
    tim[1].start(); //IMU
    while (1)
    {
        if (imu.dataReady())
        {
            imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
        }
        if (tim[0].read_ms() > 1000)
        {
            // sprintf(log_msg, "Altitude = %f m\n", bmp180.calAltitude(pressure, pressure0));
            // printf(log_msg);
            // nh.loginfo(log_msg);

            temp_msg.data = bmp180.getTemperature();
            pub_temp.publish(&temp_msg);

            pressure_msg.data = bmp180.getPressure();
            pub_pressure.publish(&pressure_msg);

            tim[0].reset();
        }
        if (tim[1].read_ms() > 100)
        {
            if (imu.fifoAvailable())
            {
                if (imu.dmpUpdateFifo() == INV_SUCCESS)
                {
                    // sprintf(log_msg, "ACCEL = %f, %f, %f\n", imu.accelData[0], imu.accelData[1], imu.accelData[2]);
                    // printf(log_msg);
                    imu.computeEulerAngles();

                    imu_msg.header.stamp = nh.now();
                    imu_msg.header.frame_id = "imu_link";
                    imu_msg.angular_velocity.x = imu.calcGyro(imu.gx) * DPS_2_RPS;
                    imu_msg.angular_velocity.y = imu.calcGyro(imu.gy) * DPS_2_RPS;
                    imu_msg.angular_velocity.z = imu.calcGyro(imu.gz) * DPS_2_RPS;
                    imu_msg.angular_velocity_covariance[0] = 0.0003;
                    imu_msg.angular_velocity_covariance[4] = 0.0003;
                    imu_msg.angular_velocity_covariance[8] = 0.0003;
                    imu_msg.linear_acceleration.x = imu.calcAccel(imu.ax) * G_2_MPSS;
                    imu_msg.linear_acceleration.y = imu.calcAccel(imu.ay) * G_2_MPSS;
                    imu_msg.linear_acceleration.z = imu.calcAccel(imu.az) * G_2_MPSS;
                    imu_msg.linear_acceleration_covariance[0] = 0.0003;
                    imu_msg.linear_acceleration_covariance[4] = 0.0003;
                    imu_msg.linear_acceleration_covariance[8] = 0.0003;
                    imu_msg.orientation.x = imu.calcQuat(imu.qx);
                    imu_msg.orientation.y = imu.calcQuat(imu.qy);
                    imu_msg.orientation.z = imu.calcQuat(imu.qz);
                    imu_msg.orientation.w = imu.calcQuat(imu.qw);
                    pub_imu.publish(&imu_msg);

                    mag_msg.header.stamp = nh.now();
                    mag_msg.header.frame_id = "imu_link";
                    mag_msg.magnetic_field.x = imu.calcMag(imu.mx);// * uT_2_T;
                    mag_msg.magnetic_field.y = imu.calcMag(imu.my);// * uT_2_T;
                    mag_msg.magnetic_field.z = imu.calcMag(imu.mz);// * uT_2_T;
                    mag_msg.magnetic_field_covariance[0] = 0.0003;
                    mag_msg.magnetic_field_covariance[4] = 0.0003;
                    mag_msg.magnetic_field_covariance[8] = 0.0003;
                    pub_mag.publish(&mag_msg);

                    rpy_msg.x = imu.roll;
                    rpy_msg.y = imu.pitch;
                    rpy_msg.z = imu.yaw;
                    pub_rpy.publish(&rpy_msg);
                }
            }

            tim[1].reset();
        }
        nh.spinOnce();
        wait_ms(1);
    }
}
