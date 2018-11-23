#include <mbed.h>
#include <MPU9250.h>
#include <BMP180.h>

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>

#define G_2_MPSS 9.80665
#define DPS_2_RPS 0.0174533
#define uT_2_T 1000000

DigitalOut myled(LED1);
// Serial pc(USBTX, USBRX);

MPU9250 imu;
BMP180 bmp180(&i2c);

Timer tim[4];

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
    // pc.baud(115200);
    i2c.frequency(400000);

    while (true)
    {
        if (bmp180.init() != 0)
        {
            // pc.printf("Error communicating with BMP180");
            nh.logerror("Error communicating with BMP180");
        }
        else
        {
            // pc.printf("Initialized BMP180");
            nh.loginfo("Initialized BMP180");
        }
        if (imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250) != 0x73)
        {
            // pc.printf("Unable to communicate with MPU-9250");
            // pc.printf("Check connections, and try again.\n");
            nh.logerror("Unable to communicate with MPU-9250");
            nh.logerror("Check connections, and try again.");
        }
        else
        {
            // pc.printf("MPU9250 is online...\n");
            nh.loginfo("MPU9250 is online...");
            imu.resetMPU9250();
            imu.MPU9250SelfTest(SelfTest);
            imu.calibrateMPU9250(gyroBias, accelBias);
            wait(2.0);
            imu.initMPU9250();
            imu.initAK8963(magCalibration);
            imu.getAres();
            imu.getGres();
            imu.getMres();
            nh.loginfo("imu.begin() suceeded");
            break;
        }
        wait(1.0);
    }

    // magbias[0] = +470.; // User environmental x-axis correction in milliGauss, should be automatically calculated
    // magbias[1] = +120.; // User environmental x-axis correction in milliGauss
    // magbias[2] = +125.; // User environmental x-axis correction in milliGauss
    tim[0].start();
    tim[1].start();

    while (1)
    {
        if (imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
        {
            imu.readAccelData(accelCount); // Read the x/y/z adc values
            // Now we'll calculate the accleration value into actual g's
            ax = (float)accelCount[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
            ay = (float)accelCount[1] * aRes - accelBias[1];
            az = (float)accelCount[2] * aRes - accelBias[2];

            imu.readGyroData(gyroCount); // Read the x/y/z adc values
            // Calculate the gyro value into actual degrees per second
            gx = (float)gyroCount[0] * gRes - gyroBias[0]; // get actual gyro value, this depends on scale being set
            gy = (float)gyroCount[1] * gRes - gyroBias[1];
            gz = (float)gyroCount[2] * gRes - gyroBias[2];

            imu.readMagData(magCount); // Read the x/y/z adc values
            // Calculate the magnetometer values in milliGauss
            // Include factory calibration per data sheet and user environmental corrections
            mx = (float)magCount[0] * mRes * magCalibration[0] - magbias[0]; // get actual magnetometer value, this depends on scale being set
            my = (float)magCount[1] * mRes * magCalibration[1] - magbias[1];
            mz = (float)magCount[2] * mRes * magCalibration[2] - magbias[2];
        }

        Now = tim[1].read_us();
        deltat = (float)((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
        lastUpdate = Now;

        imu.MadgwickQuaternionUpdate(ax, ay, az, gx * DPS_2_RPS, gy * DPS_2_RPS, gz * DPS_2_RPS, my, mx, mz);
        //        mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

        // if (tim[0].read_ms() > 200)
        // {
        //     myled = !myled;

        //     tim[0].reset();
        // }
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
            yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
            pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
            roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

            // pc.printf("%f, %f, %f\n", ax * G_2_MPSS, ay * G_2_MPSS, az * G_2_MPSS);
            // pc.printf("%f, %f, %f\n", gx * DPS_2_RPS, gy * DPS_2_RPS, gz * DPS_2_RPS);
            // pc.printf("%f, %f, %f\n", mx / uT_2_T, my / uT_2_T, mz / uT_2_T);
            // pc.printf("%f, %f, %f, %f\n", q[0], q[1], q[2], q[3]);
            // pc.printf("%f, %f, %f\n\n", yaw, pitch, roll);

            imu_msg.header.stamp = nh.now();
            imu_msg.header.frame_id = "imu_link";
            imu_msg.angular_velocity.x = gx * DPS_2_RPS;
            imu_msg.angular_velocity.y = gy * DPS_2_RPS;
            imu_msg.angular_velocity.z = gz * DPS_2_RPS;
            imu_msg.angular_velocity_covariance[0] = 0.0003;
            imu_msg.angular_velocity_covariance[4] = 0.0003;
            imu_msg.angular_velocity_covariance[8] = 0.0003;
            imu_msg.linear_acceleration.x = ax * G_2_MPSS;
            imu_msg.linear_acceleration.y = ay * G_2_MPSS;
            imu_msg.linear_acceleration.z = az * G_2_MPSS;
            imu_msg.linear_acceleration_covariance[0] = 0.0003;
            imu_msg.linear_acceleration_covariance[4] = 0.0003;
            imu_msg.linear_acceleration_covariance[8] = 0.0003;
            imu_msg.orientation.x = q[1];
            imu_msg.orientation.y = q[2];
            imu_msg.orientation.z = q[3];
            imu_msg.orientation.w = q[0];
            pub_imu.publish(&imu_msg);

            mag_msg.header.stamp = nh.now();
            mag_msg.header.frame_id = "imu_link";
            mag_msg.magnetic_field.x = mx / uT_2_T;
            mag_msg.magnetic_field.y = my / uT_2_T;
            mag_msg.magnetic_field.z = mz / uT_2_T;
            mag_msg.magnetic_field_covariance[0] = 0.0003;
            mag_msg.magnetic_field_covariance[4] = 0.0003;
            mag_msg.magnetic_field_covariance[8] = 0.0003;
            pub_mag.publish(&mag_msg);

            rpy_msg.x = roll;
            rpy_msg.y = pitch;
            rpy_msg.z = yaw;
            pub_rpy.publish(&rpy_msg);

            tim[1].reset();
        }
        nh.spinOnce();
        wait_ms(1);
    }
}