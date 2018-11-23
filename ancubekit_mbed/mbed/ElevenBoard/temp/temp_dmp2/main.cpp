#include <mbed.h>
// #include <BMP180.h>
#include <SparkFunMPU9250-DMP.h>
#include <mdcompat.h>

#define G_2_MPSS 9.80665
#define DPS_2_RPS 0.0174533
#define uT_2_T 1000000

Serial pc(USBTX, USBRX);

I2C i2c(PB_9, PB_8);

// BMP180 bmp180(&i2c);
float temp;
int32_t pressure, pressure0;

MPU9250_DMP imu;

char log_msg[255];
// const signed char orientationMatrix[9] = {
//     -0.7071068,  0.0000000,  0.7071068,
//    0.0000000,  1.0000000,  0.0000000,
// -0.7071068,  0.0000000,  -0.7071068};

int main(int argc, char **argv)
{
    pc.baud(115200);
    pc.printf("Hello World\n");
    imu_init();
    stamper_init();

    while (true)
    {
        // if (bmp180.init() != 0)
        // {
        //     pc.printf("Error communicating with BMP180\n");
        // }
        // else
        // {
        //     pc.printf("Initialized BMP180\n");
        //     temp = bmp180.getTemperature();
        //     pressure0 = bmp180.getPressure();
        // }
        if (imu.begin() != INV_SUCCESS)
        {
            pc.printf("Unable to communicate with MPU-9250");
            pc.printf("Check connections, and try again.\n");
        }
        else
        {
            pc.printf("imu.begin() suceeded\n");

            imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
            // imu.setGyroFSR(2000);
            // imu.setAccelFSR(2);
            // imu.setLPF(5);
            // imu.setSampleRate(10);
            // imu.setCompassSampleRate(10);

            // imu.dmpBegin(DMP_FEATURE_GYRO_CAL |         // Enable gyro cal
            //                  DMP_FEATURE_SEND_CAL_GYRO, // Send cal'd gyro values
            //              10);
            // imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |
            //                  DMP_FEATURE_GYRO_CAL,
            //              10);

            // break;
            if (imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL | // Send accelerometer data
                                 DMP_FEATURE_SEND_RAW_GYRO |
                                 DMP_FEATURE_GYRO_CAL |      // Calibrate the gyro data
                                 DMP_FEATURE_SEND_CAL_GYRO | // Send calibrated gyro data
                                 DMP_FEATURE_6X_LP_QUAT,     // Calculate quat's with accel/gyro, // Use gyro calibration
                             10) == INV_ERROR)
            {                                            // Set DMP FIFO rate to 150 Hz
                pc.printf("imu.dmpBegin have failed\n"); //dmpLoad function under it fails which is caused by memcmp(firmware+ii, cur, this_write) (it is located at row 2871 of inv_mpu.c)
            }
            else
            {
                // imu.dmpSetOrientation(orientationMatrix);
                pc.printf("imu.dmpBegin() suceeded\n");
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
            imu.update();
            //     float accelX = imu.calcAccel(imu.ax) * G_2_MPSS; // accelX is x-axis acceleration in g's
            //     float accelY = imu.calcAccel(imu.ay) * G_2_MPSS; // accelY is y-axis acceleration in g's
            //     float accelZ = imu.calcAccel(imu.az) * G_2_MPSS; // accelZ is z-axis acceleration in g's

            //     float gyroX = imu.calcGyro(imu.gx) * DPS_2_RPS; // gyroX is x-axis rotation in dps
            //     float gyroY = imu.calcGyro(imu.gy) * DPS_2_RPS; // gyroY is y-axis rotation in dps
            //     float gyroZ = imu.calcGyro(imu.gz) * DPS_2_RPS; // gyroZ is z-axis rotation in dps

            //     float magX = imu.calcMag(imu.mx) / uT_2_T; // magX is x-axis magnetic field in uT
            //     float magY = imu.calcMag(imu.my) / uT_2_T; // magY is y-axis magnetic field in uT
            //     float magZ = imu.calcMag(imu.mz) / uT_2_T; // magZ is z-axis magnetic field in uT

            //     pc.printf("%f, %f, %f\n", accelX, accelY, accelZ);
            //     pc.printf("%f, %f, %f\n", gyroX, gyroY, gyroZ);
            //     pc.printf("%f, %f, %f\n\n", magX, magY, magZ);
        }
        if (imu.fifoAvailable())
        {
            if (imu.dmpUpdateFifo() == INV_SUCCESS)
            {
                imu.computeEulerAngles();

                float accelX = imu.calcAccel(imu.ax) * G_2_MPSS; // accelX is x-axis acceleration in g's
                float accelY = imu.calcAccel(imu.ay) * G_2_MPSS; // accelY is y-axis acceleration in g's
                float accelZ = imu.calcAccel(imu.az) * G_2_MPSS; // accelZ is z-axis acceleration in g's

                float gyroX = imu.calcGyro(imu.gx) * DPS_2_RPS; // gyroX is x-axis rotation in dps
                float gyroY = imu.calcGyro(imu.gy) * DPS_2_RPS; // gyroY is y-axis rotation in dps
                float gyroZ = imu.calcGyro(imu.gz) * DPS_2_RPS; // gyroZ is z-axis rotation in dps

                float magX = imu.calcMag(imu.mx) / uT_2_T; // magX is x-axis magnetic field in uT
                float magY = imu.calcMag(imu.my) / uT_2_T; // magY is y-axis magnetic field in uT
                float magZ = imu.calcMag(imu.mz) / uT_2_T; // magZ is z-axis magnetic field in uT

                float q0 = imu.calcQuat(imu.qw);
                float q1 = imu.calcQuat(imu.qx);
                float q2 = imu.calcQuat(imu.qy);
                float q3 = imu.calcQuat(imu.qz);
                float norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
                q0 = q0 / norm;
                q1 = q1 / norm;
                q2 = q2 / norm;
                q3 = q3 / norm;

                // pc.printf("%f, %f, %f\n", accelX, accelY, accelZ);
                // pc.printf("%f, %f, %f\n", gyroX, gyroY, gyroZ);
                // pc.printf("%f, %f, %f\n", magX, magY, magZ);
                // pc.printf("%f, %f, %f, %f\n", q0, q1, q2, q3);
                // pc.printf("%f, %f, %f\n\n", imu.roll, imu.pitch, imu.yaw);
                // pc.printf("$,%f,%f,%f,%f,*\n", q0, q1, q2, q3);
                // pc.printf("%f, %f, %f\n", accelX, accelY, accelZ);
                // pc.printf("%f, %f, %f\n", gyroX, gyroY, gyroZ);
                // pc.printf("%f, %f, %f\n", magX, magY, magZ);
            }
        }
        if (tim[0].read_ms() > 1000)
        {
            // temp = bmp180.getTemperature();
            // pressure = bmp180.getPressure();
            //  pc.printf("%d\n",imu.dmpGetOrientation());
            // sprintf(log_msg, "Altitude = %f m\n", bmp180.calAltitude(pressure, pressure0));
            // printf(log_msg);
            // nh.loginfo(log_msg);

            // sonar_msg.data[0] = 1;
            // sonar_msg.data[1] = 2;
            // sonar_msg.data[2] = 3;
            // sonar_msg.data[3] = 4;
            // pub_sonar.publish(&sonar_msg);
            tim[0].reset();
        }
        if (tim[1].read_ms() > 100)
        {

            // if (imu.fifoAvailable())
            // {
            //     //     imu.update();
            //     if (imu.dmpUpdateFifo() == INV_SUCCESS)
            //     {
            //         //         imu.computeEulerAngles();
            //         // float accelX = imu.calcAccel(imu.ax); // accelX is x-axis acceleration in g's
            //         // float accelY = imu.calcAccel(imu.ay); // accelY is y-axis acceleration in g's
            //         // float accelZ = imu.calcAccel(imu.az); // accelZ is z-axis acceleration in g's

            //         // float gyroX = imu.calcGyro(imu.gx); // gyroX is x-axis rotation in dps
            //         // float gyroY = imu.calcGyro(imu.gy); // gyroY is y-axis rotation in dps
            //         // float gyroZ = imu.calcGyro(imu.gz); // gyroZ is z-axis rotation in dps

            //         // float magX = imu.calcMag(imu.mx); // magX is x-axis magnetic field in uT
            //         // float magY = imu.calcMag(imu.my); // magY is y-axis magnetic field in uT
            //         // float magZ = imu.calcMag(imu.mz); // magZ is z-axis magnetic field in uT

            //         float q0 = imu.calcQuat(imu.qw);
            //         float q1 = imu.calcQuat(imu.qx);
            //         float q2 = imu.calcQuat(imu.qy);
            //         float q3 = imu.calcQuat(imu.qz);

            //         // pc.printf("$,%.4lf,%.4lf,%.4lf,*\n", imu.ax, imu.ay, imu.az);
            //         pc.printf("$,%.4lf,%.4lf,%.4lf,%.4lf,*\n", q0, q1, q2, q3);
            //         // pc.printf("%f, %f, %f\n", a0, a1, a2);
            //         // pc.printf("%f, %f, %f\n", g0, g1, g2);
            //     }
            // }

            tim[1].reset();
        }
    }
}
