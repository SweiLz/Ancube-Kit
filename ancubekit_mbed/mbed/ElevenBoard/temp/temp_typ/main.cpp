#include <mbed.h>
#include <BMP180.h>
#include <MPU9250.h>

#define G_2_MPSS 9.80665
#define DPS_2_RPS 0.0174533
#define uT_2_T 1000000

Serial pc(USBTX, USBRX);
I2C i2c(PB_9, PB_8);

// BMP180 bmp180(&i2c);
MPU9250 imu(&i2c);

char log_msg[255];

int main(int argc, char **argv)
{
    pc.baud(115200);
    pc.printf("Hello World\n");
    // uint8_t whoami = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250); // Read WHO_AM_I register for MPU-9250
    // pc.printf("I AM 0x%x\n\r", whoami);
    // pc.printf("I SHOULD BE 0x71\n\r");

    wait(1);
    imu.resetMPU9250();                                // Reset registers to default in preparation for device calibration
    imu.calibrateMPU9250(imu.gyroBias, imu.accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
    imu.initMPU9250();
    imu.initAK8963(imu.magCalibration);
    wait(2);
    imu.getAres(); // Get accelerometer sensitivity
    imu.getGres(); // Get gyro sensitivity
    imu.getMres(); // Get magnetometer sensitivity
    pc.printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f / imu.aRes);
    pc.printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f / imu.gRes);
    pc.printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f / imu.mRes);
    imu.magbias[0] = 0; // User environmental x-axis correction in milliGauss, should be automatically calculated
    imu.magbias[1] = 0; // User environmental x-axis correction in milliGauss
    imu.magbias[2] = 0;

    // while (true)
    // {
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
    // if (imu.begin())
    // {
    //     pc.printf("Unable to communicate with MPU-9250");
    //     pc.printf("Check connections, and try again.\n");
    // }
    // else
    // {
    //     pc.printf("imu.begin() suceeded\n");

    //         break;

    // }
    //     wait(1);
    // }

    Timer tim[5];
    tim[0].start(); //Debug
    tim[1].start(); //IMU
    while (1)
    {

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
            // if (imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
            // { // On interrupt, check if data ready interrupt

                imu.readAccelData(imu.accelCount); // Read the x/y/z adc values
                // Now we'll calculate the accleration value into actual g's
                imu.ax = (float)imu.accelCount[0] * imu.aRes - imu.accelBias[0]; // get actual g value, this depends on scale being set
                imu.ay = (float)imu.accelCount[1] * imu.aRes - imu.accelBias[1];
                imu.az = (float)imu.accelCount[2] * imu.aRes - imu.accelBias[2];

                imu.readGyroData(imu.gyroCount); // Read the x/y/z adc values
                // Calculate the gyro value into actual degrees per second
                imu.gx = (float)imu.gyroCount[0] * imu.gRes - imu.gyroBias[0]; // get actual gyro value, this depends on scale being set
                imu.gy = (float)imu.gyroCount[1] * imu.gRes - imu.gyroBias[1];
                imu.gz = (float)imu.gyroCount[2] * imu.gRes - imu.gyroBias[2];

                imu.readMagData(imu.magCount); // Read the x/y/z adc values
                // Calculate the magnetometer values in milliGauss
                // Include factory calibration per data sheet and user environmental corrections
                imu.mx = (float)imu.magCount[0] * imu.mRes * imu.magCalibration[0] - imu.magbias[0]; // get actual magnetometer value, this depends on scale being set
                imu.my = (float)imu.magCount[1] * imu.mRes * imu.magCalibration[1] - imu.magbias[1];
                imu.mz = (float)imu.magCount[2] * imu.mRes * imu.magCalibration[2] - imu.magbias[2];
            // }

            imu.Now = tim[1].read_us();
            imu.deltat = (float)((imu.Now - imu.lastUpdate) / 1000000.0f);
            imu.lastUpdate = imu.Now;
            imu.MahonyQuaternionUpdate(imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.mx, imu.my, imu.mz);
            pc.printf("$,%f,%f,%f,%f,*\n", imu.q[0], imu.q[1], imu.q[2], imu.q[3]);
            // pc.printf("%f, %f, %f\n", imu.ax*G_2_MPSS, imu.ay*G_2_MPSS, imu.az*G_2_MPSS);
            // pc.printf("%f, %f, %f\n", imu.gx, imu.gy, imu.gz);

            // if (imu.gz > .3 || imu.gz < -.3)
            // {
            //     yaw = (yaw - tim[1].read() * imu.gz + drift);
            //     // t.reset();
            //     if (yaw > 360)
            //         yaw -= 360;
            //     if (yaw < 0)
            //         yaw += 360;
            //     pc.printf("Yaw: %f \n\r", yaw);
            // }
            // pc.printf("%f, %f, %f\n", g0, g1, g2);

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
