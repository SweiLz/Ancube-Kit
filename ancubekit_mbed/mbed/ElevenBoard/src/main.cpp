#include <mbed.h>
#include <ros.h>
#include <ros/time.h>

#define G_2_MPSS 9.80665
#define DPS_2_RPS 0.0174533
#define uT_2_T 1000000

DigitalOut myled(LED1);
// Serial pc(USBTX, USBRX);

Timer tim[4];

char log_msg[255];

ros::NodeHandle nh;

int main(int argc, char **argv)
{
    nh.getHardware()->setBaud(250000);
    nh.initNode();
    // pc.baud(115200);


    // magbias[0] = +470.; // User environmental x-axis correction in milliGauss, should be automatically calculated
    // magbias[1] = +120.; // User environmental x-axis correction in milliGauss
    // magbias[2] = +125.; // User environmental x-axis correction in milliGauss
    tim[0].start();
    tim[1].start();

    while (1)
    {
        if (tim[0].read_ms() > 200)
        {
            myled = !myled;
            tim[0].reset();
        }
        if (tim[1].read_ms() > 1000)
        {
            sprintf(log_msg, "Hello World\n");
            nh.loginfo(log_msg);
            tim[1].reset();
        }
        if (tim[2].read_ms() > 100)
        {
            tim[2].reset();
        }
        nh.spinOnce();
        wait_ms(1);
    }
}