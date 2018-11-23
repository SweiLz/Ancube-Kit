#include "mbed.h"

/***** Definitions *****/
#define I2C_ADDR            (0xEE) // 1110111x

#define REG_ADDR_RESET      (0xE0)
#define REG_ADDR_ID         (0xD0)
#define REG_ADDR_CTRL       (0xF4)
#define REG_ADDR_DATA       (0xF6)
#define REG_ADDR_AC1        (0xAA)

#define CTRL_REG_TEMP       (0x2E)
#define CTRL_REG_PRESS_0    (0x34)
#define CTRL_REG_PRESS_1    (0x74)
#define CTRL_REG_PRESS_2    (0xB4)
#define CTRL_REG_PRESS_3    (0xF4)


class BMP180
{

public:

    /**
     * @brief   Oversampling ratio.
     * @details Dictates how many pressure samples to take. Conversion time varies
     *          depending on the number of samples taken. Refer to data sheet
     *          for timing specifications.
     */
    typedef enum {
        ULTRA_LOW_POWER       = 0, ///< 1 pressure sample
        STANDARD              = 1, ///< 2 pressure samples
        HIGH_RESOLUTION       = 2, ///< 4 pressure samples
        ULTRA_HIGH_RESOLUTION = 3, ///< 8 pressure samples
    } oversampling_t;

    /**
     * BMP180 constructor.
     *
     * @param sda mbed pin to use for SDA line of I2C interface.
     * @param scl mbed pin to use for SCL line of I2C interface.
     */
    BMP180(PinName sda, PinName scl);

    /**
     * BMP180 constructor.
     *
     * @param i2c I2C object to use.
     */
    BMP180(I2C *i2c);

    /**
     * BMP180 destructor.
     */
    ~BMP180();

    /**
     * @brief   Initialize BMP180.
     * @details Gets the device ID and saves the calibration values.
     * @returns 0 if no errors, -1 if error.
     */
    int init(void);

    /**
     * @brief   Reset BMP180.
     * @details Performs a soft reset of the device. Same sequence as power on reset.
     * @returns 0 if no errors, -1 if error.
     */
    int reset(void);

    /**
     * @brief   Check ID.
     * @details Checks the device ID, should be 0x55 on reset.
     * @returns 0 if no errors, -1 if error.
     */
    int checkId(void);

    /**
     * @brief   Start pressure conversion.
     * @details Initiates the pressure conversion sequence. Refer to data sheet
     *          for timing specifications.
     *
     * @param   oss Number of samples to take.
     * @returns 0 if no errors, -1 if error.
     */
    int startPressure(BMP180::oversampling_t oss);

    /**
     * @brief   Get pressure reading.
     * @details Calculates the pressure using the data calibration data and formula.
     *          Pressure is reported in Pascals.
     * @note    This function should be called after calling startPressure().
     *          Refer to the data sheet for the timing requirements. Calling this
     *          function too soon can result in oversampling.
     *
     * @param   pressure Pointer to store pressure reading.
     * @returns 0 if no errors, -1 if error.
     */
    int getPressure(int *pressure);

    /**
     * @brief   Start temperature conversion.
     * @details Initiates the temperature conversion sequence. Refer to data
     *          sheet for timing specifications.
     * @returns 0 if no errors, -1 if error.
     */
    int startTemperature(void);

    /**
     * @brief   Get temperature reading.
     * @details Calculates the temperature using the data calibration data and formula.
     *          Temperature is reported in degrees Celcius.
     *
     * @note    This function should be called after calling startTemperature().
     *          Refer to the data sheet for the timing requirements. Calling this
     *          function too soon can result in oversampling.
     *
     * @param   tempC Pointer to store temperature reading.
     * @returns 0 if no errors, -1 if error.
     */
    int getTemperature(float *tempC);

    /**
     * @brief   Get temperature reading.
     * @details Calculates the temperature using the data calibration data and formula.
     *          Temperature is reported in 1/10ths degrees Celcius.
     *
     * @note    This function should be called after calling startTemperature().
     *          Refer to the data sheet for the timing requirements. Calling this
     *          function too soon can result in oversampling.
     *
     * @param   tempCx10 Pointer to store temperature reading.
     * @returns 0 if no errors, -1 if error.
     */
    int getTemperature(int16_t *tempCx10);

private:

    typedef union {
        uint16_t value[11];
        struct {
            int16_t ac1;
            int16_t ac2;
            int16_t ac3;
            uint16_t ac4;
            uint16_t ac5;
            uint16_t ac6;
            int16_t b1;
            int16_t b2;
            int16_t mb;
            int16_t mc;
            int16_t md;
        };
    } calibration_t;

    I2C *i2c_;
    bool i2c_owner;

    BMP180::calibration_t calib;
    int32_t b5;
    BMP180::oversampling_t oss_;
};