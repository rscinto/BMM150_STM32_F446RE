#ifndef _BMM150_H_
#define _BMM150_H_


/** Includes */
#include "bmm150_defs.h"
#include "stdint.h"
#include "stm32f4xx_hal.h"

class BMM150 {

  public:
    BMM150();
    /**
        \brief initialze device

    */
    int8_t initialize(I2C_HandleTypeDef* hi2c1);

    /**
        \brief Read magnetometer data
    */
    void read_mag_data(I2C_HandleTypeDef* hi2c1);

    /**
        @brief This internal API is used to obtain the compensated
        magnetometer x axis data(micro-tesla) in float.
    */
    int16_t compensate_x(int16_t mag_data_z, uint16_t data_rhall);

    /**
        @brief This internal API is used to obtain the compensated
        magnetometer Y axis data(micro-tesla) in int16_t.
    */
    int16_t compensate_y(int16_t mag_data_z, uint16_t data_rhall);

    /**
        @brief This internal API is used to obtain the compensated
        magnetometer Z axis data(micro-tesla) in int16_t.
    */
    int16_t compensate_z(int16_t mag_data_z, uint16_t data_rhall);

    /**
        \brief Set power mode
    */
    void set_op_mode(uint8_t op_mode, I2C_HandleTypeDef* hi2c1);

    /**
        @brief This internal API reads the trim registers of the sensor and stores
        the trim values in the "trim_data" of device structure.
    */
    void read_trim_registers(I2C_HandleTypeDef* hi2c1);

    /**
        @brief This internal API writes the op_mode value in the Opmode bits
        (bits 1 and 2) of 0x4C register.
    */
    void write_op_mode(uint8_t op_mode, I2C_HandleTypeDef* hi2c1);

    /**
        \brief Set preset mode mode
    */
    void set_preset_mode(uint8_t mode);

    /**
        @brief This internal API sets/resets the power control bit of 0x4B register.
    */
    void set_power_control_bit(uint8_t pwrcntrl_bit, I2C_HandleTypeDef* hi2c1);

    /**
        @brief This internal API sets the device from suspend to sleep mode
        by setting the power control bit to '1' of 0x4B register
    */
    void suspend_to_sleep_mode(I2C_HandleTypeDef* hi2c1);

    /**
        @brief This API is used to set the preset mode of the sensor.
    */
    void set_presetmode(uint8_t preset_mode, I2C_HandleTypeDef* hi2c1);

    /**
        Self test functionality
    */
    /*
        int8_t perform_self_test(uint8_t self_test_mode);
        int8_t perform_normal_self_test();
        void enable_normal_self_test(uint8_t *self_test_enable);
        int8_t validate_normal_self_test();
        int8_t perform_adv_self_test();
        void adv_self_test_settings();
        void adv_self_test_measurement(uint8_t self_test_current, int16_t *data_z);
        int8_t validate_adv_self_test(int16_t positive_data_z, int16_t negative_data_z);
        void set_adv_self_test_current(uint8_t self_test_current);
        void set_control_measurement_xyz(struct bmm150_settings settings);
    */

    /**
        @brief This internal API sets the preset mode ODR and repetition settings.
    */
    void set_odr_xyz_rep(struct bmm150_settings settings, I2C_HandleTypeDef* hi2c1);

    /**
        @brief This internal API sets the xy repetition value in the 0x51 register.
    */
    void set_xy_rep(struct bmm150_settings settings, I2C_HandleTypeDef* hi2c1);

    /**
        @brief This internal API sets the z repetition value in the 0x52 register.
    */
    void set_z_rep(struct bmm150_settings settings, I2C_HandleTypeDef* hi2c1);

    /**
        @brief This internal API is used to set the output data rate of the sensor.
    */
    void set_odr(struct bmm150_settings settings, I2C_HandleTypeDef* hi2c1);

    /**
        @brief This API is used to perform soft-reset of the sensor
        where all the registers are reset to their default values except 0x4B.
    */
    void soft_reset(I2C_HandleTypeDef* hi2c1);


    /**

    */
    //   char* getErrorText(short errorCode);


    // protected:
    struct bmm150_settings settings;
    struct bmm150_raw_mag_data raw_mag_data;
    struct bmm150_mag_data mag_data;
    struct bmm150_trim_registers trim_data;


    void i2c_write(short address, short byte, I2C_HandleTypeDef* hi2c1);
    void i2c_read(short mem_address, int8_t* buffer, short length, short bytes_to_grab, I2C_HandleTypeDef* hi2c1);
    void i2c_read(short address, uint8_t* buffer, short length, I2C_HandleTypeDef* hi2c1);
    void i2c_read(short address, int8_t* buffer, short length, I2C_HandleTypeDef* hi2c1);
    uint8_t i2c_read(uint16_t address, I2C_HandleTypeDef* hi2c1);

};


#endif
