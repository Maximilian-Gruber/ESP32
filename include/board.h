// used for PIN definition board configuration, description of configuration at board

#define ESP32 1

#ifdef ESP32

    #define I2C_GPIO_SDA 21

    #define I2C_GPIO_SCL 22

    #define I2C_CLK_SPEED 100*1000

#endif