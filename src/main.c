#include "../include/board.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <../lib/BMP/bmp280.h>
 
static const char* TAG = "MAIN";
 
struct i2c {
  int Name;
  int Type;
};
 
void initI2C() {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_GPIO_SDA,  // select SDA GPIO specific to your project
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = I2C_GPIO_SCL,  // select SCL GPIO specific to your project
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_CLK_SPEED,  // select frequency specific to your project
      .clk_flags = 0,     // optional; you can use I2C_SCLK_SRC_FLAG_* flags to
                          // choose i2c source clock here
  };
 
  i2c_param_config(I2C_NUM_0, &conf);
  esp_err_t espRc = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
  if (espRc == ESP_OK) {
    ESP_LOGD(TAG, "success: i2c installation");
  } else {
    ESP_LOGD(TAG, "failed: i2c installation");
  }
};
 
int8_t userWrite(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len){
    int8_t iError = BMP280_OK;
    esp_err_t espRc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
    if(espRc == ESP_OK){
        iError = BMP280_OK;
    }else {
        iError = BMP280_E_COMM_FAIL;
    }
    i2c_cmd_link_delete(cmd);
    return iError;
}
 
int8_t userRead(uint8_t dev_id, uint8_t reg_add, uint8_t *data, uint16_t len) {
  int8_t iError = BMP280_OK;
  esp_err_t espRc;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_add, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_READ, true);
  if (len > 1) {
    i2c_master_read(cmd, data, len-1, true);
  }else {
    i2c_master_read_byte(cmd, data + (len-1), false);
  }
  i2c_master_stop(cmd);
  espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
  if (espRc == ESP_OK) {
    iError = BMP280_OK;
  } else {
    iError = BMP280_E_COMM_FAIL;
  }
  i2c_cmd_link_delete(cmd);
  return iError;
}
 
void delay (uint32_t period){
  vTaskDelay(period/portTICK_PERIOD_MS);
}
 
int8_t initBMP(){
  struct bmp280_dev dev = {
    .dev_id = BMP280_I2C_ADDR_PRIM,
    .intf = BMP280_I2C_INTF,
    .read = userRead,
    .write = userWrite,
    .delay_ms = delay,  
  };
  bmp280_init(&dev);
  struct bmp280_config conf;
  int8_t rslt = bmp280_get_config(&conf, &dev);
  conf.filter = BMP280_FILTER_COEFF_2;
  conf.os_pres = BMP280_OS_NONE;
  conf.os_temp = BMP280_OS_1X;
  conf.odr = BMP280_ODR_1000_MS;
  rslt = bmp280_set_config(&conf, &dev);
  rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &dev);
 
  struct bmp280_status status;
  bmp280_get_status(&status, &dev);
 
  if (status.im_update == BMP280_IM_UPDATE_DONE &&
      status.measuring == BMP280_MEAS_DONE) {
    struct bmp280_uncomp_data uncomp_data;
    bmp280_get_uncomp_data(&uncomp_data, &dev);
 
    int32_t comp_data;
 
    int8_t a =
        bmp280_get_comp_temp_32bit(&comp_data, uncomp_data.uncomp_temp, &dev);
    ESP_LOGD("DATA", "%ld", comp_data);
    ESP_LOGD("RET", "%d", a);
  }
 
  return rslt;
}
 
void app_main() {
  ESP_LOGD(TAG, "First Program");
  initI2C();
  for (;;)
  {
  delay(1000);	
  int8_t bmp = initBMP();
  }
  
  // esp_err_t getting = i2c_master_read_from_device(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}