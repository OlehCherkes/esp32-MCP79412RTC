#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <esp_log.h>

static const char *TAG = "I2C_INIT";

#define I2C_MASTER_SCL_IO    22          // GPIO SCL
#define I2C_MASTER_SDA_IO    23          // GPIO SDA
#define I2C_MASTER_NUM       I2C_NUM_0   // I2C port
#define I2C_SLAVE_ADDR       0x6F        // Slave address
#define I2C_MASTER_FREQ_HZ   100000      // I2C master frequency

#define TIME_REG 0x00     // Time register
#define ST       7        // Seconds register (TIME_REG) oscillator start/stop bit, 1==Start, 0==Stop
#define HR1224   6        // Hours register (TIME_REG+2) 12 or 24 hour mode (24 hour mode==0)
#define OSCON    5        // Day register (TIME_REG+3) oscillator running (set and cleared by hardware)
#define VBATEN   3        // Day register (TIME_REG+3) VBATEN==1 enables backup battery, VBATEN==0 disconnects the VBAT pin (e.g. to save battery)
#define VBAT     4        // Day register (TIME_REG+3) set by hardware when Vcc fails and RTC runs on battery.
#define LP       5        // Month register (TIME_REG+5) leap year bit


#define OSCRUN   0x20     // Oscillator Status bit
#define PWRFAIL  0x10     // Power Failure Status bit
#define VBEN     0x8      // External Battery Backup Supply (VBAT) Enable bit

typedef enum {
  RTC_OK = 0,
  RTC_FAIL,
  RTC_CONNECTION,
  RTC_LINK,
  RTC_TRANSACTION,
  RTC_OSCILLATOR,
  RTC_POWER,
  RTC_BATTERY
} Rtc_error;

typedef struct {
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
  uint8_t day;
  uint8_t month;
  uint8_t year;
  uint8_t wday;
  uint8_t yday;
  uint8_t isdst;
} DateTime_t;

DateTime_t t, t2 = {};

uint8_t dec2bcd(uint8_t n) {
  return n + 6 * (n / 10);
}

uint8_t bcd2dec(uint8_t n) {
  return n - 6 * (n >> 4);
}

void DataInit(DateTime_t *tm) {
  tm->sec = 50;
  tm->min = 59;
  tm->hour = 23;
  tm->wday = 4;
  tm->day = 31;
  tm->month = 12;
  tm->year = 24;
}

static Rtc_error i2c_init() {
  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master = { .clk_speed = 400000 },
    .clk_flags = 0
  };

  // configuration of I2C controller parameters
  esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure I2C: %d", ret);
    return RTC_CONNECTION;
  }
 
 // settings of the I2C driver
  ret = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install I2C driver: %d", ret);
    return RTC_CONNECTION;
  }

  return RTC_OK;
}

Rtc_error TimeSet(DateTime_t* tm) {
  // creating a new I2C command chain
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if (cmd == NULL) {
    ESP_LOGE(TAG, "Failed to create I2C command link");
    return RTC_LINK;
  }

  // start of I2C transaction
  esp_err_t ret = i2c_master_start(cmd);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start I2C transaction: %d", ret);
    i2c_cmd_link_delete(cmd);
    return RTC_TRANSACTION;
  }

  i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, TIME_REG, true);
  i2c_master_write_byte(cmd, 0x00, true); // stops the oscillator (Bit 7, ST == 0)
  i2c_master_write_byte(cmd, dec2bcd(tm->min), true);
  i2c_master_write_byte(cmd, dec2bcd(tm->hour), true);
  i2c_master_write_byte(cmd, tm->wday | (1 << VBATEN), true); // enable battery backup operation
  i2c_master_write_byte(cmd, dec2bcd(tm->day), true);
  i2c_master_write_byte(cmd, dec2bcd(tm->month), true);
  i2c_master_write_byte(cmd, dec2bcd(tm->year), true);

  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  // creating a new I2C command chain
  cmd = i2c_cmd_link_create();
  if (cmd == NULL) {
    ESP_LOGE(TAG, "Failed to create I2C command link");
    return RTC_CONNECTION;
  }

  // start of I2C transaction
  i2c_master_start(cmd);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start I2C transaction: %d", ret);
    i2c_cmd_link_delete(cmd);
    return RTC_TRANSACTION;
  }

  i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, TIME_REG, true);
  i2c_master_write_byte(cmd, dec2bcd(tm->sec) | (1 << ST), true); // start the oscillator (Bit 7, ST == 1)

  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  return RTC_OK;
}

Rtc_error TimeGetUtc(DateTime_t* tm) {
  // creating a new I2C command chain
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if (cmd == NULL) {
    ESP_LOGE(TAG, "Failed to create I2C command link");
    return RTC_LINK;
  }

  // start of I2C transaction
  esp_err_t ret = i2c_master_start(cmd);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start I2C transaction: %d", ret);
    i2c_cmd_link_delete(cmd);
    return RTC_TRANSACTION;
  }

  i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1), true);
  i2c_master_write_byte(cmd, TIME_REG, true);

  ret = i2c_master_start(cmd);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start I2C transaction: %d", ret);
    i2c_cmd_link_delete(cmd);
    return RTC_TRANSACTION;
  }


  // Request 7 bytes (secs, min, hr, dow, date, mth, yr)
  i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_READ, true);

  i2c_master_read_byte(cmd, &(tm->sec), I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &(tm->min), I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &(tm->hour), I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &(tm->wday), I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &(tm->day), I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &(tm->month), I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &(tm->year), I2C_MASTER_NACK);

  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  // Perform necessary bit masking and BCD to decimal conversion
  tm->sec = bcd2dec(tm->sec & ~(1 << 7));
  tm->hour = bcd2dec(tm->hour & ~(1 << HR1224));
  tm->min = bcd2dec(tm->min);
  tm->wday &= ~((1 << OSCON) | (1 << VBAT) | (1 << VBATEN));
  tm->day = bcd2dec(tm->day);
  tm->month = bcd2dec(tm->month & ~(1 << LP));
  tm->year = bcd2dec(tm->year);

  return RTC_OK;
}

Rtc_error GetRtcStatus() {
  // creating a new I2C command chain
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if (cmd == NULL) {
    ESP_LOGE(TAG, "Failed to create I2C command link");
    return RTC_LINK;
  }

  // start of I2C transaction
  esp_err_t ret = i2c_master_start(cmd);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start I2C transaction: %d", ret);
    i2c_cmd_link_delete(cmd);
    return RTC_TRANSACTION;
  }

  i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1), true);
  i2c_master_write_byte(cmd, 0x3, true);

  ret = i2c_master_start(cmd);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start I2C transaction: %d", ret);
    i2c_cmd_link_delete(cmd);
    return RTC_TRANSACTION;
  }

  i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_READ, true);

  uint8_t data;
  i2c_master_read_byte(cmd, &data, I2C_MASTER_ACK);

  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  // Check status 
  uint8_t scStatusBit = (data & OSCRUN) ? 1 : 0;  // 1 - enabled, 0 - disabled
  if (!scStatusBit) {
    ESP_LOGE(TAG, "Oscillator has stopped or has been disabled");
    return RTC_OSCILLATOR;
  }

  scStatusBit = (data & PWRFAIL) ? 1 : 0; // 1 - Power was lost, 0 - Primary power
  if (scStatusBit) {
    ESP_LOGE(TAG, "Primary power was lost and the power-fail timestamp registers have been loaded");
    return RTC_POWER;
  }

  scStatusBit = (data & VBEN) ? 1 : 0;  // 1 - VBAT input is enabled, 0 - VBAT input is disabled
  if (!scStatusBit) {
    ESP_LOGE(TAG, "VBAT input is disabled");
    return RTC_BATTERY;
  }

  return RTC_OK;
}

void app_main()
{
  i2c_init();

  DataInit(&t);
  TimeSet(&t);
  GetRtcStatus();
  

  for (;;)
  {
    TimeGetUtc(&t2);


    printf("%02u:%02u:%02u %02u/%02u/%02u %02u\n", t2.hour, t2.min, t2.sec, t2.day, t2.month, t2.year, t2.wday);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}