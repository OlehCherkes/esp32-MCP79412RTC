#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>

#define I2C_MASTER_SCL_IO    22    /*!< GPIO номер для SCL */
#define I2C_MASTER_SDA_IO    23    /*!< GPIO номер для SDA */
#define I2C_MASTER_NUM       I2C_NUM_0   /*!< I2C порт номер */
#define I2C_SLAVE_ADDR       0x6F  /*!< Адрес устройства I2C slave */
#define I2C_MASTER_FREQ_HZ   100000     /*!< Частота I2C мастера */

#define TIME_REG 0x00
#define ST 7        // Seconds register (TIME_REG) oscillator start/stop bit, 1==Start, 0==Stop
#define HR1224 6    // Hours register (TIME_REG+2) 12 or 24 hour mode (24 hour mode==0)
#define OSCON 5     // Day register (TIME_REG+3) oscillator running (set and cleared by hardware)
#define VBATEN 3    // Day register (TIME_REG+3) VBATEN==1 enables backup battery, VBATEN==0 disconnects the VBAT pin (e.g. to save battery)
#define VBAT 4      // Day register (TIME_REG+3) set by hardware when Vcc fails and RTC runs on battery.
#define LP 5        // Month register (TIME_REG+5) leap year bit

#define tmYearToCalendar(Y) ((Y) + 1970)  // full four digit year 
#define CalendarYrToTm(Y) ((Y) - 1970)

struct tmElements_t {
  uint8_t Second; // секунды (0-59)
  uint8_t Minute; // минуты (0-59)
  uint8_t Hour; // часы (0-23)
  uint8_t Wday; // день недели (0-6, 0 = воскресенье)
  uint8_t Day; // день (1-31)
  uint8_t Month; // месяц (1-12)
  uint16_t Year; // год
};

struct tmElements_t tm, tm2 = {};

uint8_t dec2bcd(uint8_t n)
{
  return n + 6 * (n / 10);
}

uint8_t bcd2dec(uint8_t n) {
  return n - 6 * (n >> 4);
}

static void i2c_init() {
  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000
  };

  i2c_param_config(I2C_NUM_0, &conf);
  i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

void write_RTC(struct tmElements_t *tm) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1)  /* | I2C_MASTER_WRITE */, true);
  i2c_master_write_byte(cmd, TIME_REG, true);
  i2c_master_write_byte(cmd, 0x00, true); // stops the oscillator (Bit 7, ST == 0)
  i2c_master_write_byte(cmd, dec2bcd(tm->Minute), true);
  i2c_master_write_byte(cmd, dec2bcd(tm->Hour), true);
  i2c_master_write_byte(cmd, tm->Wday | (1 << VBATEN), true); // enable battery backup operation
  i2c_master_write_byte(cmd, dec2bcd(tm->Day), true);
  i2c_master_write_byte(cmd, dec2bcd(tm->Month), true);
  i2c_master_write_byte(cmd, dec2bcd(CalendarYrToTm(tm->Year)), true);
  i2c_master_stop(cmd);
  if (i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS) == ESP_OK)
    printf("Send data\n");
  i2c_cmd_link_delete(cmd);

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) /*| I2C_MASTER_WRITE */, true);
  i2c_master_write_byte(cmd, TIME_REG, true);
  i2c_master_write_byte(cmd, dec2bcd(tm->Second) | (1 << ST), true); // start the oscillator (Bit 7, ST == 1)
  i2c_master_stop(cmd);
  if (i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS) == ESP_OK)
    printf("Send data\n");
  i2c_cmd_link_delete(cmd);
}

void read_RTC(struct tmElements_t *tm) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) /* | I2C_MASTER_READ */, true);
  i2c_master_write_byte(cmd, TIME_REG, true);

  i2c_master_start(cmd);

  // Request 7 bytes (secs, min, hr, dow, date, mth, yr)
  i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_READ, true);

  i2c_master_read_byte(cmd, &(tm->Second), I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &(tm->Minute), I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &(tm->Hour), I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &(tm->Wday), I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &(tm->Day), I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &(tm->Month), I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &(tm->Year), I2C_MASTER_NACK);

  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  // Perform necessary bit masking and BCD to decimal conversion
  tm->Second = bcd2dec(tm->Second & ~(1 << 7));
  tm->Hour = bcd2dec(tm->Hour & ~(1 << HR1224));
  tm->Wday &= ~((1 << OSCON) | (1 << VBAT) | (1 << VBATEN));
  tm->Month = bcd2dec(tm->Month & ~(1 << LP));
  tm->Year = tmYearToCalendar(bcd2dec(tm->Year)); 
}

void app_main()
{
  i2c_init();

//  tm.Second = 0;
//  tm.Minute = 2;
//  tm.Hour = 1;
//  tm.Wday = 1;
//  tm.Day = 3;
//  tm.Month = 4;
//  tm.Year = 2024;
//  write_RTC(&tm);

  for (;;)
  {
    
    read_RTC(&tm2);

    printf("%02u:%02u:%02u %02u/%02u/%04u %02u\n", tm2.Hour, tm2.Minute, tm2.Second, tm2.Day, tm2.Month, tm2.Year, tm2.Wday);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

}