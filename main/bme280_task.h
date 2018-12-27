#include <esp_log.h>
#include <driver/i2c.h>
#include <bme280.h>

#define ACK_VAL 0x0  /*!< I2C ack value */
#define NACK_VAL 0x1 /*!< I2C nack value */

void task_bme280(void *ignore);
