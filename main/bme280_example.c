/* BME280 Example
   More interesting details are in bme280_task.c
*/
#include <stdio.h>

#include "bme280_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

static char* LOG_MAIN = "BME280_main";

void app_main()
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(LOG_MAIN, "This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI(LOG_MAIN, "silicon revision %d, ", chip_info.revision);

    ESP_LOGI(LOG_MAIN, "%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    //TODO: Figure out proper stack size
    xTaskCreate(&task_bme280, "task_bme280", 2048, NULL, 5, NULL);
}
