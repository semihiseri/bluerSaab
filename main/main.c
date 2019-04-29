#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"


#include "driver/gpio.h"

//#include "bluetoothAudio.h"
#include "CAN.h"
#include "CDC.h"

TickType_t lastInterrupt;

#define ESP_INTR_FLAG_DEFAULT 0

#define MCP2515_INT GPIO_NUM_13

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	lastInterrupt = xTaskGetTickCount()*portTICK_RATE_MS/1000;
	return;
}

void sleeper()
{

	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
	//set as output mode
	io_conf.mode = GPIO_MODE_INPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = 	(((uint64_t) 1)<<MCP2515_INT);
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);

	//install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(MCP2515_INT, gpio_isr_handler, NULL);

	TickType_t now = xTaskGetTickCount()*portTICK_RATE_MS/1000;
	lastInterrupt = now;
	
	while (1)
	{
		now = xTaskGetTickCount()*portTICK_RATE_MS/1000;


		printf("sleeper: %d and %d\n", now, lastInterrupt);

		if (now-lastInterrupt > 10)
		{
			esp_sleep_enable_ext0_wakeup(MCP2515_INT, 0);
			esp_deep_sleep_start();
		}

		if (now-lastInterrupt > 5)
		{
			printf("Sleeping in %d seconds\n", 10-now+lastInterrupt);
		}

		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

void app_main()
{
//	startBluetooth();

	//printf("Now power the peripherals\n");
	//vTaskDelay(5000/portTICK_RATE_MS);
	//printf("Program started\n");


	//CAN_begin(47);
	CDC_openCanBus();

	xTaskCreate(&sleeper, "sleeper", 2048, NULL, 3, NULL);
	

	while(1)
	{

		CDC_handleCdcStatus();
	/*	printf("%s\n", getTitle());
		printf("%s\n", getArtist());*/
		vTaskDelay(1/portTICK_RATE_MS);
	}


	
   /* CDC.openCanBus();
while(1)
{
 CDC.handleCdcStatus();
}*/


}



