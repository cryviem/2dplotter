#include "stm32f4xx_hal.h"
#include "main.h"
#include "app_msg.h"

extern SPI_HandleTypeDef hspi2;
uint16_t txdata_db[4] = {0x8001, 0x8002, 0x8003, 0x8004};

void fpga_sendcplt_cb(SPI_HandleTypeDef *hspi)
{
	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
}

void fpga_button_pressed(void)
{
	HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)txdata_db, 4);
	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}

void fpga_transmit_task(void)
{
	msg_t msg = {0};
	osStatus_t ret;

	while(1)
	{
		ret = osMessageQueueGet(spi_queueHandle, &msg, (void*)0, 5000 );
		if (osOK != ret)
		{
			/**/
		}
		else
		{
			switch (msg.msgid)
			{
			case FPGA__STATUS_CHANGE_MSG:

				break;

			default:
				break;
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* verify FPGA_pin */
	if (GPIO_Pin != FPGA_BUSY_Pin)
	{
		return;
	}

	if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, FPGA_BUSY_Pin))
	{
		/* rising edge */


	}
	else
	{
		/* falling edge */

	}

	/* notify spi_task */
	osMessageQueuePut(spi_queueHandle,0,0,0);
}
