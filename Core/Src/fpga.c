#include "stm32f4xx_hal.h"
#include "main.h"
#include "fpga.h"
#include "plotter.h"

void fpga_sendcplt_cb(SPI_HandleTypeDef *hspi)
{
	msg_t msg = {0};
	msg.msgid = FPGA_SEND_CMPLT_MSG;
	osMessageQueuePut(plotter_queueHandle, &msg,0,0);
}

bool fpga_send(uint16_t* pdata, uint16_t len)
{
	return (HAL_OK == HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)pdata, len));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	msg_t msg = {0};
	/* verify FPGA_pin */
	if (GPIO_Pin != FPGA_BUSY_Pin)
	{
		return;
	}

	if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, FPGA_BUSY_Pin))
	{
		/* rising edge */
		msg.payload.fpga_sts = FPGA_STATUS_BUSY;

	}
	else
	{
		/* falling edge */
		msg.payload.fpga_sts = FPGA_STATUS_READY;
	}

	msg.msgid = FPGA_STATUS_CHANGE_MSG;
	/* notify spi_task */
	osMessageQueuePut(plotter_queueHandle, &msg,0,0);
}
