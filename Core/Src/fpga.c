#include "stm32f4xx_hal.h"
#include "main.h"
#include "fpga.h"
#include "plotter.h"

fpga_buffer_t	fpga_buff = {0};
static fpga_status_t	fpga_sts = FPGA_STATUS_BUSY;

void fpga_sendcplt_cb(SPI_HandleTypeDef *hspi)
{
	msg_t msg = {0};
	msg.msgid = FPGA_SEND_CMPLT_MSG;
	osMessageQueuePut(plotter_queueHandle, &msg,0,0);
}

bool fpga_send(void)
{
	uint16_t* pdata;
	uint16_t len;
	bool ret = false;

	if (fpga_buff.load_cnt > 0)
	{
		pdata = fpga_buff.packet[fpga_buff.rptr].rawdata;
		len = fpga_buff.packet[fpga_buff.rptr].rawdata[0] & 0x00FF;
		ret = (HAL_OK == HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)pdata, len));
	}

	return ret;
}

void fpga_rd_buff_cmplt(void)
{
	fpga_buff.load_cnt--;
	fpga_buff.rptr = (fpga_buff.rptr + 1) % SIZE_OF_FPGA_BUFFER;
}

pl_block_t* fpga_wr_buff_start_pl(void)
{
	if (fpga_buff.load_cnt < SIZE_OF_FPGA_BUFFER)
	{
		return &fpga_buff.packet[fpga_buff.wptr].pl_data;
	}
	else
	{
		return NULL;
	}
}

bool fpga_wr_single_cmd(uint16_t cmd)
{
	if (fpga_buff.load_cnt < SIZE_OF_FPGA_BUFFER)
	{
		fpga_buff.packet[fpga_buff.wptr].one_word = cmd;
		fpga_buff.load_cnt++;
		fpga_buff.wptr = (fpga_buff.wptr + 1) % SIZE_OF_FPGA_BUFFER;
		return true;
	}

	return false;
}

void fpga_wr_buff_cmplt(void)
{
	fpga_buff.load_cnt++;
	fpga_buff.wptr = (fpga_buff.wptr + 1) % SIZE_OF_FPGA_BUFFER;
}

bool fpga_send_ready(void)
{
	if ((FPGA_STATUS_READY == fpga_sts)&&(fpga_buff.load_cnt > 0))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool fpga_wr_ready(void)
{
	return (fpga_buff.load_cnt < FPGA_BUFFER_WR_THRESHOLD);
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
		fpga_sts = FPGA_STATUS_BUSY;
		LED_ORANGE_ON();
	}
	else
	{
		/* falling edge */
		fpga_sts = FPGA_STATUS_READY;
		LED_ORANGE_OFF();
	}

	msg.payload.fpga_sts = fpga_sts;
	msg.msgid = FPGA_STATUS_CHANGE_MSG;
	/* notify spi_task */
	osMessageQueuePut(plotter_queueHandle, &msg,0,0);
}

void fpga_enable(void)
{
	HAL_GPIO_WritePin(FPGA_EN_GPIO_Port, FPGA_EN_Pin, GPIO_PIN_SET);
	LED_GREEN_ON();
}

void fpga_disable(void)
{
	HAL_GPIO_WritePin(FPGA_EN_GPIO_Port, FPGA_EN_Pin, GPIO_PIN_RESET);
	LED_GREEN_OFF();
}
