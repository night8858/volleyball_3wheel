  
  // ██╗    ██╗  ██╗  ██╗ ███████╗ ███████╗ ██╗     
  // ██║    ██║  ██║  ██║ ██╔════╝ ██╔════╝ ██║     
  // ██║ ██╗ ██║ ███████║ ██████╗  ██████╗  ██║     
  // ██║ ╚██╗██║ ██╔══██║ ██╔══╝   ██╔══╝   ██║     
  // ╚██╗╚████║  ██║  ██║ ███████╗ ███████╗ ███████╗
  //  ╚═╝ ╚═══╝  ╚═╝  ╚═╝ ╚══════╝ ╚══════╝ ╚══════╝
  
  
  
  /*
  *   █████████████╗   ███████████╗     ████████████╗
  *   ╚════██╔═════╝  ██╔════════██╗    ██╔═════════╝
  *        ██║        ██║        ██║    ██║
  *        ██║        ██║        ██║    ██║
  *        ██║        ██║        ██║    ██║█████████╗
  *        ██║        ██║        ██║    ██╔═════════╝    
  *        ██║        ██║        ██║    ██║    
  *        ██║        ██║        ██║    ██║ 
  *        ██║        ╚███████████╔╝    ████████████╗
  *        ╚═╝         ╚══════════╝     ╚═══════════╝
  *  Version    Date            Author          Modification
  *  V1.0.0     2024            night8858       
  */


  //负责ops9的数据接收
  
#include "sensor.h"
#include "stm32f407xx.h"
#include "Variables.h"
#include "Monitor.h"
#include "gpio.h"
#include "main.h"
#include <stdio.h>
#include "usart.h"

#define RX_BUF_NUM 32u

#define FRAME_LENGTH 28u

extern s_robo_Mode_Setting robot_StateMode;
extern s_task_flags task_flags;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

int ops9_flag = 0;
s_ops9_data_t ops9_data;      //ops9数据结构体

uint8_t RX3_Buf[50]  = {0};
uint8_t TX3_Buf[256] = {0};
uint8_t TX3_Count = 0;
uint8_t Count3 = 0;

static uint8_t ops9_rx_buf[2][32];    //接收缓冲区

static union
{
	uint8_t data[24];
	float Val[6];
	
}posture;


static void USART6_data_receive_prepare(uint8_t data);
static void ops9_data_receive(volatile const uint8_t *data ,  s_ops9_data_t *s_ops9_data);
static void usart6_DMA_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

void ops9_init(void)
{
    usart6_DMA_init(ops9_rx_buf[0], ops9_rx_buf[1], 32);
}

void usart6_DMA_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart6_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);

}


void USART6_IRQHandler(void)
{
    if(huart6.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if(USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if(this_time_rx_len == FRAME_LENGTH)
            {
                ops9_data_receive( ops9_rx_buf[0] , &ops9_data);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if(this_time_rx_len == FRAME_LENGTH)
            {
                //处理遥控器数据
                ops9_data_receive( ops9_rx_buf[1] , &ops9_data);
            }
        }
    }
}

void ops9_data_receive(volatile const uint8_t *data ,  s_ops9_data_t *s_ops9_data)
{
    if (data == NULL)
    {
    return ;       
    }
    
    if(data[0] != 0x0D || data[1] != 0x0A)
    {
        return;
    }

    for (uint8_t i = 0; i < 24; i++)
    {
        posture.data[i] = data[i + 2];
    }

    s_ops9_data->yaw   =  posture.Val[0] ;
    s_ops9_data->pitch =  posture.Val[1] ;
    s_ops9_data->roll  =  posture.Val[2] ;
    s_ops9_data->pos_x =  posture.Val[3] ;
    s_ops9_data->pos_y =  posture.Val[4] ;
    s_ops9_data->w_z   =  posture.Val[5] ;

    if (s_ops9_data->yaw < -135)
    {
        s_ops9_data->yaw += 360;
    }
    
    //这里出现了进位损失导致数据出错。
    // s_ops9_data->yaw   =  (data[2]  | (data[3]  << 8) | (data[4]  << 16) | (data[5]  << 24)) ;
    // s_ops9_data->pitch =  data[6]  | (data[7]  << 8) | (data[8]  << 16) | (data[9]  << 24) ;
    // s_ops9_data->roll  =  data[10] | (data[11] << 8) | (data[12] << 16) | (data[13] << 24);
    // s_ops9_data->pos_x =  data[14] | (data[15] << 8) | (data[16] << 16) | (data[17] << 24);
    // s_ops9_data->pos_y =  data[18] | (data[19] << 8) | (data[20] << 16) | (data[21] << 24);
    // s_ops9_data->w_z   =  data[22] | (data[23] << 8) | (data[24] << 16) | (data[25] << 24);

    if (data[26] != 0x0A || data[27] != 0x0D)
    {
        s_ops9_data = 0;
        return;
    }
    ops9_flag = 1;
}


void Strcat(char str1[],char str2[],uint8_t num)
{
	int i=0,j=0;

	while(str1[i]!='\0')
	i++;
	
	for(j=0;j<num;j++)
	{
		str1[i++]=str2[j];
	}
}

/* 向opes发送数据 */

void usart6_send(char *data, uint8_t num)
{
	HAL_UART_Transmit(&huart6, data, num, 0x01);
}


/* 数据清零 */
void ops9_Zero_Clearing(void)
{
	char zero_clear[4] = "ACT0";
	usart6_send(zero_clear,sizeof(zero_clear));
}



/* 设定当前yaw角度 - -180~180 */
void Update_A(float angle)
{
	char update_yaw[8] = "ACTJ";
	static union
	{
		float A;
		char data[4];
		
	}set;
	
	set.A = angle;
	
	Strcat(update_yaw,set.data,4);
	
	usart6_send(update_yaw,sizeof(update_yaw));
}


/* 设定当前X */
void Update_X(float posx)
{
	char update_x[8] = "ACTX";
	static union
	{
		float X;
		char data[4];
		
	}set;
	
	set.X = posx;
	
	Strcat(update_x,set.data,4);
	
	usart6_send(update_x,sizeof(update_x));
}


/* 设定当前Y */
void Update_Y(float posy)
{
	char update_y[8] = "ACTY";
	static union
	{
		float Y;
		char data[4];
		
	}set;
	
	set.Y = posy;
	
	Strcat(update_y,set.data,4);
	
	usart6_send(update_y,sizeof(update_y));
}


/* 设定当前XY */
void Update_XY(float posx,float posy)
{
	char update_xy[12] = "ACTD";
	static union
	{
		float XY[2];
		char data[8];
		
	}set;
	
	set.XY[0] = posx;
	set.XY[1] = posy;
	
	Strcat(update_xy,set.data,8);
	
	usart6_send(update_xy,sizeof(update_xy));
}




