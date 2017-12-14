/*********************************************************/
//@demo
//@固件库：超核V2.4
//@author：th
//@2016.11.30
//@for seu2016 摄像头四轮组
/*********************************************************/

#include "isr.h"
#include "MK60DZ10.h"
uint16_t vsync=0;

#include "varieble.h"

uint8_t uart_buff[2]={0,0};

uint8_t uart_buff_num=0;
uint8_t rec_flag = 0;
uint8_t row_counter=0;

extern enum PHOTO photo_state;

void delay();
void fbb_photo();

//中断服务函数，需要采集数据量(行，场)可自行修改
//可以在函数中加入led的控制来检测是否进入中断
//不建议在中断服务函数中执行延时或数据处理/串口收发

 void GPIO_ISR(uint32_t array)
{

    if(array & (1 << 7)) //行中断
    {
      if(H_Cnt%2==1 && H_Cnt<100)
        {
          DMA_EnableRequest(HW_DMA_CH0);
        }
        H_Cnt++; 
        if(((V_Cnt%60)==0))
                {
                  if(PCout(4)==1) PCout(4)=0;
                  else PCout(4)==1;
                }
    }

    if(array & (1 << 6)) //场中断
    {
      H_Cnt = 0;
      if(V_Cnt<20)
      V_Cnt++;
      else
      {     //20场之后开始采集
        if(V_Cnt%2==1)	
	    photo_state=odd_ok;
	else
            photo_state=even_ok;
        
        vsync=1-vsync;                            //奇偶场切换
     // DMA_SetDestAddress(HW_DMA_CH0, vsync?(uint32_t)img1[0]:(uint32_t)img2[0]);
        DMA_SetDestAddress(HW_DMA_CH0, vsync?(uint32_t)filebuff[0]:(uint32_t)filebuff[1]);
        imgadd=vsync?filebuff[1]:filebuff[0];
     // imgadd=vsync?img2[0]:img1[0];
      }
    }
    

}


/*
 void GPIO_ISR(uint32_t array)
{
   
     if(array & (1 << 7)) //行中断
    {
      
       if(V_Cnt>=20)
          {
		//清中断标志必需在最前面，否则会导致进两次中断
		//PORTC_ISFR =0xFFFFFFFF;		
		
		//在特定行开启DMA采集 
		if(H_Cnt>=42&&H_Cnt<120&&V_Cnt>=200)
		{		
			if(row_counter<row_num)
				DMA_EnableRequest(HW_DMA_CH0);//使能通道4硬件DMA请求
			row_counter++;
		}		

		H_Cnt++;
               /* if(((V_Cnt%60)==0))
                {
                  if(PCout(4)==1) PCout(4)=0;
                  else PCout(4)==1;
                }
                */
           //}
    

/*
    if(array & (1 << 6)) //场中断
    {
      //PORTC_ISFR =0xFFFFFFFF;
      H_Cnt = 0;
      if(V_Cnt<20)
        V_Cnt++;
      else if(V_Cnt>=20)
	{
		//第20场开始进行直立控制			
		if(V_Cnt>=200)
		{			
			//200场开始采集图像，从201场开始进行偶场图像的处理
			if(V_Cnt>=201)
			{
                
				if(V_Cnt%2==1)	
					photo_state=odd_ok;
				else
                                {
                                  photo_state=even_ok;
                                       
                                  //fbb_photo(2,0,0);
                                 // FTM2_C0V=2688;
                                  
                                }
			}
		
			//DMA_SetDestAddress(HW_DMA_CH0, filebuff[V_Cnt%2][2]);//设置DMA目的地址，image数组第一个元素
			//奇偶场标志反转
			vsync=1-vsync;
                        DMA_SetDestAddress(HW_DMA_CH0, vsync?(uint32_t)filebuff[0]:(uint32_t)filebuff[1]);
                        imgadd=vsync?filebuff[1]:filebuff[0];
		}
		
		row_counter=0;

	}
    }
    
    
    
    
}
}
*/
