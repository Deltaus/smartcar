/*********************************************************/
//@demo
//@�̼��⣺����V2.4
//@author��th
//@2016.11.30
//@for seu2016 ����ͷ������
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

//�жϷ���������Ҫ�ɼ�������(�У���)�������޸�
//�����ں����м���led�Ŀ���������Ƿ�����ж�
//���������жϷ�������ִ����ʱ�����ݴ���/�����շ�

 void GPIO_ISR(uint32_t array)
{

    if(array & (1 << 7)) //���ж�
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

    if(array & (1 << 6)) //���ж�
    {
      H_Cnt = 0;
      if(V_Cnt<20)
      V_Cnt++;
      else
      {     //20��֮��ʼ�ɼ�
        if(V_Cnt%2==1)	
	    photo_state=odd_ok;
	else
            photo_state=even_ok;
        
        vsync=1-vsync;                            //��ż���л�
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
   
     if(array & (1 << 7)) //���ж�
    {
      
       if(V_Cnt>=20)
          {
		//���жϱ�־��������ǰ�棬����ᵼ�½������ж�
		//PORTC_ISFR =0xFFFFFFFF;		
		
		//���ض��п���DMA�ɼ� 
		if(H_Cnt>=42&&H_Cnt<120&&V_Cnt>=200)
		{		
			if(row_counter<row_num)
				DMA_EnableRequest(HW_DMA_CH0);//ʹ��ͨ��4Ӳ��DMA����
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
    if(array & (1 << 6)) //���ж�
    {
      //PORTC_ISFR =0xFFFFFFFF;
      H_Cnt = 0;
      if(V_Cnt<20)
        V_Cnt++;
      else if(V_Cnt>=20)
	{
		//��20����ʼ����ֱ������			
		if(V_Cnt>=200)
		{			
			//200����ʼ�ɼ�ͼ�񣬴�201����ʼ����ż��ͼ��Ĵ���
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
		
			//DMA_SetDestAddress(HW_DMA_CH0, filebuff[V_Cnt%2][2]);//����DMAĿ�ĵ�ַ��image�����һ��Ԫ��
			//��ż����־��ת
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
