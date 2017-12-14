/*********************************************************/
//@demo
//@�̼��⣺����V2.4
//@author��th
//@2016.11.30
//@for seu2016 ����ͷ������
/*********************************************************/

#include "chlib_k.h"
#include "varieble.h"
#include "img_processing.h"
#include "oled.h"
#include "init.h"
void send(void);


//////////////////////////defines/////////////////////////////////////////////
#define LRF_COR_TH1 4//���м�ļ��
#define LRF_COR_TH2 6//����������
#define LRF_CRO_TH1 0//
#define LRF_CRO_TH2 3//
#define LRF_DIR_LEFT 1
#define LRF_DIR_RIGHT 0
#define LRF_SEGM_TH1 2//Matchʱ�������б�ʲ����ֵ
#define LRF_SEGM_TH2 1.5//Matchʱ����м���б�ʲ����ֵ
#define LRF_PATCH_BOTM_TH1 2//6
#define LRF_PATCH_TOP_TH1 35
#define LRF_PATCH_BOTM_TH2 15
#define LRF_PATCH_TOP_TH2 42
#define LRF_PATCH_LEN_TH1 10//20//10

void LxRxFilter();
struct LRF_Seg
{
  uint8_t start;
  uint8_t end;
  uint8_t rowm1;
  float k;//(end-start)*1.0/rowm1
};

#define GPIO_PIN_MASK            0x1Fu
#define GPIO_PIN(x)              (((1)<<(x & GPIO_PIN_MASK)))
#define lowest                     Direct_Center-850
#define highest                    Direct_Center+750
#define A 140
#define B 18


//////////////////////////////varieble///////////////////////////////////////
int R;
int dd1,dd2,dd3,dd4,dd5,dd6,dd7,dt1,dt2,dt3,kp,kd;
int pulse=0;//ת��
int po;
int lost_lx_rx=0;//����ͼ�� �����߷ֱ����ܺͣ������ж��Ƿ�Ҫ����
int car_center1;//·���滮���õ�������
int xl_l_1,xl_l_2,xl_r_1,xl_r_2;
float photo_center;//�������ͼ�������
int slow;
float road1[50];
int if_cross=0;//ʮ�� ��־λ���������ݱ�Ϊ ��������߱�־λ������ض���
int if_angle=0;//������ߵı�־λ
int clost_l=0,clost_r=0;
int angle_l=0,angle_r=0;
int exit_l=0,exit_r=0;
int enter_cross=0;
int V;
int dE;
int Q=0;//������
int Q1=0,Q2=0,Q3=0;
int count=0;

float error=0;//ͼ��ƫ��
float error1=0;//���� ·���滮������������������ֵ
float photo_center=0;
float last_error=0;
float prev_error=0;
//int temp = Direct_Center;
float E;//ͼ��ƫ�����ֵ

int u=0;
int t=0;
int m_error=0;//���ƫ��
int m_last_error=0;//e[k-1]
int m_prev_error=0;//e[k-2]
int speed=0;//�ٶ�����ֵ
int now_speed=0;//ת�ٷ���

int sp1;
int sp2;
int sp3;
int sp4;

extern enum PHOTO photo_state;
extern float lx[50];
extern float rx[50];
extern int LS,LE,RS,RE;
extern float road[50];
extern int angle_pos_l;
extern int angle_pos_r;
extern float cross_exit_l,cross_exit_r;
//////////////////////////////function/////////////////////////////////
void start1();
void fbb_photo();
void set_fbb_photo();
void fbb_motor();
void add_lx_rx();//�����ұ���
void add_road();//�������ߣ����ߣ�
void search_lost();

void cross();
void find_angle();
void find_exit();
void deal_cross();
void stop1();

void disptube(int data);

int abs(int a);
void setspeed(int num,int volt);

void PIT1_CallBack(void);
///////////////////////////////////////////////////////////////////////////////////////////////////////////

int main()
{
  uint16_t i=0;
  DisableInterrupts;//��ʼ��֮ǰ�ȹص������ж�
  DelayInit();
  init();
  OLED_Init();
  
 /* PIT_QuickInit(HW_PIT_CH1,3000);         //1000us=1ms
  PIT_CallbackInstall(HW_PIT_CH1,PIT1_CallBack);
  PIT_ITDMAConfig(HW_PIT_CH1,kPIT_IT_TOF,ENABLE);
 */
  EnableInterrupts;
             dd1 =2;
             dd2 =2;
             dd3 =3;
             dd4 =3;
             dd5 =4;
             dd6 =4;
             dd7 =5;
  //FTM2_C0V = 2000;
           
  while(1){
     
     searchline_OV7620();//�ұ���

     dispimage();//OLED��ʾ
     //send();
 
     
          if(PBin(22)==0)
          {
             /*sp1 =29;
             sp2 =24;
             sp3 =23;
             sp4 =22;// �� �� ���� ���� 0 �����)   ��ʱ��2.35�׵��ٶȣ� ��2��35���ٶȣ���������Ҫ��
            */
             sp1=75;
             sp2=73;
             sp3=71;
             sp4=69;
          }
          else
          {
             sp1 =79;
             sp2 =77;
             sp3 =75;
             sp4 =73;// �������� 11  �������٣� ȫ����٣�ϣ�� ��2��5
          }
                    
          if(PBin(20)==0)  
          {
            R = 28;
            kp = 9;
            kd = 35;
          }
          else
          {
            R = 28;
            kp = 11;
            kd = 42;
          }
                   
          if(PBin(21)==0)  
            po = 100;
          else
            po = 0;
            
          dt1 = 10;         
          dt2 = 18;          
          dt3 = 28;
          
          if(photo_state!=waitphoto)					
		{											
			for(i=2+row_num*col_num;i<BUFFER_SIZE;i++)
                          filebuff[photo_state][i]=filebuff[1-photo_state][i];
			pointer_switch(photo_state);	
			searchline_OV7620();	//�ұ߽��߳���Ρ� 
                        
                       
                        if(PBin(20)==0) 
                          LxRxFilter();
                        E = abs(error);
                        //start1();
                        add_road();
                        search_lost();
                        //cross();
                        
                        //FTM2_C0V=highest;
                        set_fbb_photo();
                        
                        UART_printf(HW_UART3,"pulse:%d\n",pulse);
                        if(pulse>8)
                          //fbb_motor(650,45,180);
                          //fbb_motor(650,0,0);
                          fbb_motor(85,0,0);
                        else
                          //fbb_motor(250,10,50);
                          //fbb_motor(250,0,0);
                          fbb_motor(100,0,0);
                        if(photo_state == odd_ok)
                        ; //dispimage();	       //����ʹ��OLED��ʾͼ��ĳ������Σ����Բ����
			photo_state=waitphoto;
                       
                    

		}

  }
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void send()//����ͼ����λ��
{
      uint32_t i,j;
      UART_WriteByte(HW_UART3, 0xFF);  //ͼ��ͷ
      for(i=0; i<row_num; i++)
      {
        for(j=0;j<col_num;j++)
        {
          //����ԭͼ��
          if(imgadd[i*col_num+j] == 0xFF) UART_WriteByte(HW_UART3,imgadd[i*col_num+j]-1);
          else UART_WriteByte(HW_UART3,imgadd[i*col_num+j]);
          
          //�����ҵ��ı���
        // if(j==Lx[i]-1 || j==Rx[i]) UART_WriteByte(HW_UART3,0x00);
        // else UART_WriteByte(HW_UART3,0xfe);
         }
       }
}

void disptube(int data)
{
  int number[3];
  number[0] = ((data/10)/10)%10;
  number[1] = (data/10)%10;
  number[2] = data%10;
  for(int no=0;no<3;no++)
  {
    if(no==0)
    {
      PEout(4)=0;
      PEout(5)=1;
      PEout(6)=1;
    }
    else if(no==1)
    {
      PEout(4)=1;
      PEout(5)=0;
      PEout(6)=1;
    }
    else if(no==2)
    {
      PEout(4)=1;
      PEout(5)=1;
      PEout(6)=0;
    }
    else {}
      switch(number[no])
      {
      case 0: {  
                   PEout(0)=0;
                   PEout(1)=0;
                   PEout(2)=0;
                   PEout(3)=0;
                   DelayMs(10);
                }break;
      case 1: {  
                   PEout(0)=1;
                   PEout(1)=0;
                   PEout(2)=0;
                   PEout(3)=0;
                   DelayMs(10);
                }break;
      case 2: {  
                   PEout(0)=0;
                   PEout(1)=1;
                   PEout(2)=0;
                   PEout(3)=0;
                   DelayMs(10);
                }break;
      case 3: {  
                   PEout(0)=1;
                   PEout(1)=1;
                   PEout(2)=0;
                   PEout(3)=0;
                   DelayMs(10);
                }break;
      case 4: {  
                   PEout(0)=0;
                   PEout(1)=0;
                   PEout(2)=1;
                   PEout(3)=0;
                   DelayMs(10);
                }break;
      case 5: {  
                   PEout(0)=1;
                   PEout(1)=0;
                   PEout(2)=1;
                   PEout(3)=0;
                   DelayMs(10);
                }break;
      case 6: {  
                   PEout(0)=0;
                   PEout(1)=1;
                   PEout(2)=1;
                   PEout(3)=0;
                   DelayMs(10);
                }break;
      case 7: {  
                   PEout(0)=1;
                   PEout(1)=1;
                   PEout(2)=1;
                   PEout(3)=0;
                   DelayMs(10);
                }break;
      case 8: {  
                   PEout(0)=0;
                   PEout(1)=0;
                   PEout(2)=0;
                   PEout(3)=1;
                   DelayMs(10);
                }break;
       case 9: {  
                   PEout(0)=1;
                   PEout(1)=0;
                   PEout(2)=0;
                   PEout(3)=1;
                   DelayMs(10);
                }break;    
       
      }
  
  }
  PEout(0)=PEout(1)=PEout(2)=PEout(3)=0;
  PEout(4)=PEout(5)=PEout(6)=1;
  
}



#define NORM(a,b) ((a)>(b)?((a)-(b)):((b)-(a)))
void LRF_Seg_Set(struct LRF_Seg* des,uint8_t st,uint8_t en,uint8_t Dir)
{
  (*des).start = st;
  (*des).end = en;
  (*des).rowm1 = en-st;//row-1
  if(Dir == LRF_DIR_LEFT)
    (*des).k = (Lx[en]-Lx[st])*1.0/(*des).rowm1;
  else
    (*des).k = (Rx[en]-Rx[st])*1.0/(*des).rowm1;
}
uint8_t LRF_IfSegMatch(struct LRF_Seg* src,struct LRF_Seg* des,uint8_t Dir,float* gK)
{
  //desλ�ø���src
  float gapK;
  //printf("src.rowm1:%d\n",(*src).rowm1);
  //printf("dst.rowm1:%d\n",(*des).rowm1);
  
  if(NORM((*des).k,(*src).k)>LRF_SEGM_TH1)
    return 0;
 // printf("k1PASS\n");
  if(Dir == LRF_DIR_LEFT)
    gapK = (Lx[(*des).start]-Lx[(*src).end])*1.0/((*des).start-(*src).end);
  else
    gapK = (Rx[(*des).start]-Rx[(*src).end])*1.0/((*des).start-(*src).end);
 // printf("gK:%d/100\n",(int)(gapK*100));
 // printf("desk:%d/100,srck:%d/100\n",(int)((*des).k*100),(int)((*src).k*100));
 // printf("WTF:%d/100\n",(int)(100*NORM((*des).k+(*src).k,gapK+gapK)));
  if(NORM((*des).k+(*src).k,gapK+gapK)>LRF_SEGM_TH2)
    return 0;
//  printf("k2PASS\n");
  *gK = gapK;
  return 1;
}
struct LRF_Seg SegListL[10];
struct LRF_Seg SegListR[10];

void LxRxFilter()
{
  int16_t temp;
  int16_t i,j;//uint->int
  uint8_t pScanStartL,pScanStartR;
  uint8_t comCount1,comCount2;
  uint8_t SegCountL,SegCountR;//���õ�������Ϊ��������,������ָ��
  uint8_t pMatchSrc;
  uint8_t flg_badimage;
  float gapK;
  pScanStartL = 0;
  pScanStartR = 0;
  comCount1 = comCount2 = 0;
  SegCountL = SegCountR = 0;
  for(i=0;i<row_num-1;i++)
  {
    if(Lx[i]+LRF_CRO_TH1< Lx[i+1])
        comCount1++;
    if(Lx[i]==col_num || NORM(Lx[i],Lx[i+1])>LRF_COR_TH1 || comCount1 > LRF_CRO_TH2)
    {
      //i�ǻ���
      //̫����ȫ�����
      if(i-pScanStartL < LRF_COR_TH2)//��������������
        for(j=pScanStartL;j<=i;j++)
          Lx[j] = col_num;
      else
      {
        LRF_Seg_Set(SegListL+SegCountL,pScanStartL,i-1,LRF_DIR_LEFT);
        SegCountL++;
      }
      pScanStartL = i+1;
      comCount1 = 0;
    }
    
    if(Rx[i] > Rx[i+1]+LRF_CRO_TH1)
        comCount2++;
    if(Rx[i]==0 || NORM(Rx[i],Rx[i+1])>LRF_COR_TH1 || comCount2 > LRF_CRO_TH2)
    {
      if(i-pScanStartR < LRF_COR_TH2)
        for(j=pScanStartR;j<=i;j++)
          Rx[j] = 0;
      else
      {
        LRF_Seg_Set(SegListR+SegCountR,pScanStartR,i-1,LRF_DIR_RIGHT);
        SegCountR++;
      }
      pScanStartR = i+1;
      comCount2 = 0;
    }
  }
  Lx[row_num-1] = Lx[row_num-2];
  Rx[row_num-1] = Rx[row_num-2];
  //̫����ȫ�����
  if(i-pScanStartL < LRF_COR_TH2)//��������������
    for(j=pScanStartL;j<=i;j++)
      Lx[j] = col_num;
  else
  {
    LRF_Seg_Set(SegListL+SegCountL,pScanStartL,i,LRF_DIR_LEFT);
    SegCountL++;
  }
  if(i-pScanStartR < LRF_COR_TH2)
    for(j=pScanStartR;j<=i;j++)
      Rx[j] = 0;
  else
  {
    LRF_Seg_Set(SegListR+SegCountR,pScanStartR,i,LRF_DIR_RIGHT);
    SegCountR++;
  }
  //return;
  /*
  for(i=0;i<SegCountL;i++)
  {
    printf("i:%d,start:%d,end:%d,rowm1:%d,k:%d\n",i,
           ((*(SegListL+i)).start),
           ((*(SegListL+i)).end),
             ((*(SegListL+i)).rowm1),
               ((*(SegListL+i)).k));
  }
  
  return;
  */
  //ƥ��
  //����1���������������������
  pMatchSrc = 0;
 // printf("SegCountL:%d\n",SegCountL);
  for(i=1;i<SegCountL;i++)
  {
      if(LRF_IfSegMatch(SegListL+pMatchSrc,SegListL+i,LRF_DIR_LEFT,&gapK))
      {
    //    printf("MATCH,gapK:%d/100\n",(int)(gapK*100));
        //���ӣ�����pMatchSrc
        for(j=((*(SegListL+pMatchSrc)).end+1);j<((*(SegListL+i)).start);j++)
          Lx[j] = (int16_t)Lx[(*(SegListL+pMatchSrc)).end] + (int16_t)(gapK*(j-(*(SegListL+pMatchSrc)).end));//����int8��Ϊint16
        pMatchSrc = i;
      }
      else
      {
     //   printf("Not Match\n");
        //����ò�ƥ���
        //BUG
        //������ɾ���������������һ����BUG,end�϶�Ҫ+1,start������
        for(j=((*(SegListL+i)).start-1);j<=((*(SegListL+i)).end+1);++j)
          if(j>=0 && j<row_num)
            Lx[j] = col_num;
      }
  }
  
  //����
  //����һ��BUG���벹��ǰ��Ĳ���
  if(SegCountL)
  {
    //�����棬��SegList+0
    //Ҫô�ǱȽϵ��µı�Ե��Ҫô�ǱȽ�����ģ�Ҫô�Ǻܳ���
    flg_badimage = 0;
   // if((*(SegListL+0)).start <= LRF_PATCH_BOTM_TH1 ||
   //    (*(SegListL+0)).start >= LRF_PATCH_TOP_TH1 ||
   //    ((*(SegListL+pMatchSrc)).end-(*(SegListL+0)).start) > LRF_PATCH_LEN_TH1)
    if(((*(SegListL+pMatchSrc)).end-(*(SegListL+0)).start) > LRF_PATCH_LEN_TH1
       || (*(SegListL+0)).start <= LRF_PATCH_BOTM_TH1)
    {
      for(i=((*(SegListL+0)).start-1);i>=0;i--)
      {
        temp = (int16_t)Lx[(*(SegListL+0)).start] - (int16_t)((*(SegListL+0)).k*((*(SegListL+0)).start-i));
        if(temp<0)
          Lx[i] = 0;
        else if(temp >=col_num)
          Lx[i] = col_num;
        else
          Lx[i] = temp;
      }
    }
    else
      flg_badimage = 1;
    //�����棬��SegList+pMatchSrc
    //if((*(SegListL+pMatchSrc)).end <= LRF_PATCH_BOTM_TH2 ||
     //  (*(SegListL+pMatchSrc)).end >= LRF_PATCH_TOP_TH2 ||
     //  ((*(SegListL+pMatchSrc)).end-(*(SegListL+0)).start) > LRF_PATCH_LEN_TH1)
    if(((*(SegListL+pMatchSrc)).end-(*(SegListL+0)).start) > LRF_PATCH_LEN_TH1
       || (*(SegListL+0)).start <= LRF_PATCH_BOTM_TH1)
    {
      for(i=((*(SegListL+pMatchSrc)).end+1);i<row_num;i++)
      {
        temp = (int16_t)Lx[(*(SegListL+pMatchSrc)).end] + (int16_t)((*(SegListL+pMatchSrc)).k*(i-(*(SegListL+pMatchSrc)).end));
        if(temp<0)
          Lx[i] = 0;
        else if(temp >=col_num)
          Lx[i] = col_num;
        else
          Lx[i] = temp;
      }
    }
    else
      flg_badimage = 1;
    
    if(flg_badimage)
      for(i=0;i<row_num;++i)
        Lx[i] = col_num;
  }
  
  //�ұ�
  pMatchSrc = 0;
  for(i=1;i<SegCountR;i++)
  {
      if(LRF_IfSegMatch(SegListR+pMatchSrc,SegListR+i,LRF_DIR_RIGHT,&gapK))
      {
        for(j=((*(SegListR+pMatchSrc)).end+1);j<((*(SegListR+i)).start);j++)
          Rx[j] = (int16_t)Rx[(*(SegListR+pMatchSrc)).end] + (int16_t)(gapK*(j-(*(SegListR+pMatchSrc)).end));
        pMatchSrc = i;
      }
      else
      {
        for(j=((*(SegListR+i)).start-1);j<=((*(SegListR+i)).end+1);++j)
          if(j>=0 && j<row_num)
            Rx[j] = 0;
      }
  }
  
  if(SegCountR)
  {
    flg_badimage = 0;
   // if((*(SegListR+0)).start <= LRF_PATCH_BOTM_TH1 ||
    //   (*(SegListR+0)).start >= LRF_PATCH_TOP_TH1 ||
    //   ((*(SegListR+pMatchSrc)).end-(*(SegListR+0)).start) > LRF_PATCH_LEN_TH1)
    if(((*(SegListR+pMatchSrc)).end-(*(SegListR+0)).start) > LRF_PATCH_LEN_TH1
       || (*(SegListR+0)).start <= LRF_PATCH_BOTM_TH1)
    {
      for(i=((*(SegListR+0)).start-1);i>=0;i--)
      {
        temp = (int16_t)Rx[(*(SegListR+0)).start] - (int16_t)((*(SegListR+0)).k*((*(SegListR+0)).start-i));
        if(temp<0)
          Rx[i] = 0;
        else if(temp >=col_num)
          Rx[i] = col_num;
        else
          Rx[i] = temp;
      }
    }
    else
      flg_badimage = 1;
    //�����棬��SegList+pMatchSrc
    //if((*(SegListR+pMatchSrc)).end <= LRF_PATCH_BOTM_TH2 ||
     //  (*(SegListR+pMatchSrc)).end >= LRF_PATCH_TOP_TH2 ||
      // ((*(SegListR+pMatchSrc)).end-(*(SegListR+0)).start) > LRF_PATCH_LEN_TH1)
    if(((*(SegListR+pMatchSrc)).end-(*(SegListR+0)).start) > LRF_PATCH_LEN_TH1
       || (*(SegListR+0)).start <= LRF_PATCH_BOTM_TH1)
    {
      for(i=((*(SegListR+pMatchSrc)).end+1);i<row_num;i++)
      {
        temp = (int16_t)Rx[(*(SegListR+pMatchSrc)).end] + (int16_t)((*(SegListR+pMatchSrc)).k*(i-(*(SegListR+pMatchSrc)).end));
        if(temp<0)
          Rx[i] = 0;
        else if(temp >=col_num)
          Rx[i] = col_num;
        else
          Rx[i] = temp;
      }
    }
    else
      flg_badimage = 1;
    
    if(flg_badimage)
      for(i=0;i<row_num;++i)
        Rx[i] = 0;
  }   
  //����ƨ��
}


void add_road()//add_foad�� if_cross�Ž���
{
  for(int i = 0;i<50;i++)
  {lx[i]=Lx[i];
  rx[i]=Rx[i];}//��һ�����ݣ���ԭ���鴦������ì�ܣ�д����ȥ
 /****************************************************************/
  int raw_start =0;
  int raw_end =0;
  float delta=0;
  float road1[50];
  int lost_l=0,lost_r=0;
  LS=0;LE=0;RS=0;RE=0;
  for(int i = 2; i < 48; i++)
  if(Lx[i-2]<A && Lx[i-1]<A && Lx[i]>=A && Lx[i+1]>=A)
  {LS = i;break;}
   for(int i = 2; i < 48; i++)
  if(Lx[i-1]>=A && Lx[i]>=A && Lx[i+1]<A && Lx[i+2]<A)
  {LE = i;break;}  
   for(int i = 2; i < 48; i++)
  if(Rx[i-2]>=B && Rx[i-1]>=B && Rx[i]<B && Rx[i+1]<B)
  {RS = i;break;}
   for(int i = 2; i < 48; i++)
  if(Rx[i-1]<B && Rx[i]<B && Rx[i+1]>=B && Rx[i+2]>=B)
  {RE = i;break;}
                           //�ĸ������Ƕ������ߵ�ʼĩ��    �ҵĲ������ǻ������Ż��������н���·�ڵ�ֱ�ǣ��䲻�Ƕ��ߣ�����Ӧ����Ϊֱ���Ƕ���    **********                         
 //�������Բ����ߣ���LX   RX
                   //if_angle��־�Ƿ���������ͻ�䣬��������Ǵ������ʮ�ֳ���
                    //�����⵽ͻ�䣬�Ȳ�ͻ�䴦���ߣ�����ͨ����                  
                    //���û��⵽����ͻ�䣬�������ͨ����    
   if(clost_l>30&&clost_l>clost_r+5)//clost_l>20 && clost_r < 20)//��߶���̫�࣬���߲�Ҫ��ȫ�����߲�
  {
    if(RE < 3 && RS >=10 && RS <= 47)//   �ұ��ұ��϶���
      { 
        delta = (Rx[RS-4-1]-Rx[RS-4-6])/5;
        for(int i = RS-4; i < 50; i++)
          rx[i]=rx[RS-5] + (i-RS+5)*delta;
        for(int i = 0;i < 50; i++)
         lx[i]=rx[i]+94-1.04*i;
      }
  }
  else if(clost_r>30&&clost_r>clost_l+5)//clost_r>20 && clost_l < 20)//�ұ߶�̫��
  {
      if(LE < 3 && LS >=10 && LS <= 47)//�϶���
      { 
        delta = (Lx[LS-4-1]-Lx[LS-4-6])/5;
        for(int i = LS-4; i < 50; i++)
          lx[i]=lx[LS-5] + (i-LS+5)*delta;
        for(int i = 0; i < 50; i++)
          rx[i]=lx[i]-94+1.04*i;
      }
  }
/*************************************************************************************************************/

}


void search_lost()
{
  int Lx1[50],Rx1[50];
  xl_l_1=0;
  xl_l_2=0;
  xl_r_1=0;
  xl_r_2=0;
  lost_lx_rx=0;
  if_cross=0;
  if_angle=0;
  for(int i = 2; i < 48; i++)
  {
    Lx1[i] = (Lx[i-2]+Lx[i-1]+Lx[i+1]+Lx[i+2])/4;
    Rx1[i] = (Rx[i-2]+Rx[i-1]+Rx[i+1]+Rx[i+2])/4; 
  }
  for(int i = 2; i < 48; i++)
  {
    Lx[i] = Lx1[i];
    Rx[i] = Rx1[i];
  }
  clost_l = 0;
  clost_r = 0;
  for(int i = 49; i > 0; i--)//����ܶ���
  {
      if(Lx[i] > A || Rx[i] < B)
      lost_lx_rx += 1;
      if(Lx[i] >A)
        clost_l+=1;
      if(Rx[i] <B)
        clost_r+=1;
  }
}

void set_fbb_photo()
{
  if(E > 25)
  fbb_photo(kp-1,0,0,0,po,100,100);
  else if(E > 17)
  fbb_photo(kp-2,0,0,0,po,100,100);
  else 
  //fbb_photo(13,0,33,10,15,10,0);
  fbb_photo(kp-3,0,0,0,po,100,100);
}


void fbb_photo(int Kp,int Ki,int Kd,int a1,int a2,int a3,int a4)
{
  //��Ϊ���߲�����LXRX����ͼ��һֱ����������LXRXһֱ�仯�����ϲ��ߣ��������ء�
  //���ǰ�����ͷ̧�ߣ���ֻȡ0 - 30 �У�30�к�ë�����
  //���Ƕ�ͼ��ļ��ʹ��������漰50��
  float road1[50];
  float box[6];
  float temp;
  for(int t = 0; t < 50; t++)//ͼ�����ɳ�������ɣ�������Ϊ�õ�����������ͼ��
  road[t] = (rx[t] + lx[t])/2;
//������ֵ�˲�
  
  for(int i = 3;i<47;i++)
  {
    box[0] = road[i-3];
     box[1] = road[i-2];
      box[2] = road[i-1];
       //box[3] = road[i];
        box[3] = road[i+1];
         box[4] = road[i+2]; 
          box[5] = road[i+3]; 
    for(int j = 0;j<6;j++)
      for(int k = j;k<6;k++)
        if(box[j]>box[k])
        {
          temp = box[j];
          box[j] = box[k];
          box[k] = temp;
        }  //�Żأ��ɲ��Բ��Żؽ��
    road[i] = (box[2]+box[3])/2;
  }
    for(int i = 3;i<47;i++)
  {
    box[0] = road[i-3];
     box[1] = road[i-2];
      box[2] = road[i-1];
       //box[3] = road[i];
        box[3] = road[i+1];
         box[4] = road[i+2]; 
          box[5] = road[i+3]; 
    for(int j = 0;j<6;j++)
      for(int k = j;k<6;k++)
       if(box[j]>box[k])
       {
         temp = box[j];
         box[j] = box[k];
         box[k] = temp;
       }  //�Żأ��ɲ��Բ��Żؽ��
    road[i] = (box[2]+box[3])/2;
  }
 //////////////////////////// 
  float photo_center_0,photo_center_10,photo_center_25,photo_center_45,average_center;

  photo_center_0 = (road[R]+road[R+1]+road[R+2]+road[R+3]+road[R+4])/5;
  photo_center_10 = (road[R+5]+road[R+6]+road[R+7]+road[R+8]+road[R+9])/5;
  photo_center_25 = (road[R+10]+road[R+11]+road[R+12]+road[R+13]+road[R+14])/5;/////////
  photo_center_45 = (road[R+15]+road[R+16]+road[R+17]+road[R+18]+road[R+19])/5;
  dE=ABS(error1-last_error);
  average_center=(a1 * photo_center_0 + a2 * photo_center_10 + a3 * photo_center_25 + a4 * photo_center_45)/(a1 + a2 + a3 + a4);//���м�Ȩ��ҲҪ�ģ���ƨ��ͨ��Ҫ�òв��жϣ�Ȼ��״̬��,MARK
  car_center1 = car_center;

/*  if(E < 10)                // ·���滮�����ڵ�����ƫ��滮�������·�������ͺܺã�����·���ã�Ч��Ҳͦ��
  {if(error>0) car_center1 = car_center - 0;
  if(error<0) car_center1 = car_center + 0;}
  else if(E < 15)
  {if(error>0) car_center1 = car_center - dd1;
  if(error<0) car_center1 = car_center + dd1;}
  else if(E < 20)
  {if(error>0) car_center1 = car_center - dd2;
  if(error<0) car_center1 = car_center + dd2;}
   else if(E < 25)
  {if(error>0) car_center1 = car_center - dd3;
  if(error<0) car_center1 = car_center + dd3;}
    else if(E < 30)
  {if(error>0) car_center1 = car_center - dd3;
  if(error<0) car_center1 = car_center + dd3;}
    else if(E < 35)
  {if(error>0) car_center1 = car_center - dd4;
  if(error<0) car_center1 = car_center + dd4;}
      else if(E < 40)
  {if(error>0) car_center1 = car_center - dd4;
  if(error<0) car_center1 = car_center + dd4;}
  else
  {if(error>0) car_center1 = car_center - dd5;
  if(error<0) car_center1 = car_center + dd5;}

 */
 
  last_error = error1;                  //���PID��ͦ�ã��Ĳ����Ϳ�����
  error = average_center-car_center; 
  error1 = average_center-car_center1; 
  temp = Direct_Center - Kp * error1 - Kd * (error1 - last_error); 
  
  uint16_t temp16;
  temp16=(uint16_t)temp;
  UART_printf(HW_UART3,"steer temp:%d\n",temp16);

  temp=temp-260;
 
  FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, Direct_Center);
 
  if(temp<lowest)  //�������
    {
      FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, lowest);
    }
    else if(temp>highest)
    {
      FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, highest);
    }
    else
      FTM_PWM_ChangeDuty(HW_FTM2, HW_FTM_CH0, temp);//����ͦ�ã����ø�
  
}
 
int abs(int a)
{
  if(a>0)
    return a;
  else  
    return -a;
}

void start1()
{
  if(clost_l<3&&clost_r<3)
  {
    for(int i=0;i<50;i++)
    {count=0;
    for(int j = Rx[i];j<Lx[i];j++)
      if(imgadd[i*col_num+j]<whiteRoad)
        count+=1;
    
    if(count > 60-0.7*i)
    {
      stop1();
      Q=1;
      break;
    }
    else
      Q=0;
    }
  }
}

void cross()
{
//�������¶��嶪�ߣ�ԭ����LxRx�У���������ĸ���һ�㣬̫����Ե�ĸ��ſ���ȥ��������һ�߲������������߶��϶�����
  find_angle();//����ط���Ҫ��һЩ��ǿ�жϣ�б��ʱһ��Ҫ��׼����ʮ�֣�ֱ��ʱ�����нǣ����ж�
  if(angle_l||angle_r)
  {
          enter_cross=1;
     //     V = V_cnt;//���³��������˵���ɳ����Ҳ������ڣ�������ȥ 
          
  }
  else
    enter_cross = 0;

  if(enter_cross)/////////////////////////////
  {
  //�ҳ��ڣ������Ǹ���һ������״���������ʱ�����Ǹ���״���л�̫�࣬�����ڽ���ʮ������
        //�������дһ�����������ʮ�֣���ô��
    find_exit();
  deal_cross();
   enter_cross=0;
   enter_cross=0;
  }

}

void find_exit()
{
  cross_exit_l=0;
  cross_exit_r=0;//���ҳ��ڱ߽�����0���ҵ��Ͳ���0
  //////////////////////////////////////////////////////�Ұ��ֵĳ���û���
  float xl_up,xl_dn,xr_up,xr_dn;
//  if(angle_l)
  for(int i = 47;i > angle_pos_l;i--)
  {
    if(i>1)
    {
      xl_up = abs(Lx[i+2]-Lx[i]);
      xl_dn = abs(Lx[i]-Lx[i-2]);
    }
    if(xl_dn > xl_up * 3)
    {
      exit_l = 1;
      cross_exit_l = i;
      break;
    }
    else
    {
      exit_l = 0;
      cross_exit_l = 0;
      continue;
    }
  }
    for(int i = 47;i > angle_pos_r;i--)
  {
    if(i>1)
    {
      xr_up = abs(Lx[i+2]-Lx[i]);
      xr_dn = abs(Lx[i]-Lx[i-2]);
    }
    if(xr_dn > xr_up * 3)
    {
      exit_r = 1;
      cross_exit_r = i;
      break;
    }
    else
    {
      exit_r = 0;
      cross_exit_r = 0;
      continue;
    }
  }
}

void deal_cross()
{
  float delta;
 //�����Ƕ�Ҫ���ģ�����Ҳ�����
  if(angle_r && angle_pos_r!=0)
    {
      delta = rx[angle_pos_r]-rx[angle_pos_r-5]; //�����������˾ͻ��
      delta = delta/5; //////////////////////////////////////////////////////////////////                                
      for(int i = angle_pos_r; i<50; i++)
      rx[i] = rx[angle_pos_r-1] + delta * (i-angle_pos_r+1);
    }
  if(angle_l && angle_pos_l!=0)
    {
      delta = lx[angle_pos_l]-lx[angle_pos_l-5];//�����������˾ͻ��
      delta = delta/5;                                 
      for(int i = angle_pos_l; i<50; i++)
      lx[i] = lx[angle_pos_l-1] + delta * (i-angle_pos_l+1);
    }//���ҽǵĲ���
  
  //if(exit_l || exit_r)// ���ҳ��ڱ߽綼����0�ˣ�˵���ҵ��˳��ڣ��������ҵ�������
    ///////////////////////////////////////////
  
    if(exit_l && exit_r)
    {
     delta = (lx[(int)cross_exit_l+2]-lx[(int)cross_exit_l])/2;
      for(int i = cross_exit_l-1; i > 0; i--)
        lx[i] = lx[(int)cross_exit_l] - delta * (cross_exit_l - i);
     delta = (rx[(int)cross_exit_r+2]-rx[(int)cross_exit_r])/2;
      for(int i = cross_exit_r-1; i > 0; i--)
        rx[i] = rx[(int)cross_exit_r] - delta * (cross_exit_r - i);
    }
    else if(exit_l&&!exit_r)
    {delta = (lx[(int)cross_exit_l+2]-lx[(int)cross_exit_l])/2;
      for(int i = cross_exit_l-1; i > 0; i--)
        lx[i] = lx[(int)cross_exit_l] - delta * (cross_exit_l - i);
      for(int i = 0;i<50;i++)
        rx[i] = lx[i] - 94 +1.04*i;
    }
    else if(!exit_l&&exit_r)
    {delta = (rx[(int)cross_exit_r+2]-rx[(int)cross_exit_r])/2;
      for(int i = cross_exit_r-1; i > 0; i--)
        rx[i] = rx[(int)cross_exit_r] - delta * (cross_exit_r - i);
      for(int i = 0;i<50;i++)
        lx[i] = rx[i] + 94 -1.04*i;
    }
    ;/////////////////////////////////////////����Ҳ�����ô�죨��ֻ�н��ڵ����ݰ����ж��ˣ�
      //���Ե�����ͷ������Ȼ���ҵ�
}

void find_angle()
{  //���ﲻ���ټ򵥵���һ���˻�С�����ж��ˣ���Ҫ��ǿ�жϣ����ǻ����Ȳ���
///////////////////////////////////////////////////////////
 for(int i = 5;i<45;i++)
  { 
    xl_r_1=Rx[i]-Rx[i-5];
    xl_r_2=Rx[i+5]-Rx[i];
    if(xl_r_1 * xl_r_2 < 0)
    {
      angle_r=1;
      angle_pos_r=i;
      break;
    }
    else
    {
      angle_r=0;
      angle_pos_r=0;
      continue;
    }
  }
   for(int i = 5;i<45;i++)
  { 
    xl_l_1=Lx[i]-Lx[i-5];
    xl_l_2=Lx[i+5]-Lx[i];
    if(xl_l_1 * xl_l_2 < 0)
    {
      angle_l=1;
      angle_pos_l=i;
      break;
    }
    else
    {
      angle_l=0;
      angle_pos_l=0;
      continue; 
    }
  }
}

void setspeed(int num,int volt)
{
  int pctg=volt*100;
   UART_printf(HW_UART3,"Volt:%d\n",pctg);
  switch(num)
  {
    case 1: FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4, pctg);break;
    case 2: FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH7, pctg);break;
  }
}


void stop1()
{
  speed = 0;
  now_speed = pulse;  
  m_prev_error = m_last_error;
  m_last_error = m_error;
  m_error = speed - now_speed;

  u += 300 * (m_error - m_last_error) + 10 * m_error + 0 * (m_error - 2 * m_last_error + m_prev_error); //10 times
  t = u/100;
          if(t >= 0)                                                                               //���PID ͦ��
          {
              if(t<100)setspeed(1,t);
              else setspeed(1,0);
              setspeed(2,0);
          }
          else
          {
              if(t>-100)setspeed(2,-t);
              else setspeed(2,0);
              setspeed(1,0);
          }
}
 

void fbb_motor(int Kp,int Ki,int Kd)
{
 int enable;
 if(E < dt1)             //��������E�ı��ٶȣ����ڿ�����״̬����mark            
    speed = sp1;
  else if(E < dt2)
    speed = sp2;
  else if(E < dt3)
    speed = sp3;
  else
    speed = sp4;
  //speed = 16;
  now_speed = pulse;  
  m_prev_error = m_last_error;
  m_last_error = m_error;
  m_error = speed - now_speed;
  if(u<9000 || u>-9000)
     u += Kp * (m_error - m_last_error) + Ki * m_error + Kd * (m_error - 2 * m_last_error + m_prev_error); //10 times
  else if(((Kp * (m_error - m_last_error) + Ki * m_error + Kd * (m_error - 2 * m_last_error + m_prev_error)) < 0 && u>9000)||((Kp * (m_error - m_last_error) + Ki * m_error + Kd * (m_error - 2 * m_last_error + m_prev_error)) > 0 && u<-9000))
     u += Kp * (m_error - m_last_error) + Ki * m_error + Kd * (m_error - 2 * m_last_error + m_prev_error);
  else
     u =0;
    
  t = u/100;
  
  uint16_t t16;
  t16=(uint16_t)t;
  UART_printf(HW_UART3,"motor t:%d\n",t16);

 if(t >= 0)                                                                               //���PID ͦ��
  {
      if(t<30)setspeed(1,t);
      else setspeed(1,30);
      setspeed(2,0);
  }
 else
  {
      if(t>-30)setspeed(2,-t);
      else setspeed(2,30);
      setspeed(1,0);
  } 
}                                                     //0.88 m/s = 70+-5 pulses    13r/s = 2.25m/s    14.45r/s = 2.5m/s

/*
void PIT1_CallBack(void)
{
  int16_t value,temp;
  uint8_t dir;
  
  FTM_QD_GetData(HW_FTM1,&value,&dir);
  
  uint16_t v16;
  v16=(uint16_t)value;
  UART_printf(HW_UART3,"speed value:%d\n",v16);
  
   if(value>=0&&value<=100)
   {
      pulse=(int)value;
      pulse=pulse&0x0000FFFF;
   }
      
   else 
      {
        /*temp=value-65535;
        pulse=(int)temp;
        pulse=pulse&0x000000FF;
        pulse=pulse-65535;
        */
        /*pulse=(int)value;
        pulse=pulse&0x000000FF;
        pulse=-pulse;
        */
    /*    temp=65535-value;
        pulse=(int)temp;
        pulse=pulse&0x0000FFFF;
        pulse=-pulse;
      }
  
  FTM_SetMoudleCounter(HW_FTM1,0);
}
*/