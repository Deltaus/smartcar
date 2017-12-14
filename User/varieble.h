#ifndef VARIEBLE_H_
#define VARIEBLE_H_
#include "chlib_k.h"



#define row_num     50          //����ͷ�ɼ�����
#define BUFFER_SIZE   (512*BLOCK_LEN)	
#define col_num     152	        //����ͷ�ɼ�����

#define car_center    79        //��ģ����ֵ

#define P_WIDTH       8         //lp1����lp2ָ���֮����


#define BW_DELTA      50
#define whiteRoad     110
#define LINE_EDGE 	  2 

#define Direct_Center  3100

#define BLOCK_LEN     20

extern uint16_t H_Cnt;			//��¼���ж���
extern uint32_t V_Cnt;			//��¼���жϴ���
extern uint8_t filebuff[2][BUFFER_SIZE];//�ļ�����
extern uint8_t img1[row_num][col_num];
extern uint8_t img2[row_num][col_num];
extern uint8_t *imgadd;

extern uint8_t* Lx;                       //�����������ĵ��кţ��Ҳ���ʱΪcol_num
extern uint8_t* Rx;                       //�����������ĵ��кţ��Ҳ���ʱΪ0
extern uint8_t* L_Start;
extern uint8_t* L_End;
extern uint8_t* R_Start;
extern uint8_t* R_End;

extern const uint8_t offset[ ];



//����ͼ��״̬ö��
enum PHOTO{
	odd_ok=0,
	even_ok=1,
	waitphoto=2
};

void pointer_switch(uint8_t);

#endif 