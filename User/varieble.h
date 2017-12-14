#ifndef VARIEBLE_H_
#define VARIEBLE_H_
#include "chlib_k.h"



#define row_num     50          //摄像头采集行数
#define BUFFER_SIZE   (512*BLOCK_LEN)	
#define col_num     152	        //摄像头采集列数

#define car_center    79        //车模中心值

#define P_WIDTH       8         //lp1，和lp2指针的之间宽度


#define BW_DELTA      50
#define whiteRoad     110
#define LINE_EDGE 	  2 

#define Direct_Center  3100

#define BLOCK_LEN     20

extern uint16_t H_Cnt;			//记录行中断数
extern uint32_t V_Cnt;			//记录场中断次数
extern uint8_t filebuff[2][BUFFER_SIZE];//文件缓存
extern uint8_t img1[row_num][col_num];
extern uint8_t img2[row_num][col_num];
extern uint8_t *imgadd;

extern uint8_t* Lx;                       //左引导线中心点列号，找不到时为col_num
extern uint8_t* Rx;                       //右引导线中心点列号，找不到时为0
extern uint8_t* L_Start;
extern uint8_t* L_End;
extern uint8_t* R_Start;
extern uint8_t* R_End;

extern const uint8_t offset[ ];



//定义图像状态枚举
enum PHOTO{
	odd_ok=0,
	even_ok=1,
	waitphoto=2
};

void pointer_switch(uint8_t);

#endif 