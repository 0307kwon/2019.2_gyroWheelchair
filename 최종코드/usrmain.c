/*
  Copyright (C) 2009 Sung Ho Park
  Contact: ubinos.org@gmail.com

  This file is part of the exe_helloworld component of the Ubinos.

  GNU General Public License Usage
  This file may be used under the terms of the GNU
  General Public License version 3.0 as published by the Free Software
  Foundation and appearing in the file license_gpl3.txt included in the
  packaging of this file. Please review the following information to
  ensure the GNU General Public License version 3.0 requirements will be
  met: http://www.gnu.org/copyleft/gpl.html.

  GNU Lesser General Public License Usage
  Alternatively, this file may be used under the terms of the GNU Lesser
  General Public License version 2.1 as published by the Free Software
  Foundation and appearing in the file license_lgpl.txt included in the
  packaging of this file. Please review the following information to
  ensure the GNU Lesser General Public License version 2.1 requirements
  will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.

  Commercial Usage
  Alternatively, licensees holding valid commercial licenses may
  use this file in accordance with the commercial license agreement
  provided with the software or, alternatively, in accordance with the
  terms contained in a written agreement between you and rightful owner.
*/

/* -------------------------------------------------------------------------
	Include
 ------------------------------------------------------------------------- */
#include "../ubiconfig.h"

// standard c library include
#include <stdio.h>
#include <stdlib.h>
#include <sam4e.h>

// ubinos library include
#include "itf_ubinos/itf/bsp.h"
#include "itf_ubinos/itf/ubinos.h"
#include "itf_ubinos/itf/bsp_fpu.h"
#include "itf_ubinos/itf/ubik_mutex.h"
// new estk driver include
#include "lib_new_estk_api/itf/new_estk_led.h"
#include "lib_new_estk_api/itf/new_estk_glcd.h"
#include "lib_motor_driver/itf/lib_motor_driver.h"
#include "lib_new_estk_api/itf/new_estk_ioport_set.h"
//motor

//new estk ble driver include
#include "lib_bluetooth/itf/BT_Module_Interface.h"
#include "lib_bluetooth/itf/lib_BT.h"
#include "lib_sensor/itf/lib_sensor.h"
#include "lib_sensorcalib/itf/lib_sensorcalib.h"

//bluetooth module id
#define BLE_MODULE_ID_0	 0X01 //SETTING
#define BLE_MODULE_ID_1	 0X02
#define BLE_MODULE_ID_2	 0X03 //TARGET
#define BLE_MODULE_ID_3	 0X60 //BT_ID


#define DEF_FB_SPD	700

//motor port


#define FB_MOTOR_PORT	0 // frontBack

#define FORWARD		1
#define STOP		0
#define BACKWARD	2


//rot는 엔코더 값을 받아옵니다.

#define R_MOTOR_PORT 0
#define L_MOTOR_PORT 1
#define CHAIR_MOTOR_PORT 2


//Global value define & init
//** printf_packet is a 19 bytes without Command
uint8_t print_packet[DATA_SEND_BUFFER_SIZE] = {0,};
//uint8_t는 블루투스로 받아오는 기본 형태로, 0~255까지 나타낼수 있음


int gyroarr[3];



int fb_state = STOP;
int rot_state = STOP;


//main task message Queue
msgq_pt BT_user_event_queue;

// 차제 바퀴 속도 제어 변수들
int def_speed = 0;
int rot_speed = 0;

int Gain1 = 8;

mutex_pt _speed_mutex; // 차제 속도제어 뮤텍스
//------------------------------


//블루투스를 위한 뮤텍스 or glcd_printf를 위한 뮤텍스
mutex_pt _g_mutex;

//초기 의자의 평행을 맞추고 엔코더 값을 기록합니다.
int myinitialChair = 0;




static void myChairTask();
static void body_task();
void FB_function();
void ROT_function();

/*function Definitions */
int usrmain(int argc, char * argv[]) {
	int r;
	printf("\n\n\n\r");
	printf("==========");
	printf("exe_in.tr_test(build time: %s%s) \n\r", __TIME__, __DATE__);

	glcd_init();
	encoder_init();
	motor_init();
	sensor_init(EV3_IMU_SENSOR,0,0,0);
	modeSelectIMU(1);


	mutex_create(&_g_mutex);
	mutex_create(&_speed_mutex);


	glcdGotoChar(0,4);
	glcd_printf("gain1 : %d ",Gain1);



	//create task : user_task (central or peripheral)
	//r = task_create(NULL, BT_peripheraltask,NULL, task_getmiddlepriority(), 512, "root");


	r = task_create(NULL, BT_peripheraltask, NULL, task_getmiddlepriority()+2, 512, "peripheral");
	if(0 != r) {
			logme("fail at BT_USER_task_create\r\n");
		}

	r = task_create(NULL, body_task, NULL, task_getmiddlepriority()+1, 512, "body");
	if(0 != r) {
		logme("fail at BT_USER_task_create\r\n");
	}



	//message queue create : BT_usr_event_queue
	r = msgq_create(&BT_user_event_queue, sizeof(BT_Evt_t), MAIN_MSGQ_MAX_COUNT);
	if(0 != r) {
		logme("fail at msgq_create\r\n");
	}



	ubik_comp_start();

	return 0;
}

static void body_task(){
	int rot_speed_R = 0;
	int rot_speed_L = 0;

	while(1){
			mutex_lock(_speed_mutex);//-----------------------------
			rot_speed_R = rot_speed;
			rot_speed_L = rot_speed;
			if(def_speed != 0 && rot_speed != 0){ // 움직이면서 회전하면


				if(def_speed < 0){ // 뒤로 이동이면 회전방향을 반대로
					rot_speed_R = -rot_speed_R;
					rot_speed_L = -rot_speed_L;
				}


				if(rot_speed > 0){ // 오른쪽 회전이면 오른쪽 바퀴속도를 느리게
					rot_speed_R = rot_speed_R*0.5; //오른쪽 바퀴 속도 느리게
					rot_speed_L = rot_speed_L*0.5; //왼쪽 바퀴 속도 빠르게
				}else if(rot_speed < 0) {// 왼쪽 회전이면 왼쪽 바퀴속도를 느리게
					rot_speed_R = rot_speed_R*0.5;
					rot_speed_L = rot_speed_L*0.5;
				}


			}

			motor_set(R_MOTOR_PORT, -(def_speed-rot_speed_R));
			motor_set(L_MOTOR_PORT, -(def_speed+rot_speed_L));

			mutex_unlock(_speed_mutex);//----------------------------------

			mutex_lock(_g_mutex);
			glcdGotoChar(0,8);
			glcd_printf("arr : %d       ",gyroarr[0]);
			mutex_unlock(_g_mutex);


			task_sleep(500);
	}
}

static void myChairTask(){


	#define forward_range	150
	#define backward_range 250

	int encoder = 0;



	//시작하기 전에 의자의 위치를 맞추고 시작합니다.
	motor_set(CHAIR_MOTOR_PORT,200);
	getACC(0,gyroarr);
	while(gyroarr[0] >50 || gyroarr[0] <-50){
			getACC(0,gyroarr);
			motor_set(CHAIR_MOTOR_PORT,-gyroarr[0]/2);

	}
	myinitialChair = encoder_get(CHAIR_MOTOR_PORT);

	mutex_lock(_g_mutex);
	glcdGotoChar(0,4);
	glcd_printf("chair calibration done");
	mutex_unlock(_g_mutex);

	for(;;){

		getACC(0,gyroarr);
		if(gyroarr[0] <50 && gyroarr[0] >-50){

		}else{
			encoder = encoder_get(CHAIR_MOTOR_PORT);
			if(encoder <= myinitialChair-backward_range){
				if(-gyroarr[0]/2 >= 0){
					motor_set(CHAIR_MOTOR_PORT,-gyroarr[0]/2);
				}else{
					motor_set(CHAIR_MOTOR_PORT,0);
				}
			}else if(encoder >= myinitialChair+forward_range){
				if(-gyroarr[0]/2 <= 0){
					motor_set(CHAIR_MOTOR_PORT,-gyroarr[0]/2);
				}else{
					motor_set(CHAIR_MOTOR_PORT,0);
				}
			}else{
				motor_set(CHAIR_MOTOR_PORT,-gyroarr[0]/2);
			}
		}
		task_sleep(300);
	}
}




static void BT_peripheraltask(void * arg) {
	int r = 0;
	module_id_st BT_ID;
	BT_Evt_t BT_usr_msgRXBuffer = {0, };

	//set BT_module_ID
	BT_ID.module_id[0] = BLE_MODULE_ID_0;
	BT_ID.module_id[1] = BLE_MODULE_ID_1;
	BT_ID.module_id[2] = BLE_MODULE_ID_2;
	BT_ID.module_id[3] = BLE_MODULE_ID_3;

	BT_INIT(INIT_ROLE_PERIPHERAL,BT_ID,BT_user_event_queue);

	BT_ADV_START();


	r = task_create(NULL, myChairTask, NULL, task_getmiddlepriority()+3, 512, "mychair");
			if(0 != r) {
				logme("fail at BT_USER_task_create\r\n");
			}



	for(;;) {
			r = msgq_receive(BT_user_event_queue, (unsigned char*) &BT_usr_msgRXBuffer);
			if(0 != r) {
				mutex_lock(_g_mutex);
				glcdGotoChar(0,7);
				glcd_printf("error ");
				mutex_unlock(_g_mutex);
			} else {
				switch (BT_usr_msgRXBuffer.status) {
				case BT_EVT_PE_DATA_READ: {
					//memcmp
					for(int i=0; i<20 ; i++)  {
						print_packet[i]=BT_usr_msgRXBuffer.msg[i];
					}

					mutex_lock(_g_mutex);
							glcdGotoChar(0,5);
							glcd_printf("(%d , %d ) ",print_packet[0],print_packet[1]);
							mutex_unlock(_g_mutex);


					if(print_packet[0] != fb_state){
						fb_state = print_packet[0];
						FB_function(); // fb_state에 따라 전후진 속도를 정해주는 함수
						//r = msgq_send(FB_queue, (unsigned char*) &FB_usr_msgRXBuffer);
					}

					if(print_packet[1] != rot_state){
						rot_state = print_packet[1];
						ROT_function();
						//r = msgq_send(ROT_queue, (unsigned char*) &ROT_usr_msgRXBuffer);
					}
				}
				break;
				case BT_EVT_DISCONNECTED:
					break;
				case BT_EVT_CONNECTED:
					break;
				}
			}
			task_sleep(300);
	}
}

void FB_function(){
		switch (fb_state){

			mutex_lock(_speed_mutex);

			case FORWARD: {
				def_speed = DEF_FB_SPD;
				break;
			}
			case BACKWARD: {
				def_speed = -DEF_FB_SPD;
				break;
			}
			default : {
				def_speed = STOP;
				break;
			}

			mutex_unlock(_speed_mutex);
		}

		mutex_lock(_g_mutex);
		glcdGotoChar(0,6);
		glcd_printf("def_speed : %d ",def_speed);
		mutex_unlock(_g_mutex);
}

void ROT_function(){ // Rot_state로 받아온 엔코더 값을 계산해서 회전상수 Rot_speed를 계산합니다.
		mutex_lock(_speed_mutex);
		switch(rot_state){
		case 0 :
			rot_speed = 0;
			break;
		case 1 : // 오른쪽 회전
			rot_speed = 1*(90*Gain1);
			break;
		case 2 : // 왼쪽 회전
			rot_speed = -1*(90*Gain1);
			break;
		default :
			rot_speed = 0;
			break;

		}
		mutex_unlock(_speed_mutex);


		mutex_lock(_g_mutex);
		glcdGotoChar(0,7);
		glcd_printf("rot_speed : %d ",rot_speed);
		mutex_unlock(_g_mutex);
}





