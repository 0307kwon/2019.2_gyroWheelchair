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

// chipset driver include
#include "ioport.h"
#include "pio/pio.h"

// standard c library include
#include <stdio.h>
#include <stdlib.h>
#include <sam4e.h>

// ubinos library include
#include "itf_ubinos/itf/bsp.h"
#include "itf_ubinos/itf/ubinos.h"
#include "itf_ubinos/itf/bsp_fpu.h"
#include "itf_ubinos/itf/ubik_mutex.h"

//new estk driver include
#include "lib_new_estk_api/itf/new_estk_led.h"
#include "lib_new_estk_api/itf/new_estk_glcd.h"
#include "lib_switch/itf/lib_switch.h"
#include "lib_sensor/itf/lib_sensor.h"
#include "lib_EV3_sensor/itf/lib_EV3_sensor.h"
#include "lib_sensorcalib/itf/lib_sensorcalib.h"
#include "lib_motor_driver/itf/lib_motor_driver.h"
#include "lib_new_estk_api/itf/new_estk_ioport_set.h"

// custom library header file include
//#include "../../lib_default/itf/lib_default.h"
#include "../../lib_new/itf/lib_new.h"

//new estk ble driver include
#include "lib_bluetooth/itf/BT_Module_Interface.h"
#include "lib_bluetooth/itf/lib_BT.h"

#define BLE_MODULE_ID_0 0x01
#define BLE_MODULE_ID_1 0x02
#define BLE_MODULE_ID_2 0x03
#define BLE_MODULE_ID_3 0x60

//-----------------기본코드-----------------------------------------
#define FORWARD      1
#define STOP      0
#define BACKWARD   2

#define CLKWISE         1
//#define STOP         0 //frontback과 공동사용
#define COUNT_CLKWISE    2

//print_packet is a 19bytes without command
uint8_t print_packet[DATA_SEND_BUFFER_SIZE] = {0,};

//mainTask Message Queue
msgq_pt BT_user_event_queue;
mutex_pt _g_mutex;
mutex_pt _order_mutex;

///---------------------------------- ---------------------------

#define NoColor   0
#define   Black   1
#define   Blue   2
#define   Green   3
#define Yellow   4
#define   Red      5
#define   White   6
#define   Brown   7


int way;
uint8_t a = STOP; //color
uint8_t b = STOP; //way

uint8_t before_a; //color
uint8_t before_b; //way

//0~255
// 127 => 0 , 0 => -127 , 255 => 128

static void rootfunc();
static void Read_Color();
static void Choose_Way();

int usrmain(int argc, char * argv[]) {
   int r;
   printf("\n\n\n\r");
   printf("==========");
   printf("exe_intr_test(build time: %s%s) \n\r", __TIME__, __DATE__);

   glcd_init();
   ev3_sensor_init(0, COL_COLOR);
   encoder_init();

   mutex_create(&_g_mutex);
   mutex_create(&_order_mutex);

   r = task_create(NULL, rootfunc, NULL, task_getmiddlepriority()+2, 512, "root");
   if(0 != r) {
      logme("fail at BT_USER_task_create\r\n");
   }



   /*
   //message queue create BT_usr_event_queue
   r = msgq_create(&BT_user_event_queue, sizeof(BT_Evt_t), MAIN_MSGQ_MAX_COUNT);
      if(0 != r) {
         logme("fail at msgq_create\r\n");
   }
   */

   ubik_comp_start();
   return 0;
}

static void rootfunc() {
   int r;
   before_a = a;
   before_b = b;

   module_id_st target_BT_ID;

   //set BT_module target ID
   target_BT_ID.module_id[0] = BLE_MODULE_ID_0;
   target_BT_ID.module_id[1] = BLE_MODULE_ID_1;
   target_BT_ID.module_id[2] = BLE_MODULE_ID_2;
   target_BT_ID.module_id[3] = BLE_MODULE_ID_3;
   //msg[0] init
   uint8_t BT_msg[DATA_SEND_BUFFER_SIZE] = {0x01};

   BT_INIT(INIT_ROLE_CENTRAL, target_BT_ID, BT_user_event_queue);

   BT_SCAN_START();

   BT_CONNECT(target_BT_ID);

   task_sleep(1000);

   r = task_create(NULL, Read_Color, NULL, task_getmiddlepriority()+1, 512, "Read_Color");
   if(0 != r) {
      logme("fail at BT_USER_task_create\r\n");
   }

   r = task_create(NULL, Choose_Way, NULL, task_getmiddlepriority()+1, 512, "Choose_Way");
   if(0 != r) {
      logme("fail at BT_USER_task_create\r\n");
   }


   while(1){
   if(before_a != a || before_b != b) {
      mutex_lock(_order_mutex);
         before_a = a;
         before_b = b;
         BT_CUSTOM_SEND(a,b);
      mutex_unlock(_order_mutex);
   }
   task_sleep(300);
   }

}



void BT_CUSTOM_SEND(uint8_t frontBack , uint8_t rotation){
   uint8_t send_packet[DATA_SEND_BUFFER_SIZE] = {0,};

   send_packet[0] = frontBack;
   send_packet[1] = rotation;

   BT_DATA_SEND(INIT_ROLE_CENTRAL, send_packet);

   mutex_lock(_g_mutex);
   glcdGotoChar(0,7);
   glcd_printf("I send (%d,%d) data",send_packet[0],send_packet[1]);
   mutex_unlock(_g_mutex);
}

static void Read_Color(){
   int color;


   for(;;){
      color = ev3_sensor_get(0);
      mutex_lock(_order_mutex);
      switch(color){
         case Black :  a = BACKWARD;
         break;

         case Blue :  a = STOP;
         break;

         case 4 :  a = FORWARD;
         break;

         case Red :  a = FORWARD;
         break;

         case 6 : a = FORWARD;
         break;

         case 7 : a = FORWARD;
         break;
      }
      mutex_unlock(_order_mutex);

      mutex_lock(_g_mutex);
      glcdGotoChar(0,6);
      glcd_printf("I detect %d color",a);
      mutex_unlock(_g_mutex);

      task_sleep(300);
   }

   //color = 1 이면 검은색, 2면 파란색, 5이상 빨간색

}

static void Choose_Way(){
	int before_deg = 0;// 엔코더 이전 값 임시 저장

   int deg = 0; //엔코더값 임시 저장

   int order = 0; //명령
   //0이면 정지
   //1이면 오른쪽 회전
   //2이면 왼쪽 회전


   for(;;)
   {
      deg = encoder_get(0);

      if(deg < before_deg-30){ // 오른쪽 회전
    	  before_deg = deg;
    	  order = 1;
      }else if(deg > before_deg+30){//왼쪽 회전
    	  before_deg = deg;
    	  order = 2;
      }else{
    	  order = 0;
      }


      mutex_lock(_order_mutex);
      b = order;
      mutex_unlock(_order_mutex);


      mutex_lock(_g_mutex);
         glcdGotoChar(0,5);
         glcd_printf("I detect %d DEGREE",b);
         mutex_unlock(_g_mutex);


      task_sleep(300);
   }

}









