#ifndef __CONTROL_H
#define __CONTROL_H
#include "main.h"
#include "icm20600.h"
#include "IMU_Update.h"
#include "uart_ext.h"
#include "pid.h"
#include "can.h"

typedef struct{
    uint8_t ID;
    uint8_t motor_flag;
    float phi;
    float baseduty;
    float dutyamp;
    float pos;    // 此值不会实时更新
    float pos_off;
}Motor;

typedef struct{
    float baseduty;
    float phi;
    float dutyamp;
}Control_Value;

typedef enum{
    TEST,
    NORMAL,
    PREPARE,
    PITCH_COS
}Mode;

extern Mode Board_Mode;
extern PID_S Pitch_PID,Roll_PID,Pitch_A_PID,Roll_A_PID,Roll_PID2,Yaw_A_PID;
extern Motor Left_Motor,Right_Motor;
void Start_Motor(Motor * motor,uint8_t mode,uint8_t flag);
void Set_VESC_Duty_Amp_Phi(Motor *motor,float duty,float amp,float phi);
void RC_Key_Handle(uint8_t key);
typedef struct{
    float last_result;
}Filter;

#define USE_CURRENT

#ifndef USE_CURRENT
#define Set_Motor Set_Motor_Q015
#else
#define Set_Motor Set_Motor_Q78
#endif

#endif