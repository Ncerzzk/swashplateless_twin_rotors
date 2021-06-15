#include "control.h"
#include "debug_utils.h"
#include "math.h"
#include "pid.h"
#include "tim_ext.h"
#include "arm_math.h"
#include "buffer.h"
/*
正方向备忘：
以飞控上的NRF模块的指向作为 前

向前低头，Pitch 为负数 。 平衡状态Pitch = 0
*/

/*
基本调试方式备忘：
电机安装完毕后，将蓝牙插在电机驱动板上
命令：
test_direction 检测编码器正向及旋转正方向，观察电机旋转正方向是否与螺旋桨自紧方向相同，若不同，需调换两条电机线。检测完毕后，驱动板
会自动设置编码器方向符号，发送write命令进行保存。

复位查看编码器方向是否正确写入。

detect 编码器对齐检测，检测完毕后write写入。

使用start 命令进行旋转测试，观察编码器零点是否对齐。

电机旋转测试完成后，将蓝牙插在主控板上。
命令：
cmd  [l or r] [uart cmd to drivers] 可以将命令透传到左电机或者右电机的驱动板上

透传命令:
pos_off_here  该命令会使驱动板记录当前旋翼位置，并记录为旋翼零点，即旋翼相位为0的地方
write 写入

复位电机驱动板，使用主控板命令：
read [l or r] 来读取左电机和右电机的信息，观察旋翼零点是否正确写入

使用
start [l or r] 启动/关闭左右电机，默认运行占空比0.1f,幅度0,相位0
motor [l or r] [duty or amp or phi] 来改变左右电机的运行占空比、cos幅度（加减速幅度）、cos相位（加减速相位）

*/


float Pitch_Out;
Mode Board_Mode = TEST;

float Attitude[3],Ace[3];
float Angle_Speed_Rads[3];

#define Pitch Attitude[1]
#define Roll Attitude[0]
#define Yaw Attitude[2]

#define Roll_Angle_Speed Angle_Speed_Rads[1]   // 左倾为负
#define Pitch_Angle_Speed Angle_Speed_Rads[0]  // 前倾为负
#define Yaw_Angle_Speed Angle_Speed_Rads[2]   // 逆时针为正



Motor Left_Motor={108};
Motor Right_Motor={109};

float Pitch_Target=0.0f;
float Roll_Target=0.0f;
float Pitch_Amp;
float Pitch_F;

float RC_Target_Pitch;
float RC_Target_Roll;
float RC_Target_Yaw;

Wave_Group wg_angle={1,Attitude,Attitude+1,Attitude+2,Attitude};
extern float  RC_Control_Value[];
float Pitch_Angle_Speed_Target=0;
float Roll_Angle_Speed_Target=0;
Wave_Group remote_control_wave={2,RC_Control_Value,RC_Control_Value+1,RC_Control_Value+2,RC_Control_Value+3};
// 右杆上下打 RC_Control_Value[3] 上为正
// 右杆左右打 RC_Control_Value[2] 右为正
//Wave_Group pitch_roll_control_value = {3,&Left_Motor.baseduty,&Pitch_Out,&Pitch_Target,&Pitch};
Wave_Group pitch_roll_control_value = {3,&Left_Motor.baseduty,&Right_Motor.baseduty,&Pitch_Target,&Pitch};
Wave_Group wg_angle_speed={4,&RC_Target_Roll,&RC_Target_Pitch,Angle_Speed_Rads+2,Angle_Speed_Rads};

float Fordward_P=0.0f;
float Fordward_R=0.0f;




PID_S Pitch_PID ={
    .KP=0.1f,//0.23f,//0.4f,//0.23f,//0.23,//0.23,//0.12, //0.03
    .KD=0.005f,//0.01f,//0.03f,//0.01f,//0.015,//0.009,//0.001, //0.001
    .KI=0.2f,//0,//0.2，//0.00, //0.003kl
    .I_TIME=0.005f,
    .i_out_max = 5.0f,
    .I_ERR_LIMIT = 5.0f
};


/*
PID_S Pitch_PID ={
    .KP=0.2,//0.23,//0.12, //0.03
    .KD=0,//0.009,//0.001, //0.001
    .KI=0.0f,//0.2,//0.2，//0.00, //0.003kl
    .I_TIME=0.005f,
    .i_out_max = 5.0f,
    .I_ERR_LIMIT = 5.0f
};
*/

PID_S Roll_PID ={
    .KP=0,//0.15f,//0.23,//0.12,
    .KD=0,//0,//0.009,//0.001,
    .KI=0,//0.28f,
    .I_TIME=0.005f,
    .i_out_max =5.0f,
    .I_ERR_LIMIT = 5.0f
};

PID_S Roll_PID2 ={
    .KP=0.06f,//0.06f,//0.08f,//0.08f,
    .KD=0.006f,//0.006f,//0.008f,//0.008f,
    .KI=0.00f,
    .I_TIME=0.005f,
    .i_out_max =5.0f,
    .I_ERR_LIMIT = 5.0f
};

PID_S Yaw_A_PID={
    .KP=0.5f,//0.5f,//0.5f,
    .KD=0.0000,
    .KI=0.00f,
    .I_TIME=0.005f,
    .i_out_max =5.0f,
    .I_ERR_LIMIT = 5.0f
};

PID_S Yaw_PID={
    .KP=0.05f,//0.5f,//0.5f,
    .KD=0.0000,
    .KI=0.00f,
    .I_TIME=0.005f,
    .i_out_max =5.0f,
    .I_ERR_LIMIT = 5.0f
};

PID_S Roll_A_PID ={
    .KP=0,//1.5f,///1.5f,
    .KD=0,
    .KI=0.0,
    .I_TIME=0.005f,
    .i_out_max = 5.0f,
    .I_ERR_LIMIT = 5.0f
};

PID_S Pitch_A_PID ={
    .KP=0.8f,//1.2f,//1.2f,
    .KD=0,
    .KI=0.00,
    .I_TIME=0.005f,
    .i_out_max = 5.0f,
    .I_ERR_LIMIT = 5.0f
};

void Check_Mode();

float Lead_Filter(float a,float dt,float input,Filter * f){
    float result;
    //float static last_result;

    result= (dt*a*input + f->last_result)/(a*dt+1);
    f->last_result = result;

    return result;
}

Filter filters[3];
static void Get_Angle(){
    int16_t ac_raw[3],gy_raw[3];
    float ac_f[3];


    MPU_Read6500(&MPU9250,ac_raw,gy_raw);
    for(int i=0;i<3;++i){
        ac_f[i]=ac_raw[i];
    }
    
    Gyroraw_to_Angle_Speed(&MPU9250,gy_raw,Angle_Speed_Rads,0);
    IMU_Update(ac_f,Angle_Speed_Rads,Attitude,Ace);

}


float K = 0.35f;
float b = 0.15238f;
Control_Value Pitch_Roll_Control(float pitch_target,float roll_target,PID_S *pitch_pid,PID_S *roll_pid){

    float roll_out;
    Control_Value result;
    Pitch_Out = PID_Control(pitch_pid,pitch_target,Pitch);
    roll_out = -PID_Control(roll_pid,roll_target,Roll);   // 注意，此处roll_out取相反数了,为了符合下面的坐标系定义

    // pitch_out >0 为抬头力矩
    // roll_out >0 为左倾力矩
    /*
        -------------->   Y
        |
        |
        |
        | X
    */


    float length =sqrtf(Pitch_Out*Pitch_Out + roll_out*roll_out);
    float theta = atan2f(Pitch_Out,roll_out);

    theta = theta*180.0f/PI;
    /*
    if(theta<0){
        theta +=360; // -180-180 -> 0-360
    }*/

    #ifdef USE_CURRENT
        if(length>5.0f){
            length=5.0f;
        }
    #else
        if(length>0.35f){
            length = 0.35f;
        }
    #endif

    result.phi = theta;
    result.baseduty=1.2f;
    /*
    result.baseduty = length;
    if(result.baseduty<1.2f){
        result.baseduty = 1.2f;
    }else if(result.baseduty>1.5f){
        result.baseduty = 1.5f;
    }
    */
    result.dutyamp =length*K;//length*(length*K+b);
    result.dutyamp+= Fordward_P * fabsf(Pitch);
    if(result.dutyamp>0.5f){
        result.dutyamp=0.5f;
    }
    #ifndef USE_CURRENT
    result.dutyamp = length * (0.625f*length+0.1875f);
    #endif
    result.phi -= 90; // 将theta 转化为平时习惯的Y朝上的坐标系，使与电机的phi=0 对齐 
    return result;

}


Control_Value Pitch_Roll_Angle_Speed_Control(float pitch_angle_speed_target,
                        float roll_angle_speed_target, PID_S * pitch_A_PID,PID_S * roll_A_PID){
    // 由于驱动板目前不稳定，暂时不使用角速度内环
    float roll_out;
    float length,theta;

    Control_Value result={0};

    Pitch_Out = PID_Control(pitch_A_PID,pitch_angle_speed_target,Pitch_Angle_Speed);
    roll_out = -PID_Control(roll_A_PID,roll_angle_speed_target,Roll_Angle_Speed);  // 注意，此处roll_out取相反数了,为了符合下面的坐标系定义

    // pitch_out >0 为抬头力矩
    // roll_out >0 为左倾力矩
    /*
        -------------->   Y
        |
        |
        |
        | X
    */
    
    length = sqrtf(Pitch_Out*Pitch_Out + roll_out*roll_out);
    theta = atan2f(Pitch_Out,roll_out);
    theta = theta*180.0f/PI;


    if(length>5.0f){
        length=5;
    }

    result.phi = theta;
    result.baseduty=1.2f;

    result.dutyamp =length*K;//length*(length*K+b);
    if(result.dutyamp>0.8f){
        result.dutyamp=0.8f;
    }
    return result;
}


void Torque_Allocation(Control_Value * control_value){
    float yaw_max;
    //control_value->phi -= 90; // 将theta 转化为平时习惯的Y朝上的坐标系，使与电机的phi=0 对齐

    Left_Motor.phi  = 180 + control_value->phi;
    Right_Motor.phi = 180 - control_value->phi;

    float roll_ext_out = PID_Control(&Roll_PID2,RC_Target_Roll,Roll);
    float yaw_ext_out = PID_Control(&Yaw_A_PID,0,Yaw_Angle_Speed);  //左电机逆时针转，右电机顺时针
    //float yaw_ext_out = PID_Control(&Yaw_PID,RC_Target_Yaw,Yaw);  //左电机逆时针转，右电机顺时针
    if(roll_ext_out>0.5f){
        roll_ext_out=0.5f;
    }else if(roll_ext_out<-0.5f){
        roll_ext_out=-0.5f;
    }
    yaw_max = 0.8f-fabsf(roll_ext_out);
    if(yaw_ext_out>yaw_max){
        yaw_ext_out = yaw_max;
    }else if(yaw_ext_out<-yaw_max){
        yaw_ext_out = -yaw_max;
    }
    

    Left_Motor.baseduty = control_value->baseduty+roll_ext_out-yaw_ext_out;
    Right_Motor.baseduty = control_value->baseduty-roll_ext_out+yaw_ext_out; 

    float l_k,r_k;
    l_k = Left_Motor.baseduty/1.2f;
    r_k = Right_Motor.baseduty/1.2f;
    Left_Motor.dutyamp = control_value->dutyamp*l_k;
    Right_Motor.dutyamp = control_value->dutyamp*r_k;

}


extern uint8_t NRF_Cnt;
void RC_Watchdog(){
    static int cnt=0;
    static uint8_t Last_NRF_Cnt =0;
    if(Last_NRF_Cnt==NRF_Cnt){
        cnt++;
    }else{
        cnt=0;
    }

    Last_NRF_Cnt = NRF_Cnt;

    if(cnt==1000){   // 约为2s
        Board_Mode = TEST;
        Start_Motor(&Left_Motor,5,0);
        Start_Motor(&Right_Motor,5,0);
        cnt=0;
    }
}

extern TIM_HandleTypeDef htim6;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    static Control_Value out;
    float left_K,right_K;
    float left_dutyamp_K,right_dutyamp_K;
    static float t;
    float T;

    if(htim->Instance == TIM7){
        Check_Mode();
        Get_Angle();
        send_debug_wave(&wg_angle);
        send_debug_wave(&wg_angle_speed);
        send_debug_wave(&remote_control_wave);

        float duty = RC_Control_Value[0]/1000.0f-0.5f;
        RC_Target_Pitch = -(RC_Control_Value[3]/1000.0f-0.5f)*10.0f;
        RC_Target_Roll = (RC_Control_Value[2]/1000.0f-0.5f)*5.0f+2.5f;
        if(duty<0){
            duty =0.0f;
        }else if(duty>0.5f){
            duty=0.5f;
        }
        /*
        if(Board_Mode == PITCH_COS){
            t+=0.005f;
            
            Pitch_Target = Pitch_Amp*arm_cos_f32(2*PI*Pitch_F*t);
            Roll_Target = Pitch_Amp*arm_cos_f32(2*PI*Pitch_F*t); 
            T=2*PI/Pitch_F;
            if(fabsf(t-T)<=0.005f){
                t=0;
            }
        }else{
            out = Pitch_Roll_Control(0,0,&Pitch_PID,&Roll_PID);
        }
        
        Pitch_Angle_Speed_Target = PID_Control(&Pitch_PID,Pitch_Target,Pitch);
        //Pitch_Angle_Speed_Target -= Pitch * Fordward_P;

        Roll_Angle_Speed_Target = PID_Control(&Roll_PID,Roll_Target,Roll);
        //Roll_Angle_Speed_Target -=Roll*Fordward_R;
        */

        //out = Pitch_Roll_Angle_Speed_Control(Pitch_Angle_Speed_Target,Roll_Angle_Speed_Target,&Pitch_A_PID,&Roll_A_PID);   //由于驱动板不稳定，暂时不适用角速度内环
        out = Pitch_Roll_Control(RC_Target_Pitch,RC_Target_Roll,&Pitch_PID,&Roll_PID); 
        Torque_Allocation(&out);
        left_dutyamp_K= Left_Motor.dutyamp/Left_Motor.baseduty;
        right_dutyamp_K = Right_Motor.dutyamp/Right_Motor.baseduty;
        float old_left,old_right;
        old_left= Left_Motor.baseduty;
        old_right= Right_Motor.baseduty;

        Left_Motor.baseduty+= duty/0.5f*4.5f;
        Right_Motor.baseduty+=duty/0.5f*4.5f;


        if(Left_Motor.baseduty<0.3){
            Left_Motor.baseduty=0.3f;
        }
        if(Right_Motor.baseduty<0.3){
            Right_Motor.baseduty=0.3f;
        }

        left_K = Left_Motor.baseduty/old_left;
        right_K = Right_Motor.baseduty/old_right;
        

        //Right_Motor.baseduty*=1.1f;
        //Left_Motor.dutyamp = Right_Motor.dutyamp=0;
        Left_Motor.dutyamp=Left_Motor.baseduty*left_dutyamp_K;///(left_K);//*left_K);
        Right_Motor.dutyamp=Right_Motor.baseduty*right_dutyamp_K;///(right_K);//*right_K);

        send_debug_wave(&pitch_roll_control_value);
        static int Dangerours_CNT=0;
        /*
        if(Board_Mode!=TEST  && (Pitch<-30 || Pitch>30 || Roll<-40 || Roll>40)){
            Dangerours_CNT++;
            if(Dangerours_CNT>30){
                Board_Mode=TEST;
                Dangerours_CNT=0;
            }
        }else{
            Dangerours_CNT=0;
        }
        */
        switch(Board_Mode){
            case PREPARE:
            Set_VESC_Duty_Amp_Phi(&Left_Motor,0.05,0,0);
            Set_VESC_Duty_Amp_Phi(&Right_Motor,0.05,0,0);
            break;
            case NORMAL:
            case PITCH_COS:
            //Set_VESC_Duty_Amp_Phi(&Left_Motor,Left_Motor.baseduty/6.0f,Left_Motor.dutyamp/6.0f,Left_Motor.phi);
            Set_VESC_Duty_Amp_Phi(&Left_Motor,Left_Motor.baseduty/6.0f,0,0);
            Set_VESC_Duty_Amp_Phi(&Right_Motor,Right_Motor.baseduty/6.0f,Right_Motor.dutyamp/6.0f,Right_Motor.phi); 
            break;
            case TEST:
            break;
        }

        //if(Board_Mode == NORMAL || Board_Mode==PITCH_COS){
            /*
            #ifndef USE_CURRENT
            Set_Motor(&Left_Motor,Left_Motor.baseduty/6.0f,Left_Motor.dutyamp/6.0f,Left_Motor.phi);
            Set_Motor(&Right_Motor,Right_Motor.baseduty/6.0f,Right_Motor.dutyamp/6.0f,Right_Motor.phi);
            #else
            Set_Motor(&Left_Motor,Left_Motor.baseduty,Left_Motor.dutyamp,Left_Motor.phi);
            Set_Motor(&Right_Motor,Right_Motor.baseduty,Right_Motor.dutyamp,Right_Motor.phi);
            #endif
            */
        //}
        //RC_Watchdog();
        //send_wave(pitch_out.dutyamp*100,0,pitch_out.phi,0);
    }
}

int16_t float2Q015(float num){
  int16_t result=0;
  result=num/1.0f*32768;
  return result;
}

float Q0152float(int16_t num){
  float result=0;
  result = num*1.0f/32768.0f;
  return result;
}

void Start_Motor(Motor * motor,uint8_t mode,uint8_t flag){
    uint8_t data[2]={0};
    data[0]=mode;
    data[1]=flag;
    CAN_Send_Message(motor->ID,data,2);
}

void Set_VESC_Duty(Motor *motor,float duty){
    uint8_t data[8]={0};
    int32_t send_index=0;
    buffer_append_int32(data, (int32_t)(duty * 100000.0), &send_index);
    CAN_Send_Message_VESC(motor->ID,0,data,send_index);
}

void Set_VESC_Duty_Amp_Phi(Motor *motor,float duty,float amp,float phi){
	int32_t send_index = 0;
	uint8_t buffer[8];
	buffer_append_float16(buffer, duty,1e4, &send_index);
	buffer_append_float16(buffer, amp,1e4, &send_index);
	buffer_append_float32(buffer, phi, 1e3, &send_index);
    CAN_Send_Message_VESC(motor->ID,46,buffer,send_index);   // 5.03 是46，3.6x 是28
}

void RC_Key_Handle(uint8_t key){
  if(key & 0x01){
    uprintf("LEFT \r\n");

  }
  if(key == 0x02){
      Board_Mode = PREPARE;
    uprintf("UP \r\n");
  }
  if(key &0x04){
    uprintf("RIGHT \r\n");

  }
  if(key &0x08){
    uprintf("DOWN \r\n");
  }
  if(key==0x10){
      //Board_Mode = NORMAL;
      Board_Mode=PITCH_COS;
                Pitch_PID.i=0;
                Roll_PID.i=0;
                Roll_A_PID.i=0;
                Pitch_A_PID.i=0;
                Roll_PID2.i=0;
                Yaw_A_PID.i=0;
     uprintf("OK\r\n");
  }
  if(key==0x20){
      Board_Mode = TEST;
   uprintf("Cancel\r\n");
  }
}

void Check_Mode(){
    static Mode Last_Mode = TEST;

    if(Last_Mode !=Board_Mode){
        switch(Board_Mode){
            case PREPARE:
            break;
            case NORMAL:
            case PITCH_COS:
                Pitch_PID.i=0;
                Roll_PID.i=0;
                Roll_A_PID.i=0;
                Pitch_A_PID.i=0;
                Roll_PID2.i=0;
                Yaw_A_PID.i=0;
                Yaw_PID.i=0;
                RC_Target_Yaw = Yaw;
            break;
            case TEST:
                Set_VESC_Duty_Amp_Phi(&Left_Motor,0.05f,0,0);
                Set_VESC_Duty_Amp_Phi(&Right_Motor,0.05f,0,0); 
                break;
            default:
            break;
        }
    }

    Last_Mode = Board_Mode;
}