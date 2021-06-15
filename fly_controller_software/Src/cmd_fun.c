#include "cmd_fun.h"
#include "command.h"
#include "uart_ext.h"


typedef struct{
  char * var_name;
  void * value_ptr;
}Var_Edit_Struct;
extern float K,b;
extern float Pitch_Angle_Speed_Target;
extern float Pitch_Amp;
extern float Pitch_F;
extern float Fordward_P,Fordward_R;

Var_Edit_Struct Var_List[]={
  //{"first",&First_Time_Check}
  {"fp",&Fordward_P},
  {"fr",&Fordward_R},
  {"pitch_a",&Pitch_Angle_Speed_Target},
  {"pitch_amp",&Pitch_Amp},
  {"f",&Pitch_F},
  {"k",&K},
  {"b",&b}
};

#define ERROR_PARAGRAM(arg_num,target)      do{ \
                                                  if(arg_num!=target) {\
                                                  uprintf("error arg_num!\r\n"); \
                                                  return ; \
                                                  }\
                                              }while(0)


static void test(int arg_num,char **string_prams,float * arg){
  int i;
  uprintf("this is testkk\r\n");
  uprintf("i get %d args\r\n",arg_num);
  for(i=0;i<(arg_num&0xFF);++i){
    uprintf("one is %f\r\n",arg[i]);
  }
  for(i=0;i<(arg_num>>8);++i){
    uprintf("string_prams is %s\r\n",string_prams[i]);
  }
}

void set_val(int arg_num,char ** s,float * args){
  void * edit_value=0;
  ERROR_PARAGRAM(arg_num,0x0201);
  for(int i=0;i<sizeof(Var_List)/sizeof(Var_Edit_Struct);++i){
    if(compare_string(Var_List[i].var_name,s[0])){
      edit_value=Var_List[i].value_ptr;
      break;
    }
  }
  if(!edit_value){
    return ;
  }
  
  if(compare_string(s[1],"u8")){
    *(uint8_t *)edit_value=(uint8_t)args[0];
    uprintf("ok set %s = %d\r\n",s[0],*(uint8_t *)edit_value);  
  }else if(compare_string(s[1],"int")){
    *(int16_t *)edit_value=(int16_t)args[0];
    uprintf("ok set %s = %d\r\n",s[0],*(int16_t *)edit_value);
  }else if(compare_string(s[1],"f")){
    *(float *)edit_value=args[0];
    uprintf("ok set %s = %f\r\n",s[0],*(float *)edit_value);
  }
}

#include "control.h"

Motor * choose_motor(char c){
  if(c=='l'){
    return &Left_Motor;
  }else if(c=='r'){
    return &Right_Motor;
  }else{
    uprintf("error motor!\r\n");
    return 0;
  }
}
void start(int arg_num,char ** s,float * args){
  ERROR_PARAGRAM(arg_num,0x0100);
  Motor * motor=0;
  motor = choose_motor(s[0][0]);
  if(motor){
    motor->motor_flag = !motor->motor_flag;
    Start_Motor(motor,5,motor->motor_flag);
  }
  /*
  Left_Motor.motor_flag=!Left_Motor.motor_flag;
  Start_Motor(&Left_Motor,Left_Motor.motor_flag);
  */
  uprintf("ok,start!\r\n");
}


/*
  eg:motor l duty 0.4
  eg:motor r phi 180
  eg:motor r amp 0.2
*/

void motor(int arg_num,char **s,float *args){
  Motor * motor=&Left_Motor; 
  ERROR_PARAGRAM(arg_num,0x0103);
  
  motor=choose_motor(s[0][0]);
  /*
  if(s[0][0]=='r'){
    motor = &Right_Motor;
  }
  */
 if(!motor){
   return ;
 }
 //Set_Motor(motor,args[0],args[1],args[2]);

  uprintf("OK,set %s motor duty:%f amp:%f phi:%f \r\n",s[0],args[0],args[1],args[2]);
}

#include "debug_utils.h"
void wave(int arg_num,char **s,float * args){
  if(arg_num == 0x0000){
    Wave_Flag = !Wave_Flag;
  }else if(arg_num==0x0001){
    Wave_ID = args[0];
  }
  uprintf("OK ,wave = %d\r\n",Wave_ID);
}

// extern uint8_t buffer_rx[];
// void bypass_cmd(int arg_num,char **s,float *args){
//   int start=0;
//   int i=0;
//   int len=0;
//   Motor * motor=&Left_Motor;
//   if(arg_num<0x0200){
//     uprintf_polling("error arg_num!\r\n");
//     return ;
//   }

//   motor = choose_motor(s[0][0]);
//   /*
//   if(s[0][0]=='r'){
//     motor = &Right_Motor;
//   }
//   */
//  if(!motor){
//    return ;
//  }

//   int space_cnt=0;
//   while(space_cnt!=2){
//     if(buffer_rx[i]==' ' && buffer_rx[i+1]!=' '){
//       space_cnt++;
//     }
//     i++;
//   }
//   start = i;
//   while(buffer_rx[i]!='\0' && buffer_rx[i]!='\r'){
//     i++;
//   }
//   len = i-start+1;
//   SPI_To_Motor_Uart(motor,(char *)buffer_rx+start,len);
//   uprintf_polling("Len :%d, str:%s\r\n",len,buffer_rx+start);
// }

// void read(int arg_num,char **s,float * args){
//   Motor * motor=&Left_Motor;
//   ERROR_PARAGRAM(arg_num,0x0100);
//   if(s[0][0]=='r'){
//     motor=&Right_Motor;
//   }
//   Read_Motor_info(motor);

//   uprintf("pos:%f   ,    pos_off:%f\r\n",motor->pos,motor->pos_off);
// }

void set_mode(int arg_num,char **s,float *args){
  ERROR_PARAGRAM(arg_num,0x0001);

  uint8_t temp = (uint8_t )args[0];
  if(temp>NORMAL){
    uprintf_polling("error board_mode!\r\n");
    return ;
  }

  Board_Mode = temp;
  uprintf("OK ,board mode = %d\r\n",temp);
}

#include "control.h"
void set_PID(int arg_num,char ** s,float * args){
  PID_S * pid_s=0;
  float * kpkdki=0;
  ERROR_PARAGRAM(arg_num,0x0201);

  if(compare_string(s[0],"pitch")){
    pid_s=&Pitch_PID;
  }else if(compare_string(s[0],"roll2")){
    pid_s=&Roll_PID2;
  }else if(compare_string(s[0],"roll")){
    pid_s=&Roll_PID;
  }else if(compare_string(s[0],"yaw_a")){
    pid_s=&Yaw_A_PID;
  }
  else if(compare_string(s[0],"pitch_a")){
    pid_s=&Pitch_A_PID;
  }else if(compare_string(s[0],"roll_a")){
    pid_s=&Roll_A_PID;
  }

  pid_s->i=0;
  if(compare_string(s[1],"p")){
    kpkdki=&(pid_s->KP);
  }else if(compare_string(s[1],"d")){
    kpkdki=&(pid_s->KD);
  }else if(compare_string(s[1],"i")){
    kpkdki=&(pid_s->KI);
  }
  *kpkdki=args[0];
  uprintf("ok set %s 's %s = %f\r\n",s[0],s[1],args[0]);
}

/*
下面定义用户需要的函数

*/


/*
将要增加的命令与函数写在这里
*/
void command_init(void){
  add_cmd("test",test);  
  add_cmd("start",start);
  add_cmd("set",set_val);

  add_cmd("wave",wave);
  // add_cmd("cmd",bypass_cmd);
  // add_cmd("read",read);
  add_cmd("motor",motor);
  add_cmd("mode",set_mode);

  add_cmd("set_pid",set_PID);
}


