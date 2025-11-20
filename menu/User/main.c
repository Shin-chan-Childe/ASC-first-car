#include "stm32f10x.h"                  
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "string.h"
#include <math.h>

/*电机测试*/
uint8_t KeyNum;
uint8_t array_loc = 1;  // menu-1:Yes, 2:No, 3:speed; pid-1:kp,2:ki,3:kd
uint8_t menu = 0;       // 菜单(0:主菜单, 1:pid调参)
int32_t mode = 0,mode_v=0;       // 0:不可编辑, 1:可编辑
float pid_value[3] = {0.5, 0.05, 0.2}; // kp,ki,kd 初始值
int16_t PWM;
uint8_t isRunning = 0;  // 运行状态标志：0-停止，1-运行

float Target = 100, Actual, Out;     // 目标值，实际值，输出值
float Kp = 0.5, Ki = 0.05, Kd = 0.2; // 速度PID参数
float Error0, Error1, Error2;        // 速度误差

void process_key_input(void);
void display_current_menu(void);

int main(void)
{
    /*模块初始化*/
    OLED_Init();
    Key_Init();
    Motor_Init();
    Encoder_Init();
    Serial_Init();
    Timer_Init();       // 10ms中断
	
    // 初始化PID值
    pid_value[0] = Kp;
    pid_value[1] = Ki; 
    pid_value[2] = Kd;
	
    while (1)
    {
        
        KeyNum = Key_GetNum();  // 统一处理所有按键事件
		
        process_key_input();        // 处理按键输入
        display_current_menu();     // 显示当前菜单
        OLED_Update();              // 更新显示
        Delay_ms(5);
    }
}

void TIM1_UP_IRQHandler(void)
{
    static float Filtered_Actual = 0;  // 滤波后的速度
    
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        if (isRunning)  // 只有在运行状态下才执行PID控制
        {
            // 更新PID参数（如果被修改）
            Kp = pid_value[0];
            Ki = pid_value[1];
            Kd = pid_value[2];
            
            // 读取原始速度并滤波
            float raw_speed = Encoder_Get();
            Filtered_Actual = 0.6 * Filtered_Actual + 0.4 * raw_speed;
            Actual = Filtered_Actual;
            
            // 更新误差
            Error2 = Error1;
            Error1 = Error0;
            Error0 = Target - Actual;
            
            // 增量式PID
            float deltaOut = Kp * (Error0 - Error1) + Ki * Error0 + Kd * (Error0 - 2*Error1 + Error2);
            Out += deltaOut;
            
            // 输出限幅
            if (Out > 80) Out = 80;
            if (Out < -80) Out = -80;
            
            // 执行控制
//            Motor_SetPWM(Out);
//            Motor_SetPWM_right(Out);
        }
        
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

// 按键处理函数
void process_key_input(void)
{
    if (KeyNum == 11) { // 上移+增加参数
        if (menu == 0) {   // 主菜单
			if (array_loc==3 && mode_v){
				Target++;
			}
			else{
				if (array_loc > 1) {
					array_loc--;
				} else {
					array_loc = 3;  // 修正：主菜单有3个选项
				}
			}

        }
        else if (menu == 1 && mode == 1) { // PID编辑模式
            if (array_loc == 1 || array_loc == 3) { // Kp或Kd
                pid_value[array_loc-1] += 0.1;
                if (pid_value[array_loc-1] < 0) pid_value[array_loc-1] = 0;
            }
            else if (array_loc == 2) { // Ki
                pid_value[array_loc-1] += 0.01;
                if (pid_value[array_loc-1] < 0) pid_value[array_loc-1] = 0;
            }
        }
        else if (menu == 1 && mode == 0) { // PID浏览模式
            if (array_loc > 1) {
                array_loc--;
            } else {
                array_loc = 3;
            }
        }
    }
    else if (KeyNum == 12) { // 下移+减少参数
        if (menu == 0) {   // 主菜单
			if (array_loc==3 && mode_v){
				Target--;
			}
			else{
				if (array_loc < 3) {  // 修正：主菜单有3个选项
					array_loc++;
				} else {
					array_loc = 1;
				}
			}
        }
        else if (menu == 1 && mode == 1) { // PID编辑模式
            if (array_loc == 1 || array_loc == 3) { // Kp或Kd
                pid_value[array_loc-1] -= 0.1;
                if (pid_value[array_loc-1] < 0) pid_value[array_loc-1] = 0;
            }
            else if (array_loc == 2) { // Ki
                pid_value[array_loc-1] -= 0.01;
                if (pid_value[array_loc-1] < 0) pid_value[array_loc-1] = 0;
            }
        }
        else if (menu == 1 && mode == 0) { // PID浏览模式
            if (array_loc < 3) {
                array_loc++;
            } else {
                array_loc = 1;
            }
        }
    }
    else if (KeyNum == 1) { // 确认键
        if (menu == 0) { // 主菜单
            if (array_loc == 1) {  // 选择Yes，发车
                isRunning = 1;
//                // 重置速度PID变量
//                Target = 100;
                Actual = 0;
                Out = 0;
                Error0 = 0;
                Error1 = 0;
                Error2 = 0;
            }
            else if (array_loc == 2) {  // 选择No，进入PID调参
                menu = 1;
                array_loc = 1;
                mode = 0; // 初始为浏览模式
            }
            else if (array_loc == 3) {  // 速度设置
				mode_v=!mode_v;
            }
        }
        else if (menu == 1) { // PID菜单
            mode = !mode; // 切换编辑模式
        }
    }
    else if (KeyNum == 14) { // 返回键
        if (menu != 0) {
            mode = 0;
            menu = 0;
            array_loc = 1;
        }
    }
    else if (KeyNum == 40 || KeyNum == 41) { // 长按上键
        if (menu == 0 && array_loc == 3 && mode_v) {
            Target += 1;  // 增加步长，提高调节效率
        }
        else if (menu == 1 && mode == 1) { // PID编辑模式下的长按
            if (array_loc == 1 || array_loc == 3) {
                pid_value[array_loc-1] += 0.5;
            }
            else if (array_loc == 2) {
                pid_value[array_loc-1] += 0.05;
            }
        }
        if (Target > 200) Target = 200;
    }
    else if (KeyNum == 20 || KeyNum == 21) { // 长按下键
        if (menu == 0 && array_loc == 3 && mode_v) {
            Target -= 1;  // 增加步长
        }
        else if (menu == 1 && mode == 1) { // PID编辑模式下的长按
            if (array_loc == 1 || array_loc == 3) {
                pid_value[array_loc-1] -= 0.5;
                if (pid_value[array_loc-1] < 0) pid_value[array_loc-1] = 0;
            }
            else if (array_loc == 2) {
                pid_value[array_loc-1] -= 0.05;
                if (pid_value[array_loc-1] < 0) pid_value[array_loc-1] = 0;
            }
        }
        if (Target < 0) Target = 0;
    }
    else if (KeyNum == 44) { // 长按暂停
        isRunning = 0;
        Motor_SetPWM(0);
        Motor_SetPWM_right(0);
    }
}

// 菜单显示
void display_current_menu(void)
{
    OLED_Clear(); // 清屏，避免残留
    
    if (menu == 0) { // 主菜单
        // 显示当前按键状态（调试用）
        OLED_Printf(80, 0, OLED_8X16, "K:%d", KeyNum);
        
        OLED_Printf(0, 0, OLED_8X16, "Ready?");
        
        // Yes选项
        if (array_loc == 1) {
            OLED_Printf(0, 16, OLED_8X16, "> Yes");
        } else {
            OLED_Printf(0, 16, OLED_8X16, "  Yes");
        }
        
        // No选项（进入PID调参）
        if (array_loc == 2) {
            OLED_Printf(0, 32, OLED_8X16, "> No");
        } else {
            OLED_Printf(0, 32, OLED_8X16, "  No");
        }
        
        // 速度显示
        if (array_loc == 3) {
            OLED_Printf(0, 48, OLED_8X16, "> Speed: %3d", (int)Target);
			if (mode_v){
				OLED_Printf(112, 48, OLED_8X16, "E");
			}
        } else {
            OLED_Printf(0, 48, OLED_8X16, "  Speed: %3d", (int)Target);
        }
        
        // 运行状态显示
        if (isRunning) {
            OLED_Printf(70, 0, OLED_8X16, "Running");
        } else {
            OLED_Printf(70, 0, OLED_8X16, "Stopped");
        }
    }
    else if (menu == 1) { // PID调参菜单
        OLED_Printf(0, 0, OLED_8X16, "PID");
        
        // 显示编辑模式
        if (mode) {
            OLED_Printf(80, 0, OLED_8X16, "E");
        } 
        
        // 显示Kp
        if (array_loc == 1) {
            OLED_Printf(0, 16, OLED_8X16, "> Kp: %6.2f", pid_value[0]);
        } else {
            OLED_Printf(0, 16, OLED_8X16, "  Kp: %6.2f", pid_value[0]);
        }
        
        // 显示Ki
        if (array_loc == 2) {
            OLED_Printf(0, 32, OLED_8X16, "> Ki: %6.2f", pid_value[1]);
        } else {
            OLED_Printf(0, 32, OLED_8X16, "  Ki: %6.2f", pid_value[1]);
        }
        
        // 显示Kd
        if (array_loc == 3) {
            OLED_Printf(0, 48, OLED_8X16, "> Kd: %6.2f", pid_value[2]);
        } else {
            OLED_Printf(0, 48, OLED_8X16, "  Kd: %6.2f", pid_value[2]);
        }
    }
}