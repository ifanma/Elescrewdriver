#include <Arduino.h>

#include "SoftwareSerial.h"

// ============== hardware ================ //
uint8_t relay_io = 4;
uint8_t light_io = A0;;

#define DEBUG 0

#define ERROR_CMD 0xff
#define CND_INVALID 0
#define CHECKSUM_ERR 1

uint8_t cha = '0';
uint8_t commdata[10] = {'0'};
uint8_t length_read = 0;

uint8_t device_id = 0x02;
uint8_t zerp_spd_cmd[10] = {0x3E, 0xA2, device_id, 0x04, uint8_t(0x3E + 0xA2 + 0x04 + device_id), 0xFF, 0xFF, 0x08, 0x00, 0x06};
uint8_t stop_cmd[5] = {0x3E, 0x81, device_id, 0x00, uint8_t(0x3E + 0x81 + device_id)};
uint8_t start_cmd[5] = {0x3E, 0x88, device_id, 0x00, uint8_t(0x3E + 0x88 + device_id)};
uint8_t tau_cmd[5] = {0x3E, 0xA1, device_id, 0x02, uint8_t(0x3E + 0xA1 + device_id + 0x02)};
uint8_t spd_cmd[5] = {0x3E, 0xA2, device_id, 0x04, uint8_t(0x3E + 0xA2 + device_id + 0x04)};

int32_t max_spd = 6000;
int16_t max_tau = 1000;

SoftwareSerial serial_cmd(8, 9, false);      // rx tx inverseLogic

int print_trig = 0;

void setup()
{
    Serial1.begin(115200);
    Serial.begin(115200);

    serial_cmd.begin(115200);

    while (!Serial1) {
    }

#if DEBUG
    // 需要注释掉，否则必须插上调试串口才可以运行程序
    while (!Serial) {
    }

    Serial1.println("Start working!");
    Serial.println("Start working!");

#endif

    pinMode(relay_io, OUTPUT);
    digitalWrite(relay_io, HIGH);

    pinMode(light_io, INPUT_PULLUP);
}

bool check_sum(uint8_t list[], int len)
{
    uint8_t sum = 0;
    for (int i = 0; i< len - 1; i++)
    {
        sum += list[i];
    }

    if(sum == list[len -1])    //校验正确
    {
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t get_sum(uint8_t list[], int len)
{
    uint8_t sum = 0;
    for (int i = 0; i< len; i++)
    {
        sum += list[i];
    }
    return sum;
}

void send_ack(uint8_t fun, uint8_t ack)
{
    serial_cmd.write(0x3E);
    serial_cmd.write(fun);
    serial_cmd.write(device_id);
    serial_cmd.write(ack);
    serial_cmd.write( uint8_t(0x3E + fun + device_id + ack) );
}

float p = 0.07, i_= 0.06, d = 0.03;

bool controlLoopFlag = 0;
bool controlLoopFlag_last = 0;

int16_t speed_req, tau_max;
int16_t speed_now = 0;
int16_t tau_now = 0;
int16_t tau_send = 0;

float err = 0;
float err_last = 0;
float sum_pid = 0;

uint8_t read_fail = 0;

void loop()
{
    if(serial_cmd.available() > 0)
    {
        delay(2);
        cha = serial_cmd.read();

        if(cha == 0x3E)   
        {
            commdata[0] = 0x3E;             // 帧头

            delay(2);
            commdata[1] = serial_cmd.read();   // 命令字
            delay(2);
            commdata[2] = serial_cmd.read();   // 设备ID
            delay(2);
            commdata[3] = serial_cmd.read();   // 后续帧长度
            delay(2);
            commdata[4] = serial_cmd.read();   // 校验和

            if (check_sum(commdata, 5) && commdata[2] == device_id)
            {
                if (commdata[1] == 0xF1)        // 打开继电器
                {
                    digitalWrite(relay_io, HIGH);
                    controlLoopFlag = 0;

                    // feedback
                    send_ack(commdata[1], 1);
                }
                else if (commdata[1] == 0xF2)       // 关闭继电器
                {
                    digitalWrite(relay_io, LOW);
                    controlLoopFlag = 0;

                    // feedback
                    send_ack(commdata[1], 1);
                }
                else if (commdata[1] == 0xF3)       // 设备回零
                {
                    delay(50);

                    // do something
                    Serial1.write(zerp_spd_cmd, sizeof(zerp_spd_cmd));

                    uint16_t wait = 0;
                    uint16_t succcess = 0;
                    while(wait ++ < 500)
                    {
                        if (digitalRead(light_io) == LOW)
                        {
                            succcess = 1;
                            break;
                        }
                        delay(5);
                    }
                    
                    delay(5);

                    Serial1.write(stop_cmd, sizeof(stop_cmd));

                    delay(20);

                    // feedback
                    if (succcess)
                    {
                        send_ack(commdata[1], 1);
                    }
                    else{
                        send_ack(commdata[1], 0);
                    }

                    controlLoopFlag = 0;
                    
                }
                else if (commdata[1] == 0xF4)       // 速度模式
                {
                    length_read = commdata[3] + 1;
                    for (int8_t i = 0; i< length_read; i++)
                    {
                        delay(2);
                        commdata[i] = serial_cmd.read();   // 
                    }

                    delay(30);

                    if (check_sum(commdata, length_read) && length_read == 5)
                    {
                        speed_req = commdata[0] | (commdata[1] << 8);
                        tau_max = commdata[2] | (commdata[3] << 8);

                        // set tau
                        if (speed_req > max_spd) speed_req = max_spd;
                        if (speed_req < -max_spd) speed_req = -max_spd;

                        if (tau_max > max_tau) tau_max = max_tau;
                        if (tau_max < -max_tau) tau_max = -max_tau;

                        controlLoopFlag = 1;

                        // feedback
                        send_ack(0xF4, 1);
                    }
                    else{
                        send_ack(ERROR_CMD, CHECKSUM_ERR);

                    }
                }

                // 速度和力矩模式直接给电机发指令就可以
                else if (commdata[1] == 0xA1)       // 力矩模式
                {
                    length_read = commdata[3] + 1;
                    for (int8_t i = 0; i< length_read; i++)
                    {
                        delay(2);
                        commdata[i]= serial_cmd.read();   // 
                    }

                    delay(30);

                    if (check_sum(commdata, length_read) && length_read == 3)
                    {
                        int16_t tau = ((int16_t *)commdata)[0];

                        if (tau > max_tau ) tau = max_tau;
                        if (tau < -max_tau ) tau = -max_tau;

                        Serial1.write(tau_cmd, 5);
                        Serial1.write( (uint8_t *)(&tau), 2 );
                        Serial1.write( get_sum( (uint8_t *)(&tau), 2) );
                    
                        controlLoopFlag = 0;

                        send_ack(0xA1, 1);
                    }
                    else{
                        send_ack(ERROR_CMD, CHECKSUM_ERR);
                    }
                }
                else if (commdata[1] == 0xA2)       // 速度模式
                {
                    length_read = commdata[3] + 1;
                    for (int8_t i = 0; i< length_read; i++)
                    {
                        delay(2);
                        commdata[i]= serial_cmd.read();   // 
                    }

                    if (check_sum(commdata, length_read) && length_read == 5)
                    {
                        int32_t spd = ((int32_t *)commdata)[0];

                        if (spd > max_spd * 100 ) spd = max_spd * 100;
                        if (spd < -max_spd * 100 ) spd = -max_spd * 100;

                        Serial1.write(spd_cmd, 5);
                        Serial1.write( (uint8_t *)(&spd), 4 );
                        Serial1.write( get_sum( (uint8_t *)(&spd), 4) ); 

                        controlLoopFlag = 0;

                        send_ack(0xA2, 1);
                    }
                    else{
                        send_ack(ERROR_CMD, CHECKSUM_ERR);
                    }
                }
                else if (commdata[1] == 0x81)       // 停止指令
                {
                    Serial1.write(stop_cmd, sizeof(stop_cmd));
                    delay(5);
                    send_ack(stop_cmd[1], 1);
                    
                    controlLoopFlag = 0;
                }
                else if (commdata[1] == 0x88)       // 开始指令
                {
                    Serial1.write(start_cmd, sizeof(start_cmd));
                    delay(5);
                    send_ack(start_cmd[1], 1);
                    
                    controlLoopFlag = 0;
                }

                else{
                    // 指令失败，不支持的指令
                    send_ack(ERROR_CMD, CND_INVALID);
                }

                // 其他指令
            
            }// sum right

            else{
                // 校验失败（在软件串口中经常出现）
                send_ack(ERROR_CMD, CHECKSUM_ERR);
            }
        }// head = 0x3e
    }// ser.available


    // ================ 控制部分 ===================== //

    // 在进入之前清空全部的缓存
    if (controlLoopFlag == 1 && controlLoopFlag_last == 0)
    {
        while (Serial1.available() > 0)
        {
            cha = Serial1.read();
        }
    }

    if (controlLoopFlag == 1)
    {
        // 发送指令
        Serial1.write( tau_cmd, 5 );
        Serial1.write( uint8_t( tau_send & 0xff) );
        Serial1.write( uint8_t( (tau_send >> 8 ) & 0xff) );
        Serial1.write( get_sum((uint8_t *)(&tau_send), 2) );

        delay(5);       // 需要等待反馈数据得到，这个值非常重要，通过直接影响控制周期从而影响控制效果

        if (Serial1.available() > 0)
        {
            speed_now = 0;
            tau_now = 0;
            read_fail = 0;
            for (int i = 0; i< 13; i++)
            {
                // delay(2);
                cha = Serial1.read();

                // 因为别的命令发出后，电机也会响应，所以缓存区会有没必要的数据，对控制反馈的获取造成影响
                if (i == 0 && cha != 0x3E) i = 0;   // 用来清除串口缓存区，如果第一个值不对就循环回退，直到找到帧头
                if (i == 1 && cha != 0xA1) i = 0;   // 如果命令字不是A1说明不是真的反馈，说明这一帧都是之前的数据，全部去掉
                if (i == 2 && cha != device_id) read_fail = 1;  // 如果前面都对了，但这里不对，那就说明应该是发送出错了，这一帧需要丢掉，下面的同理
                if (i == 3 && cha != 0x07) read_fail = 1;      
                if (i == 4 && cha != uint8_t( 0xE6 + device_id) ) read_fail = 1;        

                if (read_fail == 0)     // 读取没有失败
                {
                    speed_now += (cha & (i == 8? 0xff: 0x00)) + ((cha & (i == 9? 0xff: 0x00)) << 8 );       // 把i==8的数据和i==9的数据按照高低位拼接起来
                    tau_now += (cha & (i == 6? 0xff: 0x00)) + ((cha & (i == 7? 0xff: 0x00)) << 8 );
                }
            }
        }

        // 传统pid
        err = float(speed_req - speed_now);
        sum_pid += err;
        if (sum_pid > 200) sum_pid = 200;
        if (sum_pid < -200) sum_pid = -200;

        tau_send = int16_t(p * err + d * (err - err_last) + i_ * sum_pid);
        err_last = err;

        if (tau_send > abs(tau_max)) tau_send = abs(tau_max);
        if (tau_send < -abs(tau_max)) tau_send = -abs(tau_max);

        // tau_send = 50;        // 开环响应

#if DEBUG
        // 绘制图像的代码
        // if (print_trig ++ > 10)
        // {
            Serial.print("data:");
            Serial.print(speed_req);
            Serial.print(",");
            Serial.print(tau_send);
            Serial.print(",");
            Serial.print(speed_now);
            Serial.print(",");
            Serial.print(tau_now);
            Serial.print(",");
            Serial.println(tau_max);

        //     print_trig = 0;
        // }

#endif
        
    }
    else if (controlLoopFlag == 0 && controlLoopFlag_last == 1){
        // 停止电机
        tau_send = 0;
        Serial1.write(tau_cmd, 5);
        Serial1.write( uint8_t( tau_send & 0xff) );
        Serial1.write( uint8_t( (tau_send >> 8 ) & 0xff) );
        Serial1.write( get_sum((uint8_t *)(&tau_send), 2) );
    }

    controlLoopFlag_last = controlLoopFlag;

#if DEBUG

    // 调试pid的代码
    if (Serial.available())
    {
        cha = Serial.read();
        if (cha == 'q')
        {
            p += 0.01;
        }
        if (cha == 'a')
        {
            p -= 0.01;
        }
        if (cha == 'w')
        {
            i_ += 0.01;
        }
        if (cha == 's')
        {
            i_ -= 0.01;
        }
        if (cha == 'e')
        {
            d += 0.01;
        }
        if (cha == 'd')
        {
            d -= 0.01;
        }
        if (cha == 'z')
        {
            p = 0.01;
        }
        if (cha == 'x')
        {
            i_ = 0.00;
        }
        if (cha == 'c')
        {
            d = 0.00;
        }

        Serial.print("PID:");
        Serial.print(p);
        Serial.print(",");
        Serial.print(i_);
        Serial.print(",");
        Serial.println(d);
        
    }
    
    // Serial.print("light: ");
    // Serial.println(analogRead(light_io));

#endif

    delay(2);

}
