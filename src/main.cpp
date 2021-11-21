#include <Arduino.h>

// ============== hardware ================ //
uint8_t relay_io = 4;
uint8_t light_io = A0;;

#define DEBUG 0

uint8_t cha = '0';
uint8_t commdata[10] = {'0'};
uint8_t senddata[10] = {'0'};
uint8_t length_read = 0;


uint8_t device_id = 0x02;
uint8_t turn_cmd[10] = {0x3E, 0xA2, device_id, 0x04, uint8_t(0x3E + 0xA2 + 0x04 + device_id), 0xFF, 0xFF, 0x08, 0x00, 0x06};
uint8_t stop_cmd[5] = {0x3E, 0x81, device_id, 0x00, uint8_t(0x3E + 0x81 + device_id)};
uint8_t tau_cmd[5] = {0x3E, 0xA1, device_id, 0x02, uint8_t(0x3E + 0xA1 + device_id + 0x02)};


void setup()
{
    Serial1.begin(115200);
    Serial.begin(115200);

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

float p = 0.07, i_= 0.06, d = 0.03;


void loop()
{
    if(Serial1.available() > 0)
    {
        delay(2);
        cha = Serial1.read();

        if(cha == 0x3E)   
        {
            commdata[0] = 0x3E;             // 帧头

            delay(2);
            commdata[1] = Serial1.read();   // 命令字
            delay(2);
            commdata[2] = Serial1.read();   // 设备ID
            delay(2);
            commdata[3] = Serial1.read();   // 后续帧长度
            delay(2);
            commdata[4] = Serial1.read();   // 校验和

            if (check_sum(commdata, 5) && commdata[2] == device_id)
            {
                if (commdata[1] == 0xF1)        // 打开继电器
                {
                    digitalWrite(relay_io, HIGH);

                    // feedback
                    Serial1.write(commdata[0]);
                    Serial1.write(commdata[1]);
                    Serial1.write(commdata[2]);
                    Serial1.write(commdata[4]);
                }
                else if (commdata[1] == 0xF2)       // 关闭继电器
                {
                    digitalWrite(relay_io, LOW);

                    // feedback
                    Serial1.write(commdata[0]);
                    Serial1.write(commdata[1]);
                    Serial1.write(commdata[2]);
                    Serial1.write(commdata[4]);
                }
                else if (commdata[1] == 0xF3)       // 设备回零
                {
                    // do something
                    delay(50);
                    for (uint8_t i = 0; i< sizeof(turn_cmd); i++)
                    {
                        Serial1.write(turn_cmd[i]);
                    }

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

                    // delay(2500);

                    for (uint8_t i = 0; i< sizeof(stop_cmd); i++)
                    {
                        Serial1.write(stop_cmd[i]);
                    }

                    delay(20);

                    // feedback
                    if (succcess)
                    {
                        Serial1.write(commdata[0]);
                        Serial1.write(commdata[1]);
                        Serial1.write(commdata[2]);
                        Serial1.write(0x01);
                        Serial1.write(commdata[4]);
                    }
                    else{
                        Serial1.write(commdata[0]);
                        Serial1.write(commdata[1]);
                        Serial1.write(commdata[2]);
                        Serial1.write(0x00);
                        Serial1.write(commdata[4]);
                    }
                    
                }
                else if (commdata[1] == 0xF4)       // 速度模式
                {
                    length_read = commdata[3] + 1;
                    for (int8_t i = 0; i< length_read; i++)
                    {
                        delay(2);
                        commdata[i] = Serial1.read();   // 
                    }

                    if (check_sum(commdata, length_read) && length_read == 5)
                    {
                        int16_t speed_req = commdata[0] | (commdata[1] << 8);
                        int16_t tau_max = commdata[2] | (commdata[3] << 8);

                        int16_t speed_now = 0;
                        int16_t tau_now = 0;
                        int16_t tau_send = 0;

                        float err = 0;
                        float err_last = 0;
                        float sum_pid = 0;

                        // set tau
                        if (tau_max > 1000) tau_max = 1000;
                        if (tau_max < -1000) tau_max = -1000;

                        int j = 0;
                        while (j ++ < 100)
                        {

                            Serial1.write( tau_cmd, 5 );
                            Serial1.write( uint8_t( tau_send & 0xff) );
                            Serial1.write( uint8_t( (tau_send >> 8 ) & 0xff) );
                            Serial1.write( uint8_t( (tau_send & 0xff) + ((tau_send >> 8 ) & 0xff)) );

                            delay(5);

                            if (Serial1.available() > 0)
                            {
                                speed_now = 0;
                                tau_now = 0;
                                for (int i = 0; i< 13; i++)
                                {
                                    // delay(2);
                                    cha = Serial1.read();
                                    // Serial.write(cha);
                                    speed_now += (cha & (i == 8? 0xff: 0x00)) + ((cha & (i == 9? 0xff: 0x00)) << 8 );
                                    tau_now += (cha & (i == 6? 0xff: 0x00)) + ((cha & (i == 7? 0xff: 0x00)) << 8 );
                                    
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
                            // 绘制图像的代码 需要注释掉
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

#endif
                            // if (abs(err) < 10 || abs(tau_now - tau_max) < 5)     // 期望通过打断控制回路来复用，但是最后开环指令会让电机一直转
                            // {
                            //     break;
                            // }

                        }


                        // 停止电机
                        Serial1.write(tau_cmd, 5);
                        tau_send = 0;
                        Serial1.write( uint8_t( tau_send & 0xff) );
                        Serial1.write( uint8_t( (tau_send >> 8 ) & 0xff) );
                        Serial1.write( uint8_t( (tau_send & 0xff) + ((tau_send >> 8 ) & 0xff)) );

                    }
                }

                // 速度和力矩模式直接给电机发指令就可以
                // else if (commdata[1] == 0xF5)       // 力矩模式
                // {
                //     length_read = commdata[3];
                //     for (int8_t i = 0; i< length_read; i++)
                //     {
                //         delay(2);
                //         commdata[i]= Serial1.read();   // 
                //     }

                //     if (check_sum(commdata, length_read) && length_read == 3)
                //     {
                //         int16_t tau = commdata[0] | (commdata[1] << 8);
                //         Serial1.write(tau_cmd, 5);
                //         Serial1.write( uint8_t( tau & 0xff) );
                //         Serial1.write( uint8_t( (tau >> 8 ) & 0xff) );
                //         Serial1.write( uint8_t( (tau & 0xff) + ((tau >> 8 ) & 0xff)) );

                //     }
                // }
                // else if (commdata[1] == 0xF6)       // 速度模式
                // {
                //     length_read = commdata[3];
                //     for (int8_t i = 0; i< length_read; i++)
                //     {
                //         delay(2);
                //         commdata[i]= Serial1.read();   // 
                //     }

                //     if (check_sum(commdata, length_read) && length_read == 3)
                //     {
                //         int16_t tau = commdata[0] | (commdata[1] << 8);
                //         Serial1.write(tau_cmd, 5);
                //         Serial1.write( uint8_t( tau & 0xff) );
                //         Serial1.write( uint8_t( (tau >> 8 ) & 0xff) );
                //         Serial1.write( uint8_t( (tau & 0xff) + ((tau >> 8 ) & 0xff)) );

                //     }
                // }
            
            }

        }
    }// ser.available

#if DEBUG

    // 调试pid的代码，需要注释掉
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
