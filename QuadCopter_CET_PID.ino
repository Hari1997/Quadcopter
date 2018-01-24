#include <EnableInterrupt.h>
volatile int x,y,w,z;

volatile int flag=0,flag1=0,flag2=0,flag3=0;
#include <Wire.h>
#include <Servo.h>
Servo m3;
Servo m4;
Servo m2;
Servo m1;



float angle_pitch,angle_roll,angle_yaw;
long gyro_x_cal,gyro_y_cal,gyro_z_cal;
int acc_x,acc_y,acc_z;
int temperature;
int gyro_x,gyro_y,gyro_z;


//Setting PID constants

float pid_p_gain_roll = 1;               
float pid_i_gain_roll = 1;              
float pid_d_gain_roll = 1;                
int pid_max_roll = 400;                    

float pid_p_gain_pitch = 1;  
float pid_i_gain_pitch = 1;  
float pid_d_gain_pitch = 1;  
int pid_max_pitch = 400;          

float pid_p_gain_yaw = 1;                
float pid_i_gain_yaw = 1;               
float pid_d_gain_yaw = 1;                
int pid_max_yaw = 400;                     


//Setting PID variables


float error;
float pid_i_roll,pid_roll_setpoint,pid_output_roll,pid_last_roll_d_error;
float pid_i_pitch,pid_pitch_setpoint,pid_output_pitch,pid_last_pitch_d_error;
float pid_i_yaw,pid_yaw_setpoint,pid_output_yaw,pid_last_yaw_d_error;













int a;






int c = 0,d = 0;
float x1,y1,z1;

 
 int receiver_input_channel_1,receiver_input_channel_2,receiver_input_channel_3,receiver_input_channel_4;
 int last_channel_1 =0,last_channel_2=0,last_channel_3=0,last_channel_4=0;
 
 int current_time;


int esc_1,esc_2,esc_3,esc_4;













//Taking Gyro reading from registers of the MPU at a scale of 500g

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

}


















void calculate_pid()
{

//Roll Calculations

error = angle_roll - pid_roll_setpoint;
pid_i_roll += (pid_i_gain_roll*error);
if(pid_i_roll > pid_max_roll)
pid_i_roll = pid_max_roll;
else if(pid_i_roll < (-1*pid_max_roll))
pid_i_roll = (-1*pid_max_roll);


pid_output_roll = pid_p_gain_roll*error + pid_i_roll + pid_d_gain_roll*(error - pid_last_roll_d_error);
if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

pid_last_roll_d_error = error;


//Pitch calculations


error = angle_pitch - pid_pitch_setpoint;
pid_i_pitch += (pid_i_gain_pitch*error);
if(pid_i_pitch > pid_max_pitch)
pid_i_pitch = pid_max_pitch;
else if(pid_i_pitch < (-1*pid_max_pitch))
pid_i_pitch = (-1*pid_max_pitch);


pid_output_pitch = pid_p_gain_pitch*error + pid_i_pitch + pid_d_gain_pitch*(error - pid_last_pitch_d_error);
if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = (pid_max_pitch * -1);

pid_last_pitch_d_error = error;


//Yaw Calculations


error = angle_yaw - pid_yaw_setpoint;
pid_i_yaw += (pid_i_gain_yaw*error);
if(pid_i_yaw > pid_max_yaw)
pid_i_yaw = pid_max_yaw;
else if(pid_i_yaw < (-1*pid_max_yaw))
pid_i_yaw = (-1*pid_max_yaw);


pid_output_yaw = pid_p_gain_yaw*error + pid_i_yaw + pid_d_gain_yaw*(error - pid_last_yaw_d_error);
if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

pid_last_yaw_d_error = error;



}




void arm()
{
  Serial.println("hello");
  delay(5000);
  if(c==2)
  {
    
  m1.write(170);
 m2.write(170);
  m3.write(170);
 m4.write(170);
  delay(2000);
   m1.write(10);
 m2.write(10);
  m3.write(10);
 m4.write(10);
  delay(2000);
   m1.write(90);
 m2.write(90);
  m3.write(90);
 m4.write(90);
  
 delay(2000);
 
  
  
  c = c+1;
  d = d+1;
  }
}



void heli()
{
  if(digitalRead(A3)==HIGH)
  {
    w=micros();
  }
  else if(w&&flag3==0)
  {
    receiver_input_channel_4=micros()-w;
    w=0;
    flag3=1;
  }
}





void plane()
{
  if(digitalRead(A2)==HIGH)
  {
    z=micros();
  }
  else if(z&&flag2==0)
  {
    receiver_input_channel_3=micros()-z;
    z=0;
    flag2=1;
  }
}





void quad()
{
  if(digitalRead(A1)==HIGH)
  {
    y=micros();
  }
  else if(y&&flag1==0)
  {
    receiver_input_channel_2=micros()-y;
    y=0;
    flag1=1;
  }
}





void drone()
{
  if(digitalRead(A0)==HIGH)
  {
    x=micros();
  }
  else if(x&&flag==0)
  {
    receiver_input_channel_1=micros()-x;
    x=0;
    flag=1;
  }
}




void disarm()
{
  while(1)
  {
    m1.writeMicroseconds(1000);
    m2.writeMicroseconds(1000);
    m3.writeMicroseconds(1000);
    m4.writeMicroseconds(1000);
    c = 0;
  }
}












void setup() {

Serial.begin(9600);
m3.attach(9);
m4.attach(6);
m2.attach(10);
m1.attach(11);
pinMode(A0,INPUT);
pinMode(A1,INPUT);
pinMode(A2,INPUT);
pinMode(A3,INPUT);
pinMode(8,OUTPUT);
digitalWrite(8,HIGH);
enableInterrupt(A0,drone,CHANGE);
enableInterrupt(A1,quad,CHANGE);
enableInterrupt(A2,plane,CHANGE);
enableInterrupt(A3,heli,CHANGE);






//MPU Setting Registers




Wire.beginTransmission(0x68);                                       
  Wire.write(0x6B);                                                    
  Wire.write(0x00);                                                   
  Wire.endTransmission();                                             
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1C);                                                    
  Wire.write(0x10);                                                    
  Wire.endTransmission();                                              
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1B);                                                    
  Wire.write(0x08);                                                    
  Wire.endTransmission();   


Serial.begin(9600);      


//Gyro Calibration
                                            
  for (int cal_int = 0; cal_int < 5000 ; cal_int ++){                  
                                  
    read_mpu_6050_data();                                              
    gyro_x_cal += gyro_x;                                             
    gyro_y_cal += gyro_y;                                              
    gyro_z_cal += gyro_z;                                              
    delay(3);   
    //Serial.print(".");      
  }
  
  //Serial.println("Calibrated");

 gyro_x_cal /= 5000;                                                  
  gyro_y_cal /= 5000;                                                  
  gyro_z_cal /= 5000; 


                                                 
  






}


 























 
void loop() {
read_mpu_6050_data();
 gyro_x -= gyro_x_cal;                                                // Subtract offset
  gyro_y -= gyro_y_cal;                                               
  gyro_z -= gyro_z_cal;  


  angle_pitch += gyro_x *0.00030534;
  angle_roll += gyro_y*0.00030534;
  angle_yaw += gyro_z*0.00030534;

//Serial.println("done");


if(c==0 && receiver_input_channel_3 <1300 && receiver_input_channel_4 <1300)
{
c = c+1;
d = d+1;
}
if(c==1 && receiver_input_channel_3 >1700 && receiver_input_channel_4 >1700)
{
c = c+1;
d = d+1;
arm();
}
//if( receiver_input_channel_1 == 0)
//disarm();






  //Serial.print(receiver_input_channel_1);
  
  flag=0;
 

 
  

//Serial.print("  ");

  //Serial.print(receiver_input_channel_2);
  flag1=0;
 

//Serial.print(" ");

 // Serial.print(receiver_input_channel_3);
  flag2=0;
  
  

//Serial.print(" ");

  //Serial.println(receiver_input_channel_4);
  flag3=0;

//Serial.println();






 pid_i_roll = 0;
 pid_i_pitch = 0;
 pid_i_yaw = 0;                    //Resetting PID variables for a good Start
 pid_last_roll_d_error = 0;
 pid_last_pitch_d_error = 0;
 pid_last_yaw_d_error = 0;



 pid_roll_setpoint = (receiver_input_channel_1 - 1500)/3;
 pid_pitch_setpoint = (receiver_input_channel_2 - 1500)/3;      //Calculate PID set points and mapping it into deg/s
 pid_yaw_setpoint = (receiver_input_channel_4 - 1500)/3;


calculate_pid();

int throttle = receiver_input_channel_3; 

                                                         //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    if(throttle < 1200) throttle = 1200;
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW) ie,M2

if(esc_1 <1200) esc_1 = 1200;
if(esc_1 >1800) esc_1 = 1800;

    
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW) ie,M4

if(esc_2 <1200) esc_2 = 1200;
if(esc_2 >1800) esc_2 = 1800;
    
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW) ie,M3

   if(esc_3 <1200) esc_3 = 1200;
   if(esc_3 >1800) esc_3 = 1800; 
   
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW) ie,M1

    if(esc_4 < 1200) esc_4 = 1200;
    if(esc_4 > 1800) esc_4 = 1800;

Serial.print(esc_1);
Serial.print(" ");
Serial.print(esc_2);
Serial.print(" ");
Serial.print(esc_3);
Serial.print(" ");
Serial.println(esc_4);


//m1.writeMicroseconds(receiver_input_channel_3);
//m2.writeMicroseconds(receiver_input_channel_3);
//m3.writeMicroseconds(receiver_input_channel_3);
//m4.writeMicroseconds(receiver_input_channel_3);

if(c==3)
{

m1.writeMicroseconds(esc_4);
m2.writeMicroseconds(esc_1);
m3.writeMicroseconds(esc_3);
m4.writeMicroseconds(esc_2);

}
/*if(d==3 && receiver_input_channel_1 < 1300 && receiver_input_channel_2 < 1300)
{
disarm();
exit(0);
} */


}
