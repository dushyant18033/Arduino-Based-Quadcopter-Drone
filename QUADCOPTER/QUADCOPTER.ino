#include<Wire.h>
#include<MPU6050.h>

MPU6050 mpu;

/////////////////////////////////
#define pitchOrigin 19.5      ///
#define rollOrigin 0.5        ///
/////////////////////////////////

#define refresh 2500   //2.5ms Cycle duration (400Hz refresh rate)
#define MaxPIDpitchOUT 200
#define MaxPIDrollOUT 200
#define MaxPIDyawOUT 100

#define BR B11110111   //3
#define FR B11101111   //4
#define FL B11011111   //5
#define BL B10111111   //6
#define ALL_HIGH B01111000 //Pulling 3,4,5,6 high 
#define ALL_LOW B10000111  //Pulling 3,4,5,6 low

/////PID CONSTANTS/////
  #define kpP 4
  #define kiP 0.03
  #define kdP 1.5
  
  #define kpR 4
  #define kiR 0.03
  #define kdR 1.5
  
  #define kpY 10
//////////////////////

/*
  Throttle - 10  (ch3)
  Pitch - 9   (ch2)
  Roll - 8   (ch1)
  Yaw - 11    (ch4)
*/

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int throttle=1000, pitch=1500, roll=1500, yaw=1500;   ///////////////set throttle back to 1000 before actual flight/////////////////////////////////////
unsigned long timer_1, timer_2, timer_3, timer_4;       //Variables for rf reciever reading


///////////ACCEL-GYRO STUFF ///////////////////////////////////////
double gyro_x, gyro_y, gyro_z;
double acc_x, acc_y, acc_z;

int temperature;
double gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;

float angle_pitch, angle_roll;
boolean set_gyro_angles;

float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
float yawCompensate=0;
/////////////////////////////////////////////////////////////

//////////////////MOTOR SPEEDS///////////////////////////
int velBR=1000, velFR=1000, velFL=1000, velBL=1000;

///////////////////PID VARIABLES/////////////////////////
double pSetpoint=0.00, rSetpoint=0.00, ySetpoint=0.0;
double p_i_err=0, r_i_err=0; 
double p_l_err=0, r_l_err=0;

/////////////SOME OTHER REQUIRED VARIABLES///////////////
bool c=false;
bool Start=false;
unsigned long int runtime=0;
unsigned long int lastTX=0;

void setup(){
  
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);  // set PCINT1 (digital input 9)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);  // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);  // set PCINT3 (digital input 11)to trigger an interrupt on state change

  ////SET ESC PINS AS OUTPUT
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  ////ESC ARMING SEQUENCE////
  PORTD |= ALL_HIGH;
  delayMicroseconds(1000);  //1ms
  PORTD &= ALL_LOW;
  ///////////////////////////

  setupMPU();  /////SETUP THE SENSOR and CALCULATE OFFSETS
  
  Serial.begin(57600);///////////////////////////////////
}
void loop(){
  
  if(!(c&&Start))       //If MPU does not connect put throttle to 0%
  {                        //Wait for the arming sequence
    c=mpu.testConnection();
     
    PORTD |= ALL_HIGH;
    delayMicroseconds(1000);  //0% throttle
    PORTD &= ALL_LOW;
    delayMicroseconds(1000);
    return;
  }

  runtime=micros();

  readRawData();
  processRawData();
  
//////////////ESC PULSE BEGINS//////////////////
  unsigned long esc_high=micros();
  PORTD |= ALL_HIGH;    //Pull all esc pins HIGH  
////////////////////////////////////////////////

  processRawData2();


/*  Serial.print(angle_pitch_output);
  Serial.print("\t");
  Serial.println(angle_roll_output);
*/
  
  if(throttle<1050)
    throttle=1000;
  else if(throttle>1800)
    throttle=1800;
   
  velBR=throttle;
  velFR=throttle;
  velFL=throttle;
  velBL=throttle;
  
  if((throttle>=1100)&&Start)
  {
    PIDpitch();
    PIDroll();  
    PIDyaw();
  }

/////////////////////////////////////////

  if(velBR>2000)
    velBR=2000;
  else if(velBR<1000)
    velBR=1000;

  if(velFR>2000)
    velFR=2000;
  else if(velFR<1000)
    velFR=1000;
    
  if(velFL>2000)
    velFL=2000;
  else if(velFL<1000)
    velFL=1000;
    
  if(velBL>2000)
    velBL=2000;
  else if(velBL<1000)
    velBL=1000;    

    
 //Serial.println(micros()-esc_high);

 
  while(PORTD >= 8) //While(atleast one of the esc pins is HIGH)
  {
    unsigned long esc_low = micros() - esc_high;
    
    if(velBR<=esc_low)
      PORTD &= BR;
    if(velFR<=esc_low)
      PORTD &= FR;
    if(velFL<=esc_low)
      PORTD &= FL;
    if(velBL<=esc_low)
      PORTD &= BL;
  }  
///////////ESC PULSE ENDS/////////////////////

  if((millis()-lastTX)>1000)
      Start=false;

  Serial.print(velBL);
  Serial.print("\t");
  Serial.print(velFL);
  Serial.print("\t");
  Serial.print(velFR);
  Serial.print("\t");
  Serial.println(velBR);

/*  Serial.print(throttle);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(roll);
  Serial.print("\t");
  Serial.println(yaw);
*/
  //Serial.println(micros()-runtime);//////////////////////////
  while((micros()-runtime)<refresh);
}

ISR(PCINT0_vect){         //READING PWM SIGNALS FROM THE RECIEVER
 
  //Channel 1=========================================
  if(last_channel_1 == 0 && PINB & B00000001 ){         //Input 8 changed from 0 to 1
    last_channel_1 = 1;                                 //Remember current input state
    timer_1 = micros();                                 //Set timer_1 to micros()
  }
  else if(last_channel_1 == 1 && !(PINB & B00000001)){  //Input 8 changed from 1 to 0
    last_channel_1 = 0;                                 //Remember current input state
    roll = micros() - timer_1;      //Channel 1 is micros() - timer_1
  }

  
  //Channel 2=========================================
  if(last_channel_2 == 0 && PINB & B00000010 ){         //Input 9 changed from 0 to 1
    last_channel_2 = 1;                                 //Remember current input state
    timer_2 = micros();                                 //Set timer_2 to micros()
  }
  else if(last_channel_2 == 1 && !(PINB & B00000010)){  //Input 9 changed from 1 to 0
    last_channel_2 = 0;                                 //Remember current input state
    pitch = micros() - timer_2;      //Channel 2 is micros() - timer_2
  }

  
  //Channel 3=========================================
  if(last_channel_3 == 0 && PINB & B00000100 ){         //Input 10 changed from 0 to 1
    last_channel_3 = 1;                                 //Remember current input state
    timer_3 = micros();                                 //Set timer_3 to micros()
  }
  else if(last_channel_3 == 1 && !(PINB & B00000100)){  //Input 10 changed from 1 to 0
    last_channel_3 = 0;                                 //Remember current input state
    throttle = micros() - timer_3;      //Channel 3 is micros() - timer_3
  }
  
  
  //Channel 4=========================================
  if(last_channel_4 == 0 && PINB & B00001000 ){         //Input 11 changed from 0 to 1
    last_channel_4 = 1;                                 //Remember current input state
    timer_4 = micros();                                 //Set timer_4 to micros()
  }
  else if(last_channel_4 == 1 && !(PINB & B00001000)){  //Input 11 changed from 1 to 0
    last_channel_4 = 0;                                 //Remember current input state
    yaw = micros() - timer_4;      //Channel 4 is micros() - timer_4
  }  

//////////ARMING-DISARMING SEQUENCE///////////
  if(yaw<1200&&throttle<1200)
    Start=true;
  else if(yaw>1800&&throttle<1200)
    Start=false;
    
  lastTX=millis();
}

inline void PIDpitch(){
  
  if(pitch>1450 && pitch<1550)
    pSetpoint=0;
  else
    pSetpoint = map(pitch, 1000,2000, -10.00,10.00);
    
  double err = pSetpoint - angle_pitch_output;
  double d_err = err - p_l_err;
  p_i_err += err;
  p_l_err=err;

  double Out = kpP*err + kdP*d_err + kiP*p_i_err;

  if(Out>MaxPIDpitchOUT)
    Out=MaxPIDpitchOUT;
  else if(Out<(-MaxPIDpitchOUT))
    Out=(-MaxPIDpitchOUT);

  /////////////////////////
 // Serial.println(Out);
  ////////////////////////

  velFL-=Out;
  velFR-=Out;
  velBR+=Out;
  velBL+=Out;   
}
inline void PIDroll(){

  if(roll>1450 && roll<1550)
    rSetpoint=0;
  else
    rSetpoint = -map(roll, 1000,2000, -10.00,10.00);
  
  double err = rSetpoint - angle_roll_output;
  double d_err = err - r_l_err;
  r_i_err += err;
  r_l_err = err;

  double Out = kpR*err + kdR*d_err + kiR*r_i_err;
  
  if(Out>MaxPIDrollOUT)
    Out=MaxPIDrollOUT;
  else if(Out<(-MaxPIDrollOUT))
    Out=(-MaxPIDrollOUT);

  ///////////////////////////
  //Serial.print("\t");
  //Serial.println(Out);
  ////////////////////////
  
  velFL-=Out;
  velFR+=Out;
  velBR+=Out;
  velBL-=Out;  
  
}
inline void PIDyaw(){

  if(yaw>1450 && yaw<1550)
    ySetpoint=0;
  else
    ySetpoint = -map(yaw, 1000,2000, -5.00,5.00);
  
  double Out = -kpY*(ySetpoint - gyro_z);
  
  if(Out>MaxPIDyawOUT)
    Out=MaxPIDyawOUT;
  else if(Out<(-MaxPIDyawOUT))
    Out=(-MaxPIDyawOUT);

  ///////////////////////////
/*  Serial.print("\t ROLL PID OUTPUT: ");
  Serial.println(Out); */
  ////////////////////////
  
  velFL-=Out;
  velFR+=Out;
  velBR-=Out;
  velBL+=Out;  
  
}

/*inline float aSin(float a){  //Optimised sine inverse
         
  return a*(1+(0.5*a*a));
}*/

void processRawData(){

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value


  //Yaw Compensation
  //6.661562e-7 = ((1/65.5)/400(Hz) )* (3.142(PI) / 180degr) The Arduino sin function is in radians
  yawCompensate=sin(gyro_z * 6.661562e-7);

}
void processRawData2(){
//long tut=micros();

  //Gyro angle calculations
  //3.81679e-5 = (1/65.5)/400(Hz)
  angle_pitch -= gyro_y * 3.81679e-5;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_x * 3.81679e-5;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

  //Adding yaw compensate  
  angle_pitch += angle_roll * yawCompensate;                //If the IMU has yawed transfer the roll angle to the pitch angle
  angle_roll -= angle_pitch * yawCompensate;                //If the IMU has yawed transfer the pitch angle to the roll angle


  //Accelerometer angle calculations
  
  //57.29578 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = atan2(acc_x,sqrt(acc_z*acc_z + acc_y*acc_y))*57.29578;       //Calculate the pitch angle
  angle_roll_acc = atan2(acc_y,sqrt(acc_z*acc_z + acc_x*acc_x))*57.29578;       //Calculate the roll angle



  //Spirit level 
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= pitchOrigin;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= rollOrigin;                                               //Accelerometer calibration value for roll

//Serial.println(micros()-tut);

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.98 + angle_pitch_acc * 0.02;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.98 + angle_roll_acc * 0.02;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
}
void readRawData(){                                             //Subroutine for reading the raw gyro and accelerometer data
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
void setupMPU(){
  
  Wire.begin();                                                        //Start I2C as master
  Wire.setClock(800000);
  
  delay(2000);  //Timeframe to keep the drone on the take off ground

  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission

  c=mpu.testConnection();
  
  ///////OFFSET CALCULATION BEGINS//////
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++)
  {                                                                    //Run this code 2000 times
    readRawData();                                                     //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(2);                                                          //Delay 2ms to simulate the 400Hz program loop
  }
  gyro_x_cal /= 1000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 1000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 1000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  //////////////ENDS/////////////////
}
