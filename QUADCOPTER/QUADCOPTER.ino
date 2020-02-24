#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define refresh 4000   //2.5ms Cycle duration (400Hz refresh rate)
#define MaxPIDpitchOUT 200
#define MaxPIDrollOUT 200
#define MaxPIDyawOUT 100

#define BR B11110111   //3
#define FR B11101111   //4
#define FL B11011111   //5
#define BL B10111111   //6
#define ALL_HIGH B01111000 //Pulling 3,4,5,6 high 
#define ALL_LOW B10000111  //Pulling 3,4,5,6 low


#define SAMPLE 2000   //To find offsets
#define k 0.01    //contribution of accel data
#define dt 0.004   //Cycle time in seconds
#define ToDeg 57.3  // (180/PI)

//////PID CONSTANTS///
  #define kpP 4
  #define kiP 0.03
  #define kdP 1.5

  #define kpR 4
  #define kiR 0.03
  #define kdR 1.5

  #define kpY 10
/////////////////////

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

MPU6050 accelgyro;

int16_t rawAG[3][2]={0,0,0,0,0,0};  // ax ay az   gx gy gz
float offsets[3][2]={0,0,0,0,0,0};  // ax ay az   gx gy gz
float lastAG[3][2]={0,0,0,0,0,0};   // ax ay az   gx gy gz
double p=0, r=0, y=0;   //pitch roll yaw from gyro+accel data fusion

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

  ////INITITALIZE ESCs////
  PORTD |= ALL_HIGH;
  delayMicroseconds(1000);  //1ms
  PORTD &= ALL_LOW;
  ////////////////////////

  setupMPU();  /////SETUP THE SENSOR
  
  //Serial.begin(115200);///////////////////////////////////
}

void loop(){
  
  while(!(c&&Start))       //If MPU does not connect put throttle to 0%
  {                        //Wait for the arming sequence
    accelgyro.testConnection()? c=1 : c=0; 
     
    PORTD |= ALL_HIGH;
    delayMicroseconds(1000);  //0% throttle
    PORTD &= ALL_LOW;
    delayMicroseconds(1000);
  }

  runtime=micros();

  readMPU();


//////////////ESC PULSE BEGINS//////////////////
  unsigned long esc_high=micros();
  PORTD |= ALL_HIGH;    //Pull all esc pins HIGH  
////////////////////////////////////////////////

///////PART OF ESC PULSE DURATION/////////////// 
 // long tut=micros();  
  
  getYPR();
  
  if(throttle<1050)
    throttle=1000;
  else if(throttle>1800)
    throttle=1800;
   
  velBR=velFR=velFL=velBL=throttle;
  
  if((throttle>=1100)&&Start)
  {
    PIDpitch();
    PIDroll();  
    PIDyaw();
  }//~450us

 // tut=micros()-tut;
 // Serial.println(tut);
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
  /*  while(throttle>1000 && (accelgyro.testConnection()))
    {
      getYPR();
      velBR=velFR=velFL=velBL=throttle;
      pitch=roll=1500;
      PIDpitch();
      PIDroll();
      motorWrite();
      throttle-=4;
    }
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
    
  double err = p - pSetpoint;
  double d_err = err - p_l_err;
  p_i_err += err;
  p_l_err=err;

  double Out = kpP*err + kdP*d_err + kiP*p_i_err;

  if(Out>MaxPIDpitchOUT)
    Out=MaxPIDpitchOUT;
  else if(Out<(-MaxPIDpitchOUT))
    Out=(-MaxPIDpitchOUT);

  /////////////////////////
/*  Serial.print("PITCH PID OUTPUT: ");
  Serial.print(Out); */
  ////////////////////////

  velFL+=Out;
  velFR+=Out;
  velBR-=Out;
  velBL-=Out;   
}

inline void PIDroll(){

  if(roll>1450 && roll<1550)
    rSetpoint=0;
  else
    rSetpoint = map(roll, 1000,2000, -10.00,10.00);
  
  double err = r - rSetpoint;
  double d_err = err - r_l_err;
  r_i_err += err;
  r_l_err = err;

  double Out = kpR*err + kdR*d_err + kiR*r_i_err;
  
  if(Out>MaxPIDrollOUT)
    Out=MaxPIDrollOUT;
  else if(Out<(-MaxPIDrollOUT))
    Out=(-MaxPIDrollOUT);

  ///////////////////////////
/*  Serial.print("\t ROLL PID OUTPUT: ");
  Serial.println(Out); */
  ////////////////////////
  
  velFL+=Out;
  velFR-=Out;
  velBR-=Out;
  velBL+=Out;  
  
}

inline void PIDyaw(){

  if(yaw>1450 && yaw<1550)
    ySetpoint=0;
  else
    ySetpoint = map(yaw, 1000,2000, -5.00,5.00);
  
  double Out = kpY*(y - rSetpoint);
  
  if(Out>MaxPIDyawOUT)
    Out=MaxPIDyawOUT;
  else if(Out<(-MaxPIDyawOUT))
    Out=(-MaxPIDyawOUT);

  ///////////////////////////
/*  Serial.print("\t ROLL PID OUTPUT: ");
  Serial.println(Out); */
  ////////////////////////
  
  velFL+=Out;
  velFR-=Out;
  velBR+=Out;
  velBL-=Out;  
  
}

inline float aSin(float a){  //Optimised sine inverse
         
  return a*(1+(0.5*a*a));
}

inline void getYPR(){         //GET yaw pitch roll
        
    //accelgyro.getMotion6(&rawAG[0][0], &rawAG[1][0], &rawAG[2][0], &rawAG[0][1], &rawAG[1][1], &rawAG[2][1]);   

  //////////COMPLEMENTARY FILTERING TO GET YPR/////////
  p = (1-k)*( p - dt*(rawAG[1][1]*0.007634));      //(1/131) = 0.007634
     if(abs(rawAG[0][0])<=16384.0)
        p += k*(ToDeg)*aSin(rawAG[0][0]*0.00006104);  //(1/16384) = 0.00006104
        
  r = (1-k)*( r + dt*(rawAG[0][1]*0.007634));      //(1/131) = 0.007634
     if(abs(rawAG[1][0])<=16384.0)
        r += k*(ToDeg)*aSin(rawAG[1][0]*0.00006104);  //(1/16384) = 0.00006104
      
  y = rawAG[2][1]*0.007634;    //(1/131)
  ////////////////////////////////////////////////////

 /* Serial.print("p: ");
  Serial.print(p);
  Serial.print("\t | r: ");
  Serial.println(r); */
  
}

/*inline void esc(int pin, int us)  //old esc function
{  
  PORTD |= (1<<pin);    //digitalWrite(pin,1);
  delayMicroseconds(us);
  PORTD &= ~(1<<pin);   //digitalWrite(pin,0);
}*/

inline void readMPU(){
  
  Wire.beginTransmission(0x68);  //begin transmission with the gyro
  Wire.write(0x3B); //start reading from high byte register for accel
  Wire.endTransmission();
  Wire.requestFrom(0x68,14); //request 14 bytes from mpu
  //300us for all data to be received. 
  //requrestFrom is a blocking function 

  rawAG[0][0]=Wire.read()<<8|Wire.read();  
  rawAG[1][0]=Wire.read()<<8|Wire.read(); 
  rawAG[2][0]=Wire.read()<<8|Wire.read(); 
  rawAG[0][1]=Wire.read()<<8|Wire.read();  //this one is actually temperature but i dont need temp so why waste memory.
  rawAG[0][1]=Wire.read()<<8|Wire.read();  
  rawAG[1][1]=Wire.read()<<8|Wire.read();
  rawAG[2][1]=Wire.read()<<8|Wire.read();
  
  for(int i=0; i<=2; i++)
  {
      rawAG[i][0] -= offsets[i][0];
      rawAG[i][0] = 0.7*rawAG[i][0] + 0.3*lastAG[i][0];
      lastAG[i][0] = rawAG[i][0];

      rawAG[i][1] -= offsets[i][1];
      rawAG[i][1] = 0.7*rawAG[i][1] + 0.3*lastAG[i][1];
      lastAG[i][1] = rawAG[i][1];
  }//~360us
}

void setupMPU(){
  
  ////////MPU INITITALIZE/////////////
  Wire.setClock(800000);
  Wire.begin();
  accelgyro.initialize();
  accelgyro.testConnection() ? c=1 : c=0;

  delay(2000);  //Timeframe to keep the drone on the take off ground
  
  ///////OFFSET CALCULATION BEGINS//////
  
  int16_t ax=0, ay=0, az=0;  //accelerations from mpu6050
  int16_t gx=0, gy=0, gz=0;  //gyration rates from mpu6050
  
  for(int i=0;i<SAMPLE;i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    offsets[0][0]+=ax;
    offsets[1][0]+=ay;
    offsets[2][0]+=az;
    offsets[0][1]+=gx;
    offsets[1][1]+=gy;
    offsets[2][1]+=gz;    
  }
  
  for(int i=0; i<3; i++)
    for(int j=0; j<2; j++)
       offsets[i][j]/=SAMPLE;
  //////////////ENDS/////////////////
}
