// This is ESP8266 side of code for your custom ESP8266 based multirotor flight control board  
//  to be used together with the processing sketch 
//  the config tested was - my laptop was AP and ESP8266 was acting as the station
//  it is also possible that ESP is AP and laptop is connected to it, 
//  but in this config, ESP has to work more since it is creating the AP, hence responsiveness might decrease
//
//  After ESP boots, wait a few seconds till it calibrates the gyro offsets.... dont move the quad/board in the meantime 
//  later, the builtin leds will indicate the current status ex. whether the Wi-Fi is connected, quad is armed etc.. 

//  one of the feature is realtime PID control.. the PID constants can be changed without any need to reprogram the microcontroller every time 
//  the values can be changed even when the quad is in air!

//Strings that are gonna be exchanged - 
//
//to arduino from processing -
//  normal drive - max value - aAAb180b180d180e180f180g180h min value - aAb5c5d5e5f5g5h ;  
//        a (PID type A or B or C o D) b (ch 1) c (ch 2) d (ch 3) e (ch 4) f (ch 5) g (ch 6) h
//  changing PID - max value - pMq999r999s999t999u999v999w999x999y999z min value -  pMq1r1s1t1u1v1w1x1y1z  ;  
//        p (change or save - M or N) q (kpP) r (kiP) s (kdP) t (kpR) u (kiR) v (kdR) w (kpY) x (kiY) y (kdY) z 
//
//to processing from arduino -
//

#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Servo.h> '

//Intializing motors
Servo Mot1;
Servo Mot2;
Servo Mot3;
Servo Mot4;

//how many clients should be able to telnet to this ESP8266
#define MAX_SRV_CLIENTS 1
const char* ssid = "Bravos";
const char* password = "ValarMorghulis";

const char* ssid_rout0 = "Winterfell";
const char* pass_rout0 = "AryaStark";
//3.83 - 0.65
ADC_MODE(ADC_TOUT);

const bool pwm = true;
//const int P1 = 16, P2 = 14, P3 = 12, P4 = 13; //initiating pins
const int P1 = 3, P2 = 14, P3 = 12, P4 = 13; //initiating pins

const bool debug = false; // if false, no serial communication takes place and TX LED becomes status indicator...

unsigned long tim_vol = 0;
IPAddress ip1(10, 42, 3, 204);
IPAddress gateway1(10, 42, 3, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiServer server(300);
WiFiClient serverClients[MAX_SRV_CLIENTS];
WiFiClient gclient;
const int LED_PIN = 2;
const int MPU = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

unsigned long tim1, tim2, tim3, last_print = 0, last_led = 0;
unsigned long timer = 0, looptime = 0 , last_change = 0, lastsend_m = 0, 
no_wifi = 0,no_wifi1 = 0, loop1 = 0, lastloop = 0, last_sto = 0;

unsigned int ch[6], bch[6] ;

unsigned int addr = 100;


double kpR , kiR , kdR;
double kpP , kiP , kdP;
double kpY , kiY , kdY;

//intializing backup variables to prevent bad values from getting into calculation
double bkpR , bkiR , bkdR;
double bkpP , bkiP , bkdP;
double bkpY , bkiY , bkdY;

/*********************************************************/
                    byte PID_multiplier =  100;
/*********************************************************/

int count = 0;
int dPitch = 0, dRoll = 0 , dYaw = 0 ; 

double errP = 0 , erriP = 0 , errdP = 0 , errR = 0 , erriR = 0 , errdR = 0 , errY = 0 , erriY = 0 , errdY = 0;

double Ppid , Rpid , Ypid ;
double currPitch = 0, currRoll = 0, currYaw = 0;
unsigned long jps[2];
bool mot = false;
bool pri = false, last_arm = false, serial = false, acal = false, afl = false, err  = false;
double Ppid_limit =  40, Rpid_limit =  40, Ypid_limit = 40;
double PID_weight = 0.4, throttle_weight = 0.9, control_weight = 0.5;
int dm1 = 0, dm2 = 0, dm3 = 0, dm4 = 0;
double m1 , m2 , m3 , m4 ;
unsigned int pm1 , pm2 , pm3 , pm4 ;

double lm1 , lm2 , lm3 , lm4 ;

float dx = 0, dy = 0, dz = 0, ax = 0, ay = 0;
byte throttle = 4, pitch = 90, yaw = 90, roll = 90;
double apitch = 0, aroll = 0, ayaw = 0;
bool arm, counter = false,counter1 = false, had_wifi = false, sen = false, rao = false;
byte no_rf1, no_rf2;
unsigned int max_buf = 2000;
String aaa = "dn" ;

double fil[3]  = {0,0,0}; // filtered gyro values
double unf[3]  = {0,0,0}; // unfiltered offset compensated gyro values
double raw[3]  = {0,0,0}; // raw gyro values
double wm = 0.15;         // constant for low pass filering of gyro values
double uoff[3] = {0,0,0}; // unfiltered gyro offset
double off[3]  = {0,0,0}; // filtered gyro offset

double abac[3] = {0,0,0}, gbac[3] = {0,0,0}; // backup accel values

double afil[3]  = {0,0,0}; // filtered accel values
double afilc[3] = {0,0,0}; // variable for putting accel values while accel calibration
double araw[3]  = {0,0,0}; // raw accel values
double awm = 0.15;         // constant for low pass filering of accel values

double amin[3]  = {0,0,0};  // max accel value for calculating offset 
double amax[3]  = {0,0,0};  // min accel value for calculating offset 
double aoff[3]  = {0,0,0};  // calculated accel offset

bool Lsta = LOW; // LED pin status
String amode = "" ; // mode for 

int cali1 = 320, cali2 = 1000, cali3 = 560; // number of times values are read from gyro for finding gyro offsets 
bool l = LOW, lastsend = false, ypr = false,ara = false,fof = false, last_cal = false, vol = false;


byte pid_mode = 0;

void EWrite(unsigned int address, byte data)
{  if (EEPROM.read(address) != data) EEPROM.write(address, data); }

  

void E_Cal()
{
    // CHANGE FOR PWM

Mot1.attach(P1); 
Mot2.attach(P2); 
Mot3.attach(P3); 
Mot4.attach(P4); 

// Calibrating ESCs by writing max and min values 
// if any problem in calibrating or motor is behaving erratic, 
// try changing these values - bring them closer to each other
// and also remember to change corresponding values 
// only called when using brushless motors
delay(300);
   
      Mot1.write(170);
      Mot2.write(170);
      Mot3.write(170);
      Mot4.write(170);
      
  delay(2000);
  
      Mot1.write(30);
      Mot2.write(30);
      Mot3.write(30);
      Mot4.write(30);
      
  delay(2000);
}

void read_PID_val()
{
  // function called at boot where PID values are loaded from EEPROM
  EEPROM.begin(1024);
  addr = 100;

    // reading PID values from EEPROM
  kpP = ( EEPROM.read(addr) + ( 255 * (EEPROM.read(addr + 1)))) ; addr += 10;
  kiP = ( EEPROM.read(addr) + ( 255 * (EEPROM.read(addr + 1)))) ; addr += 10;
  kdP = ( EEPROM.read(addr) + ( 255 * (EEPROM.read(addr + 1)))) ; addr += 10;
  
  kpR = ( EEPROM.read(addr) + ( 255 * (EEPROM.read(addr + 1)))) ; addr += 10;
  kiR = ( EEPROM.read(addr) + ( 255 * (EEPROM.read(addr + 1)))) ; addr += 10;
  kdR = ( EEPROM.read(addr) + ( 255 * (EEPROM.read(addr + 1)))) ; addr += 10;
  
  kpY = ( EEPROM.read(addr) + ( 255 * (EEPROM.read(addr + 1)))) ; addr += 10;
  kiY = ( EEPROM.read(addr) + ( 255 * (EEPROM.read(addr + 1)))) ; addr += 10;
  kdY = ( EEPROM.read(addr) + ( 255 * (EEPROM.read(addr + 1)))) ;
  EEPROM.end();
  delayMicroseconds(20);

    // multiplier used to store float values of PID as integers in EEPROM
  kpP  /= PID_multiplier  ; kiP  /= PID_multiplier  ; kdP  /= PID_multiplier  ;
  kpR  /= PID_multiplier  ; kiR  /= PID_multiplier  ; kdR  /= PID_multiplier  ;
  kpY  /= PID_multiplier  ; kiY  /= PID_multiplier  ; kdY  /= PID_multiplier  ;

    if (debug)
    {
      Serial.print(kpY);Serial.print('\t');
      Serial.print(kiY);Serial.print('\t');
      Serial.print(kdY);Serial.println();
      delay(1);
      Serial.print(kpP);Serial.print('\t');
      Serial.print(kiP);Serial.print('\t');
      Serial.print(kdP);Serial.println();
      delay(1);
      Serial.print(kpR);Serial.print('\t');
      Serial.print(kiR);Serial.print('\t');
      Serial.print(kdR);Serial.println();
      Serial.println();
    }
 }

void store_PID_val()
{
  // function called whrn PID values are needed to be stored  
  unsigned long abcd = millis();

  EEPROM.begin(1024);
  addr = 100;

  kpP  *= PID_multiplier  ; kiP  *= PID_multiplier  ; kdP  *= PID_multiplier  ;
  kpR  *= PID_multiplier  ; kiR  *= PID_multiplier  ; kdR  *= PID_multiplier  ;
  kpY  *= PID_multiplier  ; kiY  *= PID_multiplier  ; kdY  *= PID_multiplier  ;
  
  
         EWrite(addr, (byte)( ((int)kpP) % 255 )) ; 
         EWrite(addr + 1,  (byte)( ((int)kpP) / 255 )) ; addr += 10;
         
         EWrite(addr, (byte)( ((int)kiP) % 255 )) ;    
         EWrite(addr + 1,  (byte)( ((int)kiP) / 255 )) ; addr += 10;
         
         EWrite(addr, (byte)( ((int)kdP) % 255 )) ;    
         EWrite(addr + 1,  (byte)( ((int)kdP) / 255 )) ; addr += 10;
 
         
         EWrite(addr, (byte)( ((int)kpR) % 255 )) ;    
         EWrite(addr + 1,  (byte)( ((int)kpR) / 255 )) ; addr += 10;
         
         EWrite(addr, (byte)( ((int)kiR) % 255 )) ;    
         EWrite(addr + 1,  (byte)( ((int)kiR) / 255 )) ; addr += 10;
         
         EWrite(addr, (byte)( ((int)kdR) % 255 )) ;    
         EWrite(addr + 1,  (byte)( ((int)kdR) / 255 )) ; addr += 10;
         
         
         EWrite(addr, (byte)( ((int)kpY) % 255 )) ;    
         EWrite(addr + 1,  (byte)( ((int)kpY) / 255 )) ; addr += 10;
         
         EWrite(addr, (byte)( ((int)kiY) % 255 )) ;    
         EWrite(addr + 1,  (byte)( ((int)kiY) / 255 )) ; addr += 10;
         
         EWrite(addr, (byte)( ((int)kdY) % 255 )) ;    
         EWrite(addr + 1,  (byte)( ((int)kdY) / 255 )) ; 

         
delayMicroseconds(40);

  kpP  /= PID_multiplier  ; kiP  /= PID_multiplier  ; kdP  /= PID_multiplier  ;
  kpR  /= PID_multiplier  ; kiR  /= PID_multiplier  ; kdR  /= PID_multiplier  ;
  kpY  /= PID_multiplier  ; kiY  /= PID_multiplier  ; kdY  /= PID_multiplier  ;

  if (debug)
    {
      Serial.print(kpY);Serial.print('\t');
      Serial.print(kiY);Serial.print('\t');
      Serial.print(kdY);Serial.println();
      delay(1);
      Serial.print(kpP);Serial.print('\t');
      Serial.print(kiP);Serial.print('\t');
      Serial.print(kdP);Serial.println();
      delay(1);
      Serial.print(kpR);Serial.print('\t');
      Serial.print(kiR);Serial.print('\t');
      Serial.print(kdR);Serial.println();
      Serial.println();
    }

 EEPROM.end();

//  Serial.println("end hua kya re?");
}



void dead_rf()
{
  // if Wi-Fi goes down.. or if due to any problem, communication is down,
  // throttle decreases continuously till the quad will land (blindly)..
  // this part is yet not perfect, next update will use accelerometer to smoothen the landing 
  // ultrasonic sensor also can be used for landing purposes...
  
  //  Serial.println("dead rf");delay(1);
           if ( throttle >= 110 && millis() - last_change >= 150 )//&& millis() - last_change < 75  )
                {throttle -= 1 ; last_change = millis();  }
      
           if ( throttle >= 93 && throttle < 110 && millis() - last_change >= 200 )// && millis() - last_change < 100) 
                {throttle -= 1 ; last_change = millis(); }
        
           if ( throttle < 93 && throttle >= 80 && millis() - last_change >= 250 )//&& millis() - last_change < 125)
                {throttle -= 1 ; last_change = millis(); }

           if ( throttle < 80  && millis() - last_change >= 150 && throttle >= 41 )
                {throttle -= 1 ; last_change = millis(); } 
         
           if ( throttle < 41)
                {arm = false;}
}


void read_raw()
{
  // read raw values from sensor
  delayMicroseconds(60);
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  raw[0] = GyX; raw[1] = GyY; raw[2] = GyZ;
  araw[0] = AcX; araw[1] = AcY; araw[2] = AcZ;
    for(int ga = 0; ga < 3 ; ga++)
      {
         if (raw[ga] < 28000 &&  raw[ga] > -28000 ) gbac[ga] = raw[ga];
        else raw[ga] = gbac[ga];
        if (araw[ga] < 28000 && araw[ga] > -28000 ) abac[ga] = araw[ga];
       else araw[ga] = abac[ga];
      }
  
  delayMicroseconds(60);
}

void Load_aoff()
{
 
 // Loading accelerometer offsets from EEPROM
 
 addr = 200; EEPROM.begin(4096);
 
  aoff[0] = ( EEPROM.read(addr) + ( 255 * (EEPROM.read(addr + 1) )) ) - 32768 ; addr += 10;
  aoff[1] = ( EEPROM.read(addr) + ( 255 * (EEPROM.read(addr + 1) )) ) - 32768 ; addr += 10;
  aoff[2] = ( EEPROM.read(addr) + ( 255 * (EEPROM.read(addr + 1) )) ) - 32768 ; addr += 10;
  delay(1);

EEPROM.end();

if (aoff[0] > 16000 || aoff[0] < -16000) aoff[0] = 0;
if (aoff[1] > 16000 || aoff[1] < -16000) aoff[1] = 0;
if (aoff[2] > 16000 || aoff[2] < -16000) aoff[2] = 0;

if (debug)
  {
    Serial.println("accel offsets- ");
    Serial.print(aoff[0]);  Serial.print('\t');
    Serial.print(aoff[1]);  Serial.print('\t');
    Serial.print(aoff[2]);  Serial.println();
  }
}


void Acc_off(bool sto)
{
  // Calibrating accelerometer values - 
  // enable Calibration mode from the processing sketch and move quad/board such that 
  // all axes are exposed to respective minimum and max values 
  
  if (!sto)
    {
       // this function is called when string "Calib" is recieved 
      read_raw();
      delay(2);  
      afilc[0] = (( 1 - awm ) * afilc[0]) + ( wm * araw[0]) ; 
      afilc[1] = (( 1 - awm ) * afilc[1]) + ( wm * araw[1]) ;
      afilc[2] = (( 1 - awm ) * afilc[2]) + ( wm * araw[2]) ;
      
      afilc[0] = constrain(afilc[0] , -28000, 28000);
      afilc[1] = constrain(afilc[1] , -28000, 28000);
      afilc[2] = constrain(afilc[2] , -28000, 28000);
    
      if (afilc[0] > amax[0] && afilc[0] < 25000 ) amax[0] = afilc[0] ;
      if (afilc[1] > amax[1] && afilc[1] < 25000 ) amax[1] = afilc[1] ;
      if (afilc[2] > amax[2] && afilc[2] < 25000 ) amax[2] = afilc[2] ;
      
      if (afilc[0] < amin[0] && afilc[0] > -25000 ) amin[0] = afilc[0] ;
      if (afilc[1] < amin[1] && afilc[1] > -25000 ) amin[1] = afilc[1] ;
      if (afilc[2] < amin[2] && afilc[2] > -25000 ) amin[2] = afilc[2] ;
    
    }

else if (sto && millis() - last_sto > 8000 ) 
    {
      // this part is called when string "CSave" is recieved 
      // stores calculated offset in EEPROM.. waits 8 sec as debouncing

      EEPROM.begin(4096);
      
      aoff[0] = (amax[0] + amin[0])/2 ;
      aoff[1] = (amax[1] + amin[1])/2 ;
      aoff[2] = (amax[2] + amin[2])/2 ;
      
      addr = 200;
      
      EWrite(addr, (byte)( ((int)aoff[0] + 32768) % 255 )) ;    
      EWrite(addr + 1,  (byte)( ((int)aoff[0] + 32768) / 255 )) ; addr += 10;
      
      EWrite(addr, (byte)( ((int)aoff[1] + 32768) % 255 )) ;    
      EWrite(addr + 1,  (byte)( ((int)aoff[1] + 32768) / 255 )) ; addr += 10;
      
      EWrite(addr, (byte)( ((int)aoff[2] + 32768) % 255 )) ;    
      EWrite(addr + 1,  (byte)( ((int)aoff[2] + 32768) / 255 )) ; addr += 10;
      delay(1);
      EEPROM.end();
      
    } 

}

void Calib()
{

  // just calling values for cali1 number of times to prevent transient 
  //  effect getting into offset calculation
  for ( int v = 0; v < cali1; v++)
  { 
    read_raw(); if(v%20 == 0) delay(1); 
    if (debug)
      {
        if(v%80 == 0) Serial.println();
        Serial.print('.');delayMicroseconds(60); 
      }
    else 
      {  
        if(v%80 == 0) { Lsta = !Lsta; digitalWrite(LED_PIN, Lsta); } 
      }
  }

  if(debug)
    {
      Serial.println();Serial.println("gyro- ");
      Serial.print(raw[0]);  Serial.print('\t');
      Serial.print(raw[1]);  Serial.print('\t');
      Serial.print(raw[2]);  Serial.println();
      Serial.println("accel- ");
      Serial.print(araw[0]);  Serial.print('\t');
      Serial.print(araw[1]);  Serial.print('\t');
      Serial.print(araw[2]);  Serial.println();
    }
  
delayMicroseconds(60);

  // finding unfiltered gyro offsets by taking values cali2 number of times and taking average
  // ideally for a quad at rest, gyro should return 0 for every axis, but this almost never happens 
  // hence, offset is calculated which is later subtracted from raw values to get proper value
  
  for ( int v = 0; v < cali2; v++)
  {
    read_raw(); if(v%20 == 0) delay(1);
    if (debug)
      {
        if(v%80 == 0) Serial.println();
        Serial.print('.');      
      }

    else 
      {  
        if(v%80 == 0) { Lsta = !Lsta; digitalWrite(LED_PIN, Lsta); } 
      }

    delayMicroseconds(60); 

    for ( int b = 0; b < 3; b++) {   uoff[b] +=  raw[b] / 20 ;} //  l = !l; digitalWrite( 13, l);  
  } 
  
  if (debug)
    {
       Serial.println();Serial.println("2 done");
    }

  for ( int b = 0; b < 3; b++) uoff[b] /= (cali2 / 20);

    if (debug)
      {
        Serial.print("Unfiltered gyro offsets -\t");
        Serial.print(uoff[0]); Serial.print('\t');
        Serial.print(uoff[1]); Serial.print('\t');
        Serial.print(uoff[2]); Serial.println();
      }

  delayMicroseconds(40);

  // finding filtered gyro offsets
  for ( int v = 0; v < cali3; v++)
  {
    read_raw();  if(v%20 == 0) delay(1);

    if(debug)
      {
        if(v%80 == 0) Serial.println();
        Serial.print('.');  delayMicroseconds(60); 
      }
    else 
      {  
        if(v%80 == 0) { Lsta = !Lsta; digitalWrite(LED_PIN, Lsta); } 
      }

    for ( int b = 0; b < 3; b++) 
    { unf[b] =  raw[b] -  uoff[b]; }
              //  l = !l; digitalWrite( 13, l);  
    delayMicroseconds(40);

    for ( int c = 0 ; c < 3 ; c++ )
    { fil[c] = (( 1 - wm ) * fil[c]) + ( wm * unf[c]) ;
      off[c] += fil[c] / 20 ;
    }
  }

  for ( int b = 0; b < 3; b++) { off[b] /= (cali2 / 20); }
  
  delayMicroseconds(40);
if(debug)
  {
    Serial.println();
    Serial.print("filtered gyro offsets -\t");
    Serial.print(off[0]); Serial.print('\t');
    Serial.print(off[1]); Serial.print('\t');
    Serial.print(off[2]); Serial.println();
  
     Serial.print("filtered accel offsets -\t");
    Serial.print(aoff[0]); Serial.print('\t');
    Serial.print(aoff[1]); Serial.print('\t');
    Serial.print(aoff[2]); Serial.println(); 
  }

}



void setup() {

  // CHANGE FOR PWM
if (pwm)
  {
    pinMode(P1, OUTPUT);
    pinMode(P2, OUTPUT);
    pinMode(P3, OUTPUT);
    pinMode(P4, OUTPUT);
    
    analogWrite(P1, 0);
    analogWrite(P2, 0);
    analogWrite(P3, 0);
    analogWrite(P4, 0);
  }

delay(1000);

Wire.begin(4,5);
Wire.beginTransmission(MPU);
Wire.write(0x6B); // PWR_MGMT_1 register
Wire.write(0); // set to zero (wakes up the MPU-6050)
Wire.endTransmission(true);

if(debug)
  {
    Serial.begin(115200); serial = true; delay(100);
    Serial.println("0 done");
  }

else 
  {
    pinMode(LED_PIN, OUTPUT);  serial = false ; digitalWrite(LED_PIN, LOW);
  }
  Load_aoff();
  read_raw();

     
if(debug)
  {    
    Serial.print(raw[0]);  Serial.print('\t');
    Serial.print(raw[1]);  Serial.print('\t');
    Serial.print(raw[2]);  Serial.println();
  
    Serial.print(araw[0]);  Serial.print('\t');
    Serial.print(araw[1]);  Serial.print('\t');
    Serial.print(araw[2]);  Serial.println();
  }
  Calib();


read_PID_val();
  delay(300);
  if(debug){Serial.println();  Serial.println();  Serial.println("Wifi disconnecting...");}
  WiFi.disconnect();  

  if(debug) Serial.println("Wifi disconnected...");
  else digitalWrite(LED_PIN, HIGH);

  delay(300);
  WiFi.mode(WIFI_STA);
  if(debug)  Serial.println("Mode Set...");
  
 delay(300);
  //WiFi.mode(WIFI_AP_STA);
  if(debug) Serial.println("Mode Set...");
      
  WiFi.begin(ssid_rout0, pass_rout0);
  WiFi.config(ip1, gateway1, subnet);

  delay(900);

  if(debug)      Serial.println("some text to make light blink"); else  digitalWrite(LED_PIN, HIGH);
  //  WiFi.softAP(ssid, password);   
  delay(600);
  IPAddress myIP = WiFi.softAPIP();
 
  server.begin();
  server.setNoDelay(true);

  delay(1100);if(debug)Serial.println("some delay");  else digitalWrite(LED_PIN,  Lsta);
  delay(1100);if(debug)Serial.println("some delay");  else digitalWrite(LED_PIN, !Lsta);
  delay(400) ;if(debug)Serial.println("some delay");  else digitalWrite(LED_PIN,  Lsta);
  delay(400) ;if(debug)Serial.println("some delay");  else digitalWrite(LED_PIN, !Lsta);
  delay(300) ;if(debug)Serial.println("some delay");  else digitalWrite(LED_PIN,  Lsta);

  
  // CHANGE FOR PWM
if (!pwm)  E_Cal();

if(debug)
  {
    Serial.println(WiFi.localIP());  Serial.println("Setup Done :)");
  }

}

void loop() {
  // put your main code here, to run repeatedly:
bool mod1 = false;
 uint8_t i;
  //check if there are any new clients
  if (server.hasClient()){
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      //find free/disconnected spot
      if (!serverClients[i] || !serverClients[i].connected()){
        if(serverClients[i]) serverClients[i].stop();
        serverClients[i] = server.available();
if(debug) { Serial.print("New client: "); Serial.println(i); }
        continue;
      }
    }
    delayMicroseconds(40);
    //no free/disconnected spot so reject
    WiFiClient serverClient = server.available();
    serverClient.stop();
  }
  //check clients for data
  for(i = 0; i < MAX_SRV_CLIENTS; i++){
    if (serverClients[i] && serverClients[i].connected()){
 
//           Serial.println(serverClients[i].available()); 
      if(serverClients[i].available())
      {
        had_wifi = true; counter = false; no_rf1 = false;no_rf2 = false; counter1 = false; aaa[1] = 'w';
//     Serial.print("av-\t");Serial.print(serverClients[i].available());          tim1 = micros();
     
              String ab = serverClients[i].readStringUntil('\n');serverClients[i].setTimeout(5);                          
              
 //              Serial.print('\t');Serial.print(micros() - tim1);
 //              Serial.print('\t');Serial.print(serverClients[i].available());
               
               
//            Serial.print('\t');  
//            Serial.println(ab);
            delayMicroseconds(20);
// aAAb180b180d180e180f180g180h
// aAb5c5d5e5f5g5h
// if all letters are detected then only proceed forward
 if (ab[0] == 'a' && ab.length() <= 30 && ab.length() >= 12)
          {             // checking length so that half strings or double strings will be neglected
            
            acal = false; last_cal = false; // tim2 = micros(); 

if ( ab.indexOf('a') != -1 && ab.indexOf('b') != -1 && ab.indexOf('c') != -1 &&
     ab.indexOf('d') != -1 && ab.indexOf('e') != -1 && ab.indexOf('f') != -1 &&
     ab.indexOf('g') != -1 && ab.indexOf('h') != -1 )
     {
       String aa = ab.substring(ab.indexOf('a') + 1 , ab.indexOf('b') );
       String bb = ab.substring(ab.indexOf('b') + 1 , ab.indexOf('c') );
       String cc = ab.substring(ab.indexOf('c') + 1 , ab.indexOf('d') );
       String dd = ab.substring(ab.indexOf('d') + 1 , ab.indexOf('e') );
       String ee = ab.substring(ab.indexOf('e') + 1 , ab.indexOf('f') );
       String ff = ab.substring(ab.indexOf('f') + 1 , ab.indexOf('g') );
       String gg = ab.substring(ab.indexOf('g') + 1 , ab.indexOf('h') );
       //Serial.print(micros() - tim2);Serial.println("\t<-time for\t");
       delayMicroseconds(20);

       ch[0] = bb.toInt();    ch[1] = cc.toInt();
       ch[2] = dd.toInt();    ch[3] = ee.toInt();
       ch[4] = ff.toInt();    ch[5] = gg.toInt();

      unsigned int VO =  0; 
      // VO = ESP.getVcc();
      VO = analogRead(A0);

      String voo = String(VO);
if (aa[1] == 'A') // client asking for nothing; but voltage is send continuously..
      {
        if (millis() - lastsend_m >  150) 
        { 
        String voltage = String(analogRead(A0));
        String vals = "";
        vals  = "vo";
        vals += voltage;
        vals += "l\n";              
        serverClients[i].print(vals);delay(2);
        lastsend_m = millis(); 
        }
      }

   else if (aa[1] == 'G') // client asking for gyro values 
      {
        if (millis() - lastsend_m >  150) 
        {
        String vals = "";
        vals = "gg" ;   vals += voo;
        vals += "i" ;   vals += String(((int)(fil[0]))) ;
        vals += "j" ;   vals += String(((int)(fil[1]))) ;
        vals += "k" ;   vals += String(((int)(fil[2]))) ;
        vals += "l\n";
        serverClients[i].print(vals);delay(2); 
        lastsend_m = millis(); 
        if(debug)Serial.println(" gg  ") ; 
        }// Serial.print(vals); }
      }
   else if (aa[1] == 'H') // client asking for accel values 
      {
        if (millis() - lastsend_m >  150) 
        {
        String vals = "";
        vals = "hh" ;   vals += voo;
        vals += "i" ;   vals += String(((int)(afil[0]/16.701))) ;
        vals += "j" ;   vals += String(((int)(afil[1]/16.701))) ;
        vals += "k" ;   vals += String(((int)(afil[2]/16.701))) ;
        vals += "l\n";
        serverClients[i].print(vals);delay(2); 
        lastsend_m = millis(); 
        if(debug)Serial.println(" hh  ") ; }// Serial.print(vals);}
      }
   else if (aa[1] == 'I') // client asking for motor values 
      {
        if (millis() - lastsend_m >  150) 
        {
        String vals = "";
        vals = "nn" ; vals += voo;
        if (pwm)
          {
            vals += "i" ;   vals += String(((int)(pm1))) ;
            vals += "j" ;   vals += String(((int)(pm2))) ;
            vals += "k" ;   vals += String(((int)(pm3))) ;
            vals += "l" ;   vals += String(((int)(pm4))) ;
          }
        else 
          {
            vals += "i" ;   vals += String(((int)(m1))) ;
            vals += "j" ;   vals += String(((int)(m2))) ;
            vals += "k" ;   vals += String(((int)(m3))) ;
            vals += "l" ;   vals += String(((int)(m4))) ;            
          }
        vals += "m\n";
           serverClients[i].print(vals);delay(2); 
           lastsend_m = millis(); 
           if(debug)Serial.println(" mo  ") ; 
           }// Serial.print(vals);}
      }

   else if (aa[1] == 'J') // client asking for roll & pitch values 
      {
        if (millis() - lastsend_m >  150) 
        {
        String vals = "";
        vals = "pr" ;   vals += voo;
        vals += "i" ;   vals += String(((int)(apitch*100))) ;
        vals += "j" ;   vals += String(((int)(aroll*100))) ;
        vals += "k\n";
        serverClients[i].print(vals);delay(2); 
        lastsend_m = millis(); 
        if(debug)Serial.println(" an  ") ;
        }// Serial.print(vals);}
      }      
// the below mentioned PID modes are what i was trying to fin the best type of PID for my quad 
// only first is tested yet
// feel free to edit the modes and add them per will... 
        if (aa[0] == 'A') pid_mode = 0; // only gyro values used
   else if (aa[0] == 'B') pid_mode = 1; // accel value used for roll pitch and gyro for yaw
   else if (aa[0] == 'C') pid_mode = 2; // complimentary filter.. using both accel and gyro values
   else if (aa[0] == 'D') pid_mode = 3; // uses gyro values till a range then accel values

                      
     }
/*
 * throttle (R Slider)= ch[2] ; arm(L Slider) = ch[5] ;
 * pitch (Left Y) = ch[0] ; roll ( Left X ) = ch[4] ;
 * yaw ( Right X ) = ch[1] ; nil ( Right Y ) = ch[3] ;
 */

 
for ( int ll = 0 ; ll < 6; ll++)  
          if ( ch[ll] >= 5 && ch[ll] <= 175 ) bch[ll] = ch[ll];  else ch[ll] = bch[ll];
/*              
          
            Serial.print(ch[0]);Serial.print('\t');            Serial.print(ch[1]);Serial.print('\t');
            Serial.print(ch[2]);Serial.print('\t');            Serial.print(ch[3]);Serial.print('\t');
            Serial.print(ch[4]);Serial.print('\t');            Serial.print(ch[5]);Serial.println('\t');
*/
     
          }
//  String received will vary from pMq1r1s1t1u1v1w1x1y1z to  pMq999r999s999t999u999v999w999x999y999z
//  max val restricted at 999 to decrease the processing load on esp but feel free to take it 65535 
//  as 2 EEPROM bytes have been allotted for each value

 if (ab[0] == 'p' && ab.length() <= 48 && ab.length() >= 18 )
          {
            // checking length so that half strings or double strings will be neglected
            acal = false;     last_cal = false;
if ( ab.indexOf('p') != -1 && ab.indexOf('q') != -1 && ab.indexOf('r') != -1 &&
     ab.indexOf('s') != -1 && ab.indexOf('t') != -1 && ab.indexOf('u') != -1 &&
     ab.indexOf('v') != -1 && ab.indexOf('w') != -1 && ab.indexOf('x') != -1 &&
     ab.indexOf('y') != -1 && ab.indexOf('z') != -1 )
        {            
           String aa = ab.substring(ab.indexOf('p') + 1 , ab.indexOf('q') );
           String bb = ab.substring(ab.indexOf('q') + 1 , ab.indexOf('r') );
           String cc = ab.substring(ab.indexOf('r') + 1 , ab.indexOf('s') );
           String dd = ab.substring(ab.indexOf('s') + 1 , ab.indexOf('t') );
           String ee = ab.substring(ab.indexOf('t') + 1 , ab.indexOf('u') );
           String ff = ab.substring(ab.indexOf('u') + 1 , ab.indexOf('v') );
           String gg = ab.substring(ab.indexOf('v') + 1 , ab.indexOf('w') );
           String hh = ab.substring(ab.indexOf('w') + 1 , ab.indexOf('x') );
           String ii = ab.substring(ab.indexOf('x') + 1 , ab.indexOf('y') );
           String jj = ab.substring(ab.indexOf('y') + 1 , ab.indexOf('z') );

           delayMicroseconds(30);

           kpP = bb.toInt();   kiP = cc.toInt();   kdP = dd.toInt();
           kpR = ee.toInt();   kiR = ff.toInt();   kdR = gg.toInt();
           kpY = hh.toInt();   kiY = ii.toInt();   kdY = jj.toInt();

          if ( kpP >= 0 && kpP <= 999 ) bkpP = kpP;  else kpP = bkpP;
          if ( kiP >= 0 && kiP <= 999 ) bkiP = kiP;  else kiP = bkiP;
          if ( kdP >= 0 && kdP <= 999 ) bkdP = kdP;  else kdP = bkdP;

          if ( kpR >= 0 && kpR <= 999 ) bkpR = kpR;  else kpR = bkpR;
          if ( kiR >= 0 && kiR <= 999 ) bkiR = kiR;  else kiR = bkiR;
          if ( kdR >= 0 && kdR <= 999 ) bkdR = kdR;  else kdR = bkdR;

          if ( kpY >= 0 && kpY <= 999 ) bkpY = kpY;  else kpY = bkpY;
          if ( kiY >= 0 && kiY <= 999 ) bkiY = kiY;  else kiY = bkiY;
          if ( kdY >= 0 && kdY <= 999 ) bkdY = kdY;  else kdY = bkdY;
          
            kpP  /= PID_multiplier ;  kiP  /= PID_multiplier ;  kdP  /= PID_multiplier ;
            kpR  /= PID_multiplier ;  kiR  /= PID_multiplier ;  kdR  /= PID_multiplier ;
            kpY  /= PID_multiplier ;  kiY  /= PID_multiplier ;  kdY  /= PID_multiplier ;

/*
    Serial.print(kpY);Serial.print('\t');
    Serial.print(kiY);Serial.print('\t');
    Serial.print(kdY);Serial.print('\t');
          delayMicroseconds(40);
    Serial.print(kpP);Serial.print('\t');
    Serial.print(kiP);Serial.print('\t');
    Serial.print(kdP);Serial.print('\t');
          delayMicroseconds(40);
    Serial.print(kpR);Serial.print('\t');
    Serial.print(kiR);Serial.print('\t');
    Serial.print(kdR);
    Serial.println();
*/
            
//        Serial.println(aa[0]);
          if (aa[0] == 'N' && millis() - timer > 8000 )
              {
                 if(debug)Serial.println(" \t Saved \t");  store_PID_val(); 
                 timer = millis();
              }
      
        

        if (millis() - lastsend_m >  150) 
        {
        String voltage = String(analogRead(A0));
        String vals = "";
        vals  = "v";
        vals += voltage;
        vals += "l\n";              
        serverClients[i].print(vals);delay(2);
        lastsend_m = millis(); 
        }
          }
      }

if (ab[0] == 'C' )
{
  String accc ;
//  Serial.print("  c  ");
if (ab.indexOf("Calib") != -1 ) 
    { //Calib is received
      acal = true; 
     unsigned int VO =  0; 
    // VO = ESP.getVcc();
    VO = analogRead(A0);
    String voo = String(VO);
      if (amode != "val" && amode!= "min" && amode != "max" && amode != "off") amode = "val";
      // above step alternates sending of all the below data strings 
      // so that data rate is effectively minimized 
      // as higher data rates will affect performance of the ESP


//      afilc is constrained to 28000 and 28000/16.701 = 1676
//        Range of sent strings - vv0i-1676j-1676k-1676l to vv1023i1676j1676k1676l
//      amax is constrained between 0 and  25000 and  25000/16.701 =  1496
//        Range of sent strings - ma0i-1496j-1496k-1496l to ma1023i1496j1496k1496l
//      amin is constrained between 0 and -25000 and -25000/16.701 = -1496
//        Range of sent strings - mi0i-1496j-1496k-1496l to mi1023i1496j1496k1496l
//      aoff is constrained between 0 and -25000 and -25000/16.701 = -1496
//        Range of sent strings - oo0i-1496j-1496k-1496l to oo1023i1496j1496k1496l
      
      if(!last_cal) for(int aca = 0 ; aca < 3; aca++) { amax[aca] = 0; amin[aca] = 0;}
      last_cal = true;
    
       if (amode == "val")
         {
           accc = "cu" ;accc += voo;
           accc += "i" ;   accc += String(((int)(afilc[0]/16.701))) ;
           accc += "j" ;   accc += String(((int)(afilc[1]/16.701))) ;
           accc += "k" ;   accc += String(((int)(afilc[2]/16.701))) ;
           accc += "l\n";
           if(debug)   Serial.print(" 1 "); amode = "max";
         }
       
       else if (amode == "max")
         {
           accc = "ma" ;accc += voo;
           accc += "i" ;   accc += String(((int)(amax[0]/16.701))) ;
           accc += "j" ;   accc += String(((int)(amax[1]/16.701))) ;
           accc += "k" ;   accc += String(((int)(amax[2]/16.701))) ;
           accc += "l\n";
           if(debug)   Serial.print(" 2 "); amode = "min";
          }    
       else if (amode == "min")
          {
            accc = "mm" ;accc += voo;
            accc += "i" ;   accc += String(((int)(amin[0]/16.701))) ;
            accc += "j" ;   accc += String(((int)(amin[1]/16.701))) ;
            accc += "k" ;   accc += String(((int)(amin[2]/16.701))) ;
            accc += "l\n"; 
            if(debug)   Serial.print(" 3 "); amode = "off";
          }    
      
      else if (amode = "off")
          {
            accc = "oo" ;accc += voo;
            accc += "i" ;   accc += String(((int)(aoff[0]/16.701))) ;
            accc += "j" ;   accc += String(((int)(aoff[1]/16.701))) ;
            accc += "k" ;   accc += String(((int)(aoff[2]/16.701))) ;
            accc += "l\n"; 
            if(debug)   Serial.print(" 4 ");  amode = "val";
           }
      
      if(debug)Serial.println();
      
    }

else if (ab.indexOf("Save") != -1 )  // CSave is received
    {  
      Acc_off(true);   
    }

   delay(3);
//   if (!lastsend)   {  
    serverClients[i].print(accc);delay(3); 
//    lastsend = true;  } else lastsend = false;
  }

 if (ab[0] == 'G' && ab.indexOf("PID") != -1 ) 
{
double akpR , akiR , akdR;
double akpP , akiP , akdP;
double akpY , akiY , akdY;
  
akpP = kpP * PID_multiplier; akiP = kiP * PID_multiplier; akdP = kdP * PID_multiplier; 
akpR = kpR * PID_multiplier; akiR = kiR * PID_multiplier; akdR = kdR * PID_multiplier; 
akpY = kpY * PID_multiplier; akiY = kiY * PID_multiplier; akdY = kdY * PID_multiplier; 
  
  String pidp = String((int)akpP) + "r" + String((int)akiP) + "s" + String((int)akdP) + "t"  ;  
  String pidr = String((int)akpR) + "u" + String((int)akiR) + "v" + String((int)akdR) + "w"  ;  
  String pidy = String((int)akpY) + "x" + String((int)akiY) + "y" + String((int)akdY) + "z"  ;
  
  String pid = "Kq";
  pid += pidp + pidr + pidy + "\n";
      serverClients[i].print(pid);delay(3); 

}
    
      } // serverClients[i].available() ends
   //     while(serverClients[i].available()) char buf = serverClients[i].read();

      
    } // connected ends
    
delay(1);

         if ( serverClients[i].available() > max_buf )   // clears the buffer just in case it's flooded by bad data
          while(serverClients[i].available()) char buf = serverClients[i].read();
        
   
 if( !serverClients[i].available() || serverClients[i].available() > max_buf)
      {  
        if (counter1 && millis() - no_wifi1 > 370 )
          {   
            aaa[1] = 'n'; //aaa[3] = 'w';
            if ( serverClients[i].available() > max_buf )
                while(serverClients[i].available()) char buf = serverClients[i].read();
          }
  
         if (!counter1) 
          { 
            counter1 = true;  
            no_wifi1 = millis();   
          }
      }

    if( (!serverClients[i].available() && had_wifi) || serverClients[i].available() > max_buf)
      {  
        if (!counter)
            { 
              counter = true; no_wifi = millis(); last_change = millis(); 
            }
  
    else if (counter)
            {
              if ( millis() - no_wifi > 370 )   no_rf1 = true ; // if no wifi data received for 370 miliseconds - roll pitch yaw defaults to centre value
              if ( millis() - no_wifi > 500 )   no_rf2 = true ; // if no wifi data received for 500 miliseconds - throttle decreases slowly.. so that quad can land
            }
       }
       

  } // i < max server clients ends

  
  delayMicroseconds(20);
  
/*************************************************************************/
  
    if (!no_rf2) 
        {
                  throttle = ch[2];
                  if ( ch[5] >  90) { if (  last_arm ) arm = true;  last_arm = true; }
             else if ( ch[5] <= 90) { if ( !last_arm ) arm = false; last_arm = false;}
        }
        
  else if (no_rf2) dead_rf();
  
if (!no_rf1)
      {
            yaw    = ch[1];
            pitch  = ch[0];
            roll   = ch[4];
      }
      
else if (no_rf1)
      {
            yaw    = 90;
            pitch  = 90;
            roll   = 90;
      }


delayMicroseconds(30);

  read_raw();
// finding proper offset-compensated and low pass filtered values  ...

  unf[0] = raw[0] - uoff[0] ;
  unf[1] = raw[1] - uoff[1] ;
  unf[2] = raw[2] - uoff[2] ;
                              
  fil[0] = (( 1 - wm ) * fil[0]) + ( wm * (unf[0] - off[0]));
  fil[1] = (( 1 - wm ) * fil[1]) + ( wm * (unf[1] - off[1]));
  fil[2] = (( 1 - wm ) * fil[2]) + ( wm * (unf[2] - off[2]));
delayMicroseconds(20);
//PID_weight = 0;

dx = fil[1]/500 ; dy = -fil[0]/500; dz = fil[2]/500;

dx  = constrain(dx , -7500 , 7500 );
dy  = constrain(dy , -7500 , 7500 );
dz  = constrain(dz , -7500 , 7500 );

  afil[0] = (( 1 - awm ) * afil[0]) + ( wm * (araw[0] - aoff[0]));
  afil[1] = (( 1 - awm ) * afil[1]) + ( wm * (araw[1] - aoff[1]));
  afil[2] = (( 1 - awm ) * afil[2]) + ( wm * (araw[2] - aoff[2]));
  
  apitch = atan2(afil[1], sqrt((afil[0] * afil[0]) + (afil[2] * afil[2])) ) * 180/PI;
  aroll  = atan2(afil[0], sqrt((afil[1] * afil[1]) + (afil[2] * afil[2])) ) * 180/PI;
  
  ay = apitch * 0.75;
  ax = -aroll * 0.75;
  
  if ( arm )
  {

    /************************* Golden Part of Code ******************************/
    {
        delayMicroseconds(20);

dPitch = pitch - 90; dRoll = roll - 90 ; dYaw = yaw - 90; // finding change needed in motors due to change in controls 
     
if (pid_mode != 0 && pid_mode != 1 && pid_mode != 2 && pid_mode != 3 ) pid_mode = 0;

if (pid_mode == 0 )
{
      /************************************************************/
      errP  = ( dx )  ; erriP += errP/6 ; errdP -= errP/6 ;
      errR  = ( dy )  ; erriR += errR/6 ; errdR -= errR/6 ;
      errY  = (-dz )  ; erriY += errY/6 ; errdY -= errY/6 ;
      /************************************************************/ 
}
else if  (pid_mode == 1 )

{
      /************************************************************/
      errP  = ( ax )  ; erriP += errP/6 ; errdP -= errP/6 ;
      errR  = (-ay )  ; erriR += errR/6 ; errdR -= errR/6 ;
      errY  = (-dz )      ; erriY += errY/6 ; errdY -= errY/6 ;
      /************************************************************/  
}

else if  (pid_mode == 2 )

{
      /************************************************************/
      errP  = ( ax + dx )  ; erriP += errP/6 ; errdP -= errP/6 ;
      errR  = (-ay + dy )  ; erriR += errR/6 ; errdR -= errR/6 ;
      errY  = (-dz )           ; erriY += errY/6 ; errdY -= errY/6 ;
      /************************************************************/
}
//should be complicated .... for low pitch roll values .. use gyro values... for higher pitch roll values.. use accel values
else if  (pid_mode == 3 )

{  
     double pii =  ( pow(2*( ax + dPitch), 3) + 7*dx);
     double roo =  ( pow(2*(-ay + dRoll) , 3) + 7*dy);
     int pi2 = 1, ro2 = 1;
     if (pii < 0 ) pi2 = -1; if (roo < 0 ) ro2 = -1;
     
     /*****************************************************************************************/
      errP  = pi2*pow(abs(pii), (1.00/3.00))/2 ; erriP += errP/6 ; errdP -= errP/6 ;
      errR  = ro2*pow(abs(roo), (1.00/3.00))/2 ; erriR += errR/6 ; errdR -= errR/6 ;
      errY  = (-dz )                         ; erriY += errY/6 ; errdY -= errY/6 ;
     /*****************************************************************************************/

}
errP  = constrain(errP , -8000, 8000);  errR  = constrain(errR , -8000, 8000);  errY  = constrain(errY , -8000, 8000);
erriP = constrain(erriP, -9000, 9000);  erriR = constrain(erriR, -9000, 9000);  erriY = constrain(erriY, -9000, 9000);
errdP = constrain(errdP, -9000, 9000);  errdR = constrain(errdR, -9000, 9000);  errdY = constrain(errdY, -9000, 9000);

// putting constraints on error so that motor values wont change rapidly in case of a bad value

      /*******************************************************************/
      Ppid =  ( kpP * errP ) + ( kiP * erriP ) + ( kdP * errdP ) ;
      Rpid =  ( kpR * errR ) + ( kiR * erriR ) + ( kdR * errdR ) ;
      Ypid =  ( kpY * errY ) + ( kiY * erriY ) + ( kdY * errdY ) ;
      /*******************************************************************/

delayMicroseconds(20);

      Ppid = constrain(Ppid, (-Ppid_limit), Ppid_limit);
      Rpid = constrain(Rpid, (-Rpid_limit), Rpid_limit);
      Ypid = constrain(Ypid, (-Ypid_limit), Ypid_limit);
      
      
delayMicroseconds(20);

if (pid_mode == 0 || pid_mode == 1 || pid_mode == 2 ) //!(pid_mode*mod1))
{
      dm1 = +dPitch -dRoll -dYaw ;
      dm2 = -dPitch -dRoll +dYaw ;
      dm3 = -dPitch +dRoll -dYaw ; 
      dm4 = +dPitch +dRoll +dYaw ;

      dm1 = constrain(dm1,-75 ,75);
      dm2 = constrain(dm2,-75 ,75);
      dm3 = constrain(dm3,-75 ,75);
      dm4 = constrain(dm4,-75 ,75);


/************************************************************************************************************************/
   m1 = ((throttle_weight) * (throttle)) + (PID_weight * ( - Ppid + Rpid + Ypid )) + ((control_weight) * ( dm1 )) ;
   m2 = ((throttle_weight) * (throttle)) + (PID_weight * ( + Ppid + Rpid - Ypid )) + ((control_weight) * ( dm2 )) ;
   m3 = ((throttle_weight) * (throttle)) + (PID_weight * ( + Ppid - Rpid + Ypid )) + ((control_weight) * ( dm3 )) ;
   m4 = ((throttle_weight) * (throttle)) + (PID_weight * ( - Ppid - Rpid - Ypid )) + ((control_weight) * ( dm4 )) ;
/*************************************************************************************************************************/
}

else 
{

      dm1 =  -dYaw ;
      dm2 =  +dYaw ;
      dm3 =  -dYaw ; 
      dm4 =  +dYaw ;

      dm1 = constrain(dm1,-50 ,50);
      dm2 = constrain(dm2,-50 ,50);
      dm3 = constrain(dm3,-50 ,50);
      dm4 = constrain(dm4,-50 ,50);


/************************************************************************************************************************/
   m1 = ((throttle_weight) * (throttle)) + (PID_weight * ( - Ppid + Rpid + Ypid )) + ((control_weight) * ( dm1 )) ;
   m2 = ((throttle_weight) * (throttle)) + (PID_weight * ( + Ppid + Rpid - Ypid )) + ((control_weight) * ( dm2 )) ;
   m3 = ((throttle_weight) * (throttle)) + (PID_weight * ( + Ppid - Rpid + Ypid )) + ((control_weight) * ( dm3 )) ;
   m4 = ((throttle_weight) * (throttle)) + (PID_weight * ( - Ppid - Rpid - Ypid )) + ((control_weight) * ( dm4 )) ;
/*************************************************************************************************************************/

}


if(lm1-m1> 40 || lm1-m1 < -40 || lm2-m2> 40 || lm2-m1 < -40 || lm3-m3> 40 || lm3-m1 < -40 || lm1-m1> 40 || lm1-m1 < -40 )
  {
    m1 = lm1; m2 = lm2; m3=lm3; m4=lm4; 
    // if motor value changed rapidly in a asingle loop, the new value is neglected and old value is used
  }
lm1 = m1; lm2 = m2; lm3 = m3; lm4 = m4;
   }

if ( isnan(m1) == 1 || isnan(m2) == 1 || isnan(m3) == 1 || isnan(m4) == 1 )
{
  m1 = 0; m2 = 0; m3 = 0; m4 = 0;
}
    m1 = constrain(m1 , 38, 150);
    m2 = constrain(m2 , 38, 150);
    m3 = constrain(m3 , 38, 150);
    m4 = constrain(m4 , 38, 150);
// CHANGE FOR PWM

    pm1 = map(m1, 38, 150, 0, 1023);
    pm2 = map(m2, 38, 150, 0, 1023);
    pm3 = map(m3, 38, 150, 0, 1023);
    pm4 = map(m4, 38, 150, 0, 1023);

delayMicroseconds(20);

  }


  else if ( !arm )
  {
    delayMicroseconds(20);

    m1 = 38;
    m2 = 38;
    m3 = 38;
    m4 = 38;
    lm1 = m1; lm2 = m2; lm3 = m3; lm4 = m4;

    // CHANGED FOR PWM
    pm1 = 0;
    pm2 = 0;
    pm3 = 0;
    pm4 = 0;
    delay(2);
//aaa[0] = 'd';//aaa[2] = 'd';
  }

if (pwm)
{
      analogWrite(P1, pm1);
      analogWrite(P2, pm2);
      analogWrite(P3, pm3);
      analogWrite(P4, pm4);
}
else 
{
      Mot1.write(m1);
      Mot2.write(m2);
      Mot3.write(m3);
      Mot4.write(m4);
}
         
         
         delayMicroseconds(20);

      if (millis() - looptime > 20 && pri && debug){

Serial.print(throttle); Serial.print('\t');
Serial.print(arm); Serial.print('\t');


Serial.print(dx); Serial.print('\t');
Serial.print(dy); Serial.print('\t');
Serial.print(dz); Serial.print('\t');

Serial.print(m1); Serial.print('\t');
Serial.print(m2); Serial.print('\t');
Serial.print(m3); Serial.print('\t');
Serial.print(m4); Serial.print('\t');

          
Serial.println();looptime = millis();
}

delay(1);

// below function can be used for debugging... just enter these texts in Serial monitor to get the corresponding values....
  if(debug && Serial.available())
    {
       String aa = Serial.readStringUntil('\n');Serial.setTimeout(5);
            if (aa == "rrr") ESP.restart();  // restarts ESP
       else if (aa == "pri") pri = true;     // 
       else if (aa == "npr") pri = false;    // 
       else if (aa == "sen") sen = !sen;     // prints filtered gyro values
       else if (aa == "rao") rao = !rao;     // prints raw gyrovalues
       else if (aa == "afl") afl = !afl;     // prints filtered accel values
       else if (aa == "ara") ara = !ara;     // prints raw accel values
       else if (aa == "fof") fof = !fof;     // prints complimentary (pidMode - 2 )values
       else if (aa == "ypr") ypr = !ypr;     // prints pitch roll values
       else if (aa == "err") err = !err;     // prints error values
       else if (aa == "mot") mot = !mot;     // prints motor values 
       else if (aa == "vol") vol = !vol;     // prints adc values       
       else if (aa == "aof" && debug) 
       {
            Serial.println("accel offsets- ");
    Serial.print(aoff[0]);  Serial.print('\t');
    Serial.print(aoff[1]);  Serial.print('\t');
    Serial.print(aoff[2]);  Serial.println();
       }
       else if (aa == "eep" && debug)
        { EEPROM.begin(4096); for (int ee = 0; ee < 4096; ee++) 
          {
            Serial.print(ee); Serial.print(" ;  ");Serial.println(EEPROM.read(ee));
          }
          EEPROM.end();  
        }
    }

 if(debug && millis() - last_print > 80){
  
  if (vol) 
{
    Serial.print("vol-\t");
    Serial.println(analogRead(A0));
    
}
else if (sen) 
{
    //Serial.print("filte -\t");
  Serial.print(fil[0]); Serial.print('\t');
  Serial.print(fil[1]); Serial.print('\t');
  Serial.print(fil[2]); Serial.println();
  delay(10);
}

else if (afl) 
{
    //Serial.print("filte -\t");
  Serial.print(afil[0]); Serial.print('\t');
  Serial.print(afil[1]); Serial.print('\t');
  Serial.print(afil[2]); Serial.println();
  delay(10);
}

else if (err) 
{
    //Serial.print("filte -\t");
  Serial.print(errP); Serial.print('\t');
  Serial.print(errR); Serial.print('\t');
  Serial.print(errY); Serial.println();
  delay(10);
}
else if (ara) 
{
    //Serial.print("filte -\t");
  Serial.print(araw[0]); Serial.print('\t');
  Serial.print(araw[1]); Serial.print('\t');
  Serial.print(araw[2]); Serial.println();
  delay(10);
}

else if (fof) 
{
    //Serial.print("filte -\t");
  Serial.print(afil[0] + 2*aoff[0]); Serial.print('\t');
  Serial.print(afil[1] + 2*aoff[1]); Serial.print('\t');
  Serial.print(afil[2] + 2*aoff[2]); Serial.println();
  delay(10);
}

else if (ypr) 
{
    //Serial.print("filte -\t");
  Serial.print(apitch); Serial.print('\t');
  Serial.print(aroll ); Serial.print('\t');
  Serial.print(afil[2]); Serial.println();
  delay(10);
}

else if (rao)
{
    Serial.print(raw[0]);  Serial.print('\t');
    Serial.print(raw[1]);  Serial.print('\t');
    Serial.print(raw[2]);  Serial.println();
    delay(10);
}

else if (mot)
{
    Serial.print(pm1);  Serial.print('\t');
    Serial.print(pm2);  Serial.print('\t');
    Serial.print(pm3);  Serial.print('\t');
    Serial.print(pm4);  Serial.println();
    delay(10);
}

last_print = millis();
 }
//if (!pri && !last_arm && arm ){ if (serial) { Serial.end(); pinMode(LED_PIN, OUTPUT);  serial = false ;} if(!serial) digitalWrite(LED_PIN, LOW);}
//if (!pri && last_arm && !arm ) { digitalWrite(LED_PIN, HIGH); }

//last_arm = arm;
//serverClients[i].write();
if (acal) { Acc_off(false);}


     if ( !debug && arm  &&  !no_rf2 ){ if (millis() - last_led > 100 ) { Lsta = !Lsta; digitalWrite(LED_PIN, Lsta); last_led = millis();} } 
     // armed and with connectivity - 
else if ( !debug && arm  &&   no_rf2 ){ if (millis() - last_led > 500 ) { Lsta = !Lsta; digitalWrite(LED_PIN, Lsta); last_led = millis();} } 
    // armed but lost connectivity - soon going to bedisarmed automatically

else if ( !debug && !arm &&  !no_rf2 && had_wifi){ if (millis() - last_led > 1000) { Lsta = !Lsta; digitalWrite(LED_PIN, Lsta); last_led = millis();} }
    // disarmed and with connectivity

else if ( !debug && !arm &&   no_rf2 ){ if (millis() - last_led > 2000) { Lsta = !Lsta; digitalWrite(LED_PIN, Lsta); last_led = millis();} }
else if ( !debug && !arm && !had_wifi){ if (millis() - last_led > 2000) { Lsta = !Lsta; digitalWrite(LED_PIN, Lsta); last_led = millis();} }
    // disarmed and no Wi-Fi connectivity
}

 //  aAb101c105d108e120f40g50h


//    size_t len = accc.length();
//    uint8_t bac[len];
//    accc.getBytes(bac, len);
//delay(2); serverClients[i].print(accc);delay(2);
//    serverClients[i].print(accb);delay(2);
//    serverClients[i].write(bac,len);delay(2);
//    Serial.write(bac,len);




//       size_t len = accc.length();   uint8_t bac[len];   accc.getBytes(bac, len);
 //  serverClients[i].print(accb);;delay(2);
//    serverClients[i].write(bac,len);delay(2);
//    Serial.write(bac,len);
  // Serial.print(accc);



  //  mod1 = false;     
// if (apitch + dPitch > 10 || aroll + dRoll > 10 || apitch + dPitch < -10 || aroll + dRoll < -10 )
// {
//  mod1 = true;
// }
/*
 else
 {
      /************************************************************/
//      errP  = ( dx )  ; erriP += errP/6 ; errdP -= errP/6 ;
//      errR  = ( dy )  ; erriR += errR/6 ; errdR -= errR/6 ;
//      errY  = (-dz )  ; erriY += errY/6 ; errdY -= errY/6 ;
      /************************************************************/
 //}

     

