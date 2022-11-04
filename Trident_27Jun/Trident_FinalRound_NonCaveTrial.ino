
#include <PulseInZero.h>


const float SpeedOfSound       = 343.2; // ~speed of sound (m/s) in air, at 20°C         
const float  MicrosecondsToMillimetres2  = (SpeedOfSound/ 1000.0 )/2;

const int SIGNAL_PIN  = 11;   // digital pin connected to the trigger port on the module 
                                // connect the echo port to the pin for interrupt 0 (pin 2 on Uno) 

// 
unsigned long lastTime  = 0;

int pingTimer     = 0;
int pingDelay     = 500; // milliseconds between ping pulses
int fDistance = 21;

//nokol^

#define sensorNum 8
#define maxSpeed 255

int blackLimit[sensorNum];
float safety=0.35;
int time=10;

const int motorPin1 = 9,motorPin2 = 10;        //right motor
const int motorPin3 = 5,motorPin4 = 6;       //left motor

int sensorValue = 0;
int error, prevError = 0;
int mappedValue, targetValue;

float kp = 4;
float kd = 0;
float ki;

int motorResponse;
float correction;

int surface = 0; //defualt 9 is set to black on white

int leftSpeed, rightSpeed;

//trigger variables
int surfChange = 0; //detects surface change in order to change all black decisions 

int acuteKey = 0; //prepares for first acute angle rotate

int obsKey = 0;

int stopKey = 0; //prepares to stop at endpoint



int danceDelay = 480;
int blackDelay = 500;
int prev, curr, diff;






void setup()
{

  pinMode(SIGNAL_PIN, OUTPUT);
  digitalWrite(SIGNAL_PIN, LOW); 

  //initialize IR pins
  for(int i = 0; i < sensorNum; i++)
  {
    pinMode(A0 + i, INPUT);
  }

  //initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);



  //3sec Delay before auto calibration starts
  delay(3000);
  calibration();

  PulseInZero::setup(pingPulseComplete);

  Serial.begin(9600);
}







void loop()
{

    unsigned long time = millis();
  unsigned long dt   = time - lastTime;
  lastTime       = time;
  
  pingTimer += dt;
  if(pingTimer > pingDelay)
  {  
  pingTimer = 0;
  ping();
  }
  //doing stuff


sensorRead();

//mapReading for PID
sensorMapping();



if  (fDistance < 20) 
{ 
  
    if(obsKey == 0)
    {
   //resets acuteKey variables to reverse false triggers
   curveRun();
   //prepare for first acute turn
   acuteKey = 1;
   //stops sonar trigger
   //surfChange = 0;

   leftKey = 1;
  }

  else if(obsKey == 1)
  {
     brake();
     //scan until traffic is openned
     while( fDistance < 20 );

     stopKey = 1;

    //prepare to stop at endpoint 
  }

 }



}

//reactions for normal line
if((mappedValue != 100) && (mappedValue != 111))
{
 
  pid();

  motor(leftSpeed, rightSpeed);

}

else if(mappedValue == 100)
{
   //Reaction for all-white 
   if(acuteKey == 1)
    {  
      dance();

      obsKey = 1;

    }

    else
    {
        findPath();
    }

}

else if((mappedValue == 111)&&(surface == 1))
{     
      //keep turning if line missed after a right angle turn
        findPath();
}

//stops at endpoint
else if((mappedValue == 111)&&(surface == 0)&&(stopKey == 1))
{
    delay(300);
    brake();
}


 else if((mappedValue == 111)&&(surface == 0))
 {

  mappedValue = 0;

  pid();
  motor(leftSpeed, rightSpeed);

 }

}


void sensorRead(void)
{
  int digitalValue;
  sensorValue = 0;
    
  for (int i= 0; i< sensorNum; i++)
  { 
    if (analogRead(A7-i) < blackLimit[i]) digitalValue = 1;  //A7 is leftmost IR
    else digitalValue = 0;

    sensorValue |= (digitalValue << (sensorNum-1-i));
  }
  Serial.println(sensorValue,BIN);
  
}

void sensorMapping(void)
{
  if (sensorValue == 0b00000000) {mappedValue = 100; return;}
  else if (sensorValue == 0b11111111) {mappedValue = 111; return;}
  

  //detect surface
  if ((sensorValue == 0b11100111)
||(sensorValue == 0b11000111)
||(sensorValue == 0b11100011)
||(sensorValue == 0b10011111)
||(sensorValue == 0b11000111)
||(sensorValue == 0b11110001)
||(sensorValue == 0b11111001)) 
    {
      surface = 1;
\
      surfChange=1;
    }

  else if((sensorValue == 0b00011000)
||(sensorValue == 0b00111000)
||(sensorValue == 0b00011100)
||(sensorValue == 0b01100000)
||(sensorValue == 0b00111000)
||(sensorValue == 0b00001110)
||(sensorValue == 0b00000110)) 
    {
      surface = 0;
      
    }

if(surface == 0)
{  
 

  //90 deg right turn
  if (sensorValue == 0b00011111 || sensorValue == 0b00001111 || sensorValue == 0b00111111) mappedValue = 80;         
 

  //line at right
  else if (sensorValue == 0b00000001) mappedValue = 60;
  else if (sensorValue == 0b00000011) mappedValue = 50;
  else if (sensorValue == 0b00000010) mappedValue = 40;
  else if (sensorValue == 0b00000111) mappedValue = 40;
  else if (sensorValue == 0b00000110) mappedValue = 30;
  else if (sensorValue == 0b00000100) mappedValue = 30;
  else if (sensorValue == 0b00001110) mappedValue = 30;
  else if (sensorValue == 0b00001100) mappedValue = 20;
  else if (sensorValue == 0b00001000) mappedValue = 20;
  else if (sensorValue == 0b00011100) mappedValue = 10;

  //line at left
  else if (sensorValue == 0b10000000) mappedValue = -60;
  else if (sensorValue == 0b11000000) mappedValue = -50;
  else if (sensorValue == 0b01000000) mappedValue = -40;
  else if (sensorValue == 0b11100000) mappedValue = -40;
  else if (sensorValue == 0b01100000) mappedValue = -30;
  else if (sensorValue == 0b00100000) mappedValue = -30;
  else if (sensorValue == 0b01110000) mappedValue = -30;
  else if (sensorValue == 0b00110000) mappedValue = -20;
  else if (sensorValue == 0b00010000) mappedValue = -20;
  else if (sensorValue == 0b00111000) mappedValue = -10;
 
  //90 deg left turn
  else if (sensorValue == 0b11111000 || sensorValue == 0b11110000)
  {
    mappedValue = 0; //changed due to track from -60 to 0  
    acuteKey=1;
  }    

  //line at middle
  else if (sensorValue == 0b00011000) 
  { 
    mappedValue = 0;        
  }

      

}

else if(surface == 1)
{  

  //90 deg right turn
  if (sensorValue == 0b11100000 || sensorValue == 0b11110000) { mappedValue = 80; plannedCRotate(); delay(blackDelay);
int intersectionDelay = 200;);}         
  
  //line at right
  else if (sensorValue == 0b11111110) mappedValue = 60;
  else if (sensorValue == 0b11111100) mappedValue = 50;
  else if (sensorValue == 0b11111101) mappedValue = 40;
  else if (sensorValue == 0b11111000) mappedValue = 40;
  else if (sensorValue == 0b11111001) mappedValue = 30;
  else if (sensorValue == 0b11111011) mappedValue = 30;
  else if (sensorValue == 0b11110001) mappedValue = 30;
  else if (sensorValue == 0b11110011) mappedValue = 20;
  else if (sensorValue == 0b11110111) mappedValue = 20;
  else if (sensorValue == 0b11100011) mappedValue = 10;

    //line at left
  else if (sensorValue == 0b01111111) mappedValue = -60;
  else if (sensorValue == 0b00111111) mappedValue = -50;
  else if (sensorValue == 0b10111111) mappedValue = -40;
  else if (sensorValue == 0b00011111) mappedValue = -40;
  else if (sensorValue == 0b10011111) mappedValue = -30;
  else if (sensorValue == 0b11011111) mappedValue = -30;
  else if (sensorValue == 0b10001111) mappedValue = -30;
  else if (sensorValue == 0b11001111) mappedValue = -20;
  else if (sensorValue == 0b11101111) mappedValue = -20;
  else if (sensorValue == 0b11000111) mappedValue = -10;
 
  //90 deg left turn
  else if (sensorValue == 0b00000111 || sensorValue == 0b00001111) {mappedValue = -80; plannedACRotate(); delay(blackDelay);}   

  //line at middle
  else if (sensorValue == 0b11100111) mappedValue = 0;

 
 }

}

void pid()
{
  
  error = targetValue - mappedValue;
  correction = (kp * error) + (kd * (error - prevError));

  prevError = error;

  motorResponse = (int)correction;

  if(motorResponse > maxSpeed) motorResponse = maxSpeed;
  if(motorResponse < -maxSpeed) motorResponse = -maxSpeed;

  if(motorResponse > 0)
  {
    rightSpeed = maxSpeed ;
    leftSpeed = maxSpeed - motorResponse;
  }

  else
  {
    rightSpeed = maxSpeed + motorResponse;
    leftSpeed = maxSpeed;
  }

}

//writes motor speed
void motor(int left, int right)
{
  analogWrite(motorPin1, right);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, left);
  analogWrite(motorPin4, 0);

}

//stops the bot
void brake(void)
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 0);
}



//special set of motor functions

void plannedForward()
{
  analogWrite(motorPin1, 90);
  analogWrite(motorPin2 , 0);
  analogWrite(motorPin3, 90);
  analogWrite(motorPin4, 0);

}


void plannedACRotate()
{
  analogWrite(motorPin1,100);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4,100);

}

void plannedCRotate()
{
  analogWrite(motorPin1,0);
  analogWrite(motorPin2, 100);
  analogWrite(motorPin3, 100);
  analogWrite(motorPin4,0);

}

//avoid obstacles
void curveRun()
{
Serial.println("I am curveRunning");
plannedCRotate();
delay(1000);
plannedForward();
delay(3000);
plannedACRotate();
delay(700);
plannedForward();
delay(3700);
plannedACRotate();
delay(700);

//go forward while still on all white
do
{
 sensorRead();
 sensorMapping();    

 plannedForward();
} while(mappedValue == 100);

//make sure bot aligns back to the line
plannedCRotate();
delay(200);

}

//auto calibration
void calibration()
{
  plannedCRotate();

  float upSum = 0,lowSum = 0;
  int sensorArray[sensorNum][2];

  for(int i = 0; i < sensorNum; i++)
    {
      sensorArray[i][0] = analogRead(A7-i);
      sensorArray[i][1] = analogRead(A7-i);
    }
 

  int loopCounter = (int)(time * 1000 / 2.5);  
  while(loopCounter)
  {
    for(int i = 0; i < sensorNum; i++)
    {
      if(analogRead(A7-i)<sensorArray[i][0]) sensorArray[i][0]=analogRead(A7-i);
      if(analogRead(A7-i)>sensorArray[i][1]) sensorArray[i][1]=analogRead(A7-i);
    }
  loopCounter--;
  }

 for(int i=0; i < sensorNum; i++)
  blackLimit[i] = (int)(sensorArray[i][0] + safety * (sensorArray[i][1] - sensorArray[i][0]));

prev = millis();
//take sensor reading
sensorRead();

//mapReading for PID
sensorMapping();

curr = millis();

diff = curr - prev;

  brake();
  delay(1000);

}

//reset acuteKey variables
void resetAcuteKey(void)
{
  acuteKey = 0;
  
  stopKey = 0;  
}

void findPath(void)
{
       if(prevError != 0)
       {
        motor(rightSpeed, leftSpeed);
       }

}

//acute angle movement
void dance(void)
{

int loopCounter = (int) (danceDelay / diff);

  for(int i = loopCounter; i > 0; i--)
  {

  plannedCRotate();
  sensorRead();
  sensorMapping();

   if(mappedValue != 100)
   {
    pid();
    motor(leftSpeed, rightSpeed);
    return;
   }

  }

  for(int i = 2 * loopCounter; i > 0; i--)
  {
  plannedACRotate();
  sensorRead();
  sensorMapping();

   if(mappedValue != 100)
   {
    pid();
    motor(leftSpeed, rightSpeed);
    return;
   }

  }
  
}

//Allah maaf koruk

void ping(){

 // Serial.println("ping out");
  
  digitalWrite(SIGNAL_PIN, HIGH);
  delayMicroseconds(10); // I think I can cope with blocking for a whole 10us here...
  digitalWrite(SIGNAL_PIN, LOW);
  
  // start listening out for the echo pulse on interrupt 0
  PulseInZero::begin();
}


/**
* Pulse complete callback hanlder for PulseInZero 
* @param duration - pulse length in microseconds
*/
void pingPulseComplete(unsigned long duration)
{

  fDistance = MicrosecondsToMillimetres2 * duration/ 10;
  
  
}