#define sensorNum 8
#define maxSpeed 150
int blackLimit[sensorNum];
float safety=0.35;
int time=10;
const int motorPin1 = 9,motorPin2 = 10;        //right motor
const int motorPin3 = 5,motorPin4 = 6;       //left motor

int sensorValue = 0;
int error, prevError = 0;
int mappedValue, targetValue=0;

int prevMappedValue = 0;

float kp = 4.7;
float kd = 2.5;
float ki;
int motorResponse;
float correction;

int surface = 0;

int leftSpeed, rightSpeed;

//trigger variables
int surfChange = 0;

int acuteKey = 0;
int acuteKeyTwo = 1;

int obsKey = 0;

int crotateKey = 0;
int acrotateKey = 0;

int stopKey = 0;

int resetKey = 0;


//Sonar variables
const int fSonarTrig = 11;
const int fSonarEcho = 12;
const int rSonarTrig = 8;
const int rSonarEcho = 7;
const int lSonarTrig = 4;
const int lSonarEcho = 3;


int fDistance;
int rDistance;
int lDistance;
int dDistance;
int difference;

int max=50,min;


void setup()
{
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


//initialize SonarPins
  pinMode(fSonarTrig, OUTPUT);
  pinMode(fSonarEcho, INPUT);
  pinMode(rSonarTrig, OUTPUT);
  pinMode(rSonarEcho, INPUT);
  pinMode(lSonarTrig, OUTPUT);
  pinMode(lSonarEcho, INPUT);


  delay(3000);
  calibration();
  Serial.begin(9600);
}







void loop()
{

sensorRead();

sensorMapping();

//obstacle code
takeSonarReadings();


while(fDistance <20) 
{ if(surfChange == 1) {surfChange = 0;}
  if(obsKey == 1)
  {
     brake();
     while( fDistance <20)
        fDistance= trigger(fSonarTrig,fSonarEcho);
     stopKey = 1; 
  }

  
  else
  {
  curveRun();
  fDistance= trigger(fSonarTrig,fSonarEcho);
  }

Serial.print("Front: ");
  Serial.println(fDistance);

  Serial.print("Down: ");
  Serial.println(dDistance);


}

if((mappedValue != 100) && (mappedValue != 111))
{
 
  pid();

  motor(leftSpeed, rightSpeed);
  /*analogWrite(motorPin1,rightSpeed);
  analogWrite(motorPin2,0);
  analogWrite(motorPin3,leftSpeed);
  analogWrite(motorPin4,0); */
}
else if(mappedValue == 100)
{
   //Reaction for all-white 
   if(acuteKey == 1)
    {  plannedCRotate();
       crotateKey = 1;
         /*
         mappedValue = 70;
         pid();
         motor(leftSpeed, rightSpeed);
         */
    }

    else if(acuteKeyTwo == 1)
    {
         plannedACRotate();
         acrotateKey = 1;
            /*
         mappedValue = -70;
         pid();
         motor(leftSpeed, rightSpeed);
           */
    }
    else
    {

         if(prevError<0)
              {
   
                  motor(maxSpeed,leftSpeed);
                  /*
                  analogWrite(motorPin1,leftSpeed);
                  analogWrite(motorPin2,0);
                  
                  analogWrite(motorPin3,maxSpeed);
                  analogWrite(motorPin4,0);     //go right
                  */
              }
          else 
              {
   
                 motor(rightSpeed,maxSpeed);
                  /*
                  analogWrite(motorPin1,maxSpeed);
                  analogWrite(motorPin2,0);
   
                  analogWrite(motorPin3,rightSpeed);
                  analogWrite(motorPin4,0); //go left
                   */
              }
    }

}
else if((mappedValue == 111)&&(surface == 1))
{
  if(prevError<0)
  {
   
   motor(maxSpeed,leftSpeed);
   /*
   analogWrite(motorPin1,leftSpeed);
   analogWrite(motorPin2,0);
   
   analogWrite(motorPin3,maxSpeed);
   analogWrite(motorPin4,0);     //go right
  */
  }
  else 
  {
   
   motor(rightSpeed,maxSpeed);
   /*
   analogWrite(motorPin1,maxSpeed);
   analogWrite(motorPin2,0);
   
   analogWrite(motorPin3,rightSpeed);
   analogWrite(motorPin4,0); //go left
  */
  }
     
}

else if((mappedValue == 111)&&(surface == 0))
{
  if(stopKey == 1)
  {
    brake();
  }
}


else if((mappedValue == 111)&&(surface == 0)&&(surfChange == 1))  
{

  mappedValue = 60;
  pid();

  motor(leftSpeed,rightSpeed);

  if(resetKey == 0)
  {
    acuteKey = 0;
    acuteKeyTwo = 0;
    obsKey = 0;
    resetKey = 1;
  }

}



/*
if(mappedValue == 111)
{
  blackCount++;
}
else
{
  blackCount = 0;
}
*/
if(mappedValue != 100)
prevMappedValue=mappedValue;

}


void calibrate()
{

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
  //Serial.println(sensorValue,BIN);
  
}

void sensorMapping(void)
{
  if (sensorValue == 0b00000000) {mappedValue = 100; return;}
  else if (sensorValue == 0b11111111) {mappedValue = 111; return;}
  
      //reverse acute angle turn after succesful rotate
      if(acuteKey == 1 && crotateKey == 1) 
      {
        acuteKeyTwo = 1; 
        acuteKey = 0;
      }
      //Differentiate obstacle and traffic using 2nd acute angle turn
      else if(acuteKeyTwo == 1 && acrotateKey == 1) 
      {
        acuteKey = 1; 
        acuteKeyTwo = 0; 
        obsKey = 1;
      }    

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

   //Zigzag right turn
   if (sensorValue == 0b00011010 || sensorValue == 0b00011011 || sensorValue == 0b00111011 || sensorValue == 0b00111010 ) {mappedValue = 70;}       //changed from 60 to 70 

  //90 deg right turn
  else if (sensorValue == 0b00011111 || sensorValue == 0b00001111 || sensorValue == 0b00111111) mappedValue = 80;         
 

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

  //line at middle
  else if (sensorValue == 0b00011000) 
    { 
      mappedValue = 0;        
    }

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
  else if (sensorValue == 0b11111000 || sensorValue == 0b11110000) {mappedValue = 0; acuteKey = 1;}    //changed due to track from -60 to 0      


  //Zigzag left turn
  else if (sensorValue == 0b01011000 || sensorValue == 0b11011000 || sensorValue == 0b11011100 || sensorValue == 0b01011100 ) {mappedValue = -70;} 



}
else if(surface == 1)
{  
  //Zigzag right turn
   if (sensorValue == 0b11100101) mappedValue = 70;      //changed from 0 to 50
  else if (sensorValue == 0b11100100) mappedValue = 70; 

  //90 deg right turn
  else if (sensorValue == 0b11100000) mappedValue = 80;  //changed from 60 to 80 due to line miss 
  
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

  //line at middle
  else if (sensorValue == 0b11100111) mappedValue = 0;

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
  else if (sensorValue == 0b00000111) mappedValue = -80;         
  
  //Zigzag left turn
  else if (sensorValue == 0b10100111) mappedValue = -70;   //changed from 60 to 80 due to line miss
  else if (sensorValue == 0b00100111) mappedValue = -70;
 
}

}

void pid()
{
  
  error=targetValue-mappedValue;
  correction=(kp*error)+(kd*(error-prevError));
  prevError=error;
  motorResponse=(int)correction;
  if(motorResponse>maxSpeed) motorResponse= maxSpeed;
  if(motorResponse<-maxSpeed) motorResponse= -maxSpeed;

  if(motorResponse>0)
  {
    rightSpeed = maxSpeed ;
    leftSpeed = maxSpeed-motorResponse;
  }
  else
  {
    rightSpeed = maxSpeed + motorResponse;
    leftSpeed = maxSpeed;
  }

}

void motor(int left, int right)
{
  analogWrite(motorPin1,right);
  analogWrite(motorPin2,0);
  analogWrite(motorPin3,left);
  analogWrite(motorPin4,0);

}

void brake(void)
{
  analogWrite(motorPin1,0);
  analogWrite(motorPin2,0);
  analogWrite(motorPin3,0);
  analogWrite(motorPin4,0);
}

//Sonar functions
long mstocm(long microseconds)
{
 
  return (microseconds*346.3)/2/10000;
}

int trigger(int trigPin,int echopin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int distance =  mstocm(pulseIn(echopin, HIGH));
  return distance;
}


void takeSonarReadings(){
  fDistance= trigger(fSonarTrig,fSonarEcho);
   rDistance= trigger(rSonarTrig,rSonarEcho);
   lDistance= trigger(lSonarTrig,lSonarEcho);
   
   
   
 difference  = rDistance - lDistance;
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

void plannedLeft()
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 40);
  analogWrite(motorPin3, 100);
  analogWrite(motorPin4, 0);

}

void curveRun() {
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

do{
    sensorRead();
sensorMapping();    

plannedForward();
} while(mappedValue==100);
}

//auto calibration
void calibration()
{
  float upSum=0,lowSum=0;
  int sensorArray[sensorNum][2];

  for(int i=0;i<sensorNum;i++)
    {
      sensorArray[i][0]=analogRead(A7-i);
      sensorArray[i][1]=analogRead(A7-i);
    }
 

  int loopCounter=(int)(time*1000/2.5);  
  while(loopCounter)
  {
    for(int i=0;i<sensorNum;i++)
    {
      if(analogRead(A7-i)<sensorArray[i][0]) sensorArray[i][0]=analogRead(A7-i);
      if(analogRead(A7-i)>sensorArray[i][1]) sensorArray[i][1]=analogRead(A7-i);
    }
  loopCounter--;
  }

 for(int i=0;i<sensorNum;i++)
  blackLimit[i]=(int)(sensorArray[i][0]+safety*(sensorArray[i][1]-sensorArray[i][0]));

}


