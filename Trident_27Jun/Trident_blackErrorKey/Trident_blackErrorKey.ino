#define sensorNum 8
#define maxSpeed 200

int blackLimit[sensorNum];
float safety=0.35;
int time=1;

const int motorPin1 = 9,motorPin2 = 10;        //right motor
const int motorPin3 = 5,motorPin4 = 6;       //left motor

int sensorValue = 0;
int error, prevError = 0;
int mappedValue, targetValue=0;


//PID constants
float kp = 4;
float kd = 0;
float ki;

int motorResponse;
float correction;

int surface = 0; //defualt surface is set to black on white

int leftSpeed, rightSpeed;

//trigger variables
int surfChange = 0; //detects surface change in order to change all black decisions 

int acuteKey = 0; //prepares for first acute angle rotate
int acuteKeyTwo = 0; //prepares for 2nd acute angle rotate

int obsKey = 0; //differentiates between obstacle and traffic

int crotateKey = 0; //registers successful first acute angle rotate
int acrotateKey = 0; //registers successful 2nd acute angle rotate

int stopKey = 0; //prepares to stop at endpoint

int blackSharpKey = 0; //prepares to acrotate at first sharp turn
int blackSharpKeyTwo = 0; //prepares to crotate at 2nd sharp turn

int blackacrotateKey = 0; //registers successful first sharp turn

int blackPrev, blackCurr, blackDiff;

int blackTime = 2000;

int acuteCPrev, acuteACPrev, acuteCCurr, acuteACCurr, acuteCDiff, acuteACDiff;

int acuteTime = 500;





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

  //3sec Delay before auto calibration starts
  delay(3000);
  calibration();

  Serial.begin(9600);
}







void loop()
{

//take sensor reading
sensorRead();

//mapReading for PID
sensorMapping();

//search for obstacle or traffic
takeSonarReadings();


while(fDistance <20) 
{ 
  //if(surfChange == 1) {surfChange = 0;}
  if(obsKey == 1)
  {
     brake();
     //scan until traffic is openned
     while( fDistance <20)
        fDistance= trigger(fSonarTrig,fSonarEcho);

    //prepare to stop at endpoint 
     stopKey = 1; 
  }

  
  else
  {
   //resets acuteKey variables to reverse false triggers
   curveRun();
   resetAcuteKey();
   //prepare for first acute turn
   acuteKey = 1;
   fDistance= trigger(fSonarTrig,fSonarEcho);
  }

Serial.print("Front: ");
  Serial.println(fDistance);

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
      plannedCRotate();
      acuteCPrev = millis();
      //successful cRotate
      crotateKey = 1;
    }

    else if(acuteKeyTwo == 1)
    {
      plannedACRotate();
      acuteACPrev = millis();
      //successful acRotate
      acrotateKey = 1;
    }

  
      else
        findPath();
    
}

else if((mappedValue == 111)&&(surface == 1))
{     
      //keep turning if line missed after a right angle turn
      if(blackSharpKey == 1)
      {
        plannedACRotate();
        blackacrotateKey = 1;
        blackPrev = millis();
      }

      else if(blackSharpKeyTwo == 1)
      {
        plannedCRotate();
      }

      //else
        //findPath();
}

else if((mappedValue == 111)&&(surface == 0)&&(stopKey == 1))
{
    brake();
}

//plus intersection decision for Qualification round
else if((mappedValue == 111)&&(surface == 0)&&(surfChange == 1))  
{

  resetAcuteKey();

  
//plus intersection rotate
  plannedCRotate();
        delay(1000);

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

    digitalValue ? Serial.print("B") : Serial.print("W");

    sensorValue |= (digitalValue << (sensorNum-1-i));
  }
  //Serial.println(sensorValue,BIN);
  Serial.println();
  
}

void sensorMapping(void)
{
  if (sensorValue == 0b00000000) {mappedValue = 100; return;}
  else if (sensorValue == 0b11111111) {mappedValue = 111; return;}
  
      //reverse acute angle turn after succesful rotate
      if(acuteKey == 1 && crotateKey == 1)
      { 
      acuteCCurr = millis();
      acuteCDiff = acuteCCurr - acuteCPrev;
      }
      else if(acuteKeyTwo == 1 && acrotateKey == 1)
      {
      acuteACCurr = millis();
      acuteACDiff = acuteACCurr - acuteACPrev;
      }
      
      if(acuteKey == 1 && crotateKey == 1 && acuteCDiff > acuteTime) 
      {
        acuteKeyTwo = 1; 
        acuteKey = 0;
        acrotateKey = 0;
      }


      //Differentiate obstacle and traffic using 2nd acute angle turn
      else if(acuteKeyTwo == 1 && acrotateKey == 1 && acuteACDiff > acuteTime) 
      {
        acuteKey = 1; 
        acuteKeyTwo = 0;
        crotateKey = 0; 
        obsKey = 1;
      }    

  //detect surface
  if ((sensorValue == 0b11100111)
||(sensorValue == 0b11000111)
||(sensorValue == 0b11100011)
||(sensorValue == 0b10011111)
||(sensorValue == 0b10001111)
||(sensorValue == 0b11110001)
||(sensorValue == 0b11111001)) 
    {
      surface = 1;
      surfChange=1;
      if(blackSharpKeyTwo == 0) blackSharpKey = 1;
    }

  else if((sensorValue == 0b00011000)
||(sensorValue == 0b00111000)
||(sensorValue == 0b00011100)
||(sensorValue == 0b01100000)
||(sensorValue == 0b01110000)
||(sensorValue == 0b00001110)
||(sensorValue == 0b00000110)) 
    {
      surface = 0;
    }

if(surface == 0)
{  
 

  //90 deg right turn
  if (sensorValue == 0b00011111 || sensorValue == 0b00001111 ) mappedValue = 80;         
 

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

    resetAcuteKey();

   //prepare for first acute turn
   acuteKey = 1;
  }    

  //line at middle
  else if (sensorValue == 0b00011000) 
  { 
    mappedValue = 0;        
  }

      

}

else if(surface == 1)
{  
  blackCurr = millis();
  blackDiff = blackCurr - blackPrev;

  if(blackacrotateKey == 1 && blackSharpKey == 1)
  {
    blackSharpKey = 0;
    if(blackDiff > blackTime)
    blackSharpKeyTwo = 1;
  }

  //90 deg right turn
  if (sensorValue == 0b11100000 || sensorValue == 0b11110000) mappedValue = 80;         
  
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
  else if (sensorValue == 0b00000111 || sensorValue == 0b00001111) mappedValue = -80;   

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


void takeSonarReadings()
{
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


//avoid obstacles
void curveRun()
{

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

  brake();
    delay(1000);

}

//reset acuteKey variables
void resetAcuteKey(void)
{
  acuteKey = 0;
  acuteKeyTwo = 0;
  crotateKey = 0;
  acrotateKey = 0;
  obsKey = 0;
  stopKey = 0;  
}

void findPath(void)
{
        if(prevError < 0)
      {
        //go left if line missed due to right turn
        motor(rightSpeed, maxSpeed);
      }

      else if(prevError > 0) 
      {
        //go right if line
        motor(maxSpeed, leftSpeed);
      }


}
