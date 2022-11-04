#define sensorNum 8
#define maxSpeed 255

int blackLimit[sensorNum];

const int motorPin1 = 9, motorPin2 = 10;        //right motor
const int motorPin3 = 5, motorPin4 = 6;       //left motor

float error, prevError=0;

float mappedValue, targetValue = 7;

float safety = 0.35;

float kp = 80;
float kd = 0;
float kt;                               //turn constant

int motorResponse;
float correction;

int leftSpeed,rightSpeed;
int digitalValue;

float lastAct;
int time = 10;




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
  delay(1000);
calibration();
  
	Serial.begin(9600);
}







void loop()
{

//sensorRead();

sensorMapping();

  if(mappedValue != 0)
    {
      pid();
      motor(leftSpeed,rightSpeed);
      
      lastAct = mappedValue;
    }
  else
    {
      //use lastAct if this doesnt work
      //if(mappedValue != lastAct)
      //{  
      if(rightSpeed > leftSpeed)
            motor(rightSpeed, -leftSpeed);               //goes right
      else if(rightSpeed < leftSpeed)
            motor(-rightSpeed, leftSpeed);               //goes left
      //}
      //else if(lastAct < 3.5) motor(-rightSpeed, leftSpeed);             //last line position was at left 
      //else motor(rightSpeed, -leftSpeed);                             

    }

}


void sensorMapping()
{
int sum=0,coun=0;
 
 for (int i = 0; i < sensorNum; i++)
  { 
    if (analogRead(A7-i) < blackLimit[i])              //A7 is leftmost IR 
     { 

      sum += i * 2;
      coun++;
    }


  }
   if(coun != 0){  
  mappedValue = sum / coun;
  sum = 0; //useless line
   }
  Serial.print("\tmappedValue: ");
  Serial.println(mappedValue);
}


void pid()
{
  
  error = targetValue - mappedValue;
  correction= (kp * error) + (kd * (error - prevError));
  prevError = error;
  motorResponse = (int)correction;
 
 /*if(motorResponse>maxSpeed) motorResponse=maxSpeed;
 
if(motorResponse<-maxSpeed) motorResponse=-maxSpeed;
*/
   if(motorResponse > 0)
  {
    rightSpeed = maxSpeed;
    leftSpeed = maxSpeed - motorResponse;
  }
  else 
  {
    rightSpeed = maxSpeed + motorResponse;
    leftSpeed = maxSpeed;
  }

}

void motor(int left, int right)
{
  
  if(right > 0)
  {
  analogWrite(motorPin1, right);
  analogWrite(motorPin2, 0);
  }
  else
  {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, -right);
  }

  if(left > 0)
  {
  analogWrite(motorPin3, left);
  analogWrite(motorPin4, 0);
  }
  else
  {
   analogWrite(motorPin3, 0);
   analogWrite(motorPin4, -left); 
  }

 }

void plannedCRotate()
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 100);
  analogWrite(motorPin3, 100);
  analogWrite(motorPin4, 0);

}

void brake(void)
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 0);
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
