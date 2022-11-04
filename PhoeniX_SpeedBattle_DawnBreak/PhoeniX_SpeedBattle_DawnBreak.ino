#define sensorNum 8
#define maxSpeed 200
int x=500;

 int blackLimit[sensorNum]={x,x,x,x,x,x,x,x};

const int motorPin1=10,motorPin2=9;        //right motor
const int motorPin3=6,motorPin4=5;       //left motor

float error, prevError=0;

float mappedValue, targetValue = 7;      //changed ferom 4.5 to 9

float safety=0.35;

float kp=48.5;                             //45 IF DOESN'T WORK
float kd=150 ;
float kt;                               //turn constant

int motorResponse;
float correction;

int leftSpeed,rightSpeed;
int digitalValue;

float lastAct;
int time=10;




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
 // calibration();
  
  Serial.begin(9600);
}







void loop()
{
 
//sensorRead();

sensorMapping();

for(int i = 0; i < 1; i++)
{
  Serial.print(analogRead(A0+i));
  Serial.print(" ");
  Serial.print(blackLimit[i]);
  Serial.print(" ");
 if(analogRead(A0+i)<blackLimit[i])
     Serial.print("B");
  else
     Serial.print("W");   
  Serial.print(" ");
}
Serial.println(" ");

  if(mappedValue!=0)
    {
      pid();
      motor(leftSpeed,rightSpeed);
      
      lastAct = mappedValue;
    }
  else
    {
     
      if(lastAct<targetValue) 
      {
      if(rightSpeed>leftSpeed)
       motor(leftSpeed,rightSpeed);        
      else
      motor(rightSpeed,leftSpeed); 
      }
      else  
      {
      if(leftSpeed>rightSpeed)
       motor(leftSpeed,rightSpeed);        
      else
       motor(rightSpeed,leftSpeed);
       }

      /*
      //use lastAct if this doesnt work
      if(mappedValue!=lastAct)
      {  
      if(rightSpeed>leftSpeed) 
            //motor(leftSpeed,rightSpeed);              //goes right
    
    /*  else if(rightSpeed<leftSpeed)
            motor(-rightSpeed,leftSpeed);               //goes left
      }
      else  if(lastAct<9) motor(-rightSpeed,leftSpeed);             //last line position was at left 
      else  motor(rightSpeed,-leftSpeed);                             
      */
    }

}


void sensorMapping()
{
int sum=0,coun=0;
 
 for (int i = 0; i <sensorNum; i++)
  { 
    
    if (analogRead(A0+i) < blackLimit[i])              //A7 is leftmost IR 
     { 
      
      sum += i*2;
      coun++;
    }
    

  }
   if(coun!=0){  
  mappedValue = sum / coun;
   }
   else mappedValue=0;
   /*
  Serial.print("\tmappedValue: ");
  Serial.println(mappedValue);
  */
}


void pid()
{
  
  error=targetValue-mappedValue;
  correction=(kp*error)+(kd*(error-prevError));
  prevError=error;
  motorResponse=(int)correction;
 
 if(motorResponse>maxSpeed) motorResponse=maxSpeed;
 
if(motorResponse<-maxSpeed) motorResponse=-maxSpeed;

   if(motorResponse>0)
  {
    rightSpeed=maxSpeed;
    leftSpeed=maxSpeed-motorResponse;
  }
  else 
  {
    rightSpeed=maxSpeed+ motorResponse;
    leftSpeed=maxSpeed;
  }

}

void motor(int left, int right)
{
  
  if(right>0)
  {
  analogWrite(motorPin1,right);
  analogWrite(motorPin2,0);
  }
  else
  {
    analogWrite(motorPin1,0);
    analogWrite(motorPin2,-right);
  }

  if(left>0)
  {
  analogWrite(motorPin3,left);
  analogWrite(motorPin4,0);
  }
  else
  {
   analogWrite(motorPin3,0);
   analogWrite(motorPin4,-left); 
  }

 }

void plannedCRotate()
{
  analogWrite(motorPin1,0);
  analogWrite(motorPin2, 50);
  analogWrite(motorPin3, 50);
  analogWrite(motorPin4,0);

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
      sensorArray[i][0] = analogRead(A0+i);
      sensorArray[i][1] = analogRead(A0+i);
    }
 

  int loopCounter = (int)(time * 1000 / 2.5);  
  while(loopCounter)
  {
    for(int i = 0; i < sensorNum; i++)
    {
      if(analogRead(A0+i)<sensorArray[i][0]) sensorArray[i][0]=analogRead(A0+i);
      if(analogRead(A0+i)>sensorArray[i][1]) sensorArray[i][1]=analogRead(A0+i);
    }
  loopCounter--;

  }

 for(int i=0; i < sensorNum; i++)
  blackLimit[i] = (int)(sensorArray[i][0] + safety * (sensorArray[i][1] - sensorArray[i][0]));

  brake();
  delay(1000);

}

