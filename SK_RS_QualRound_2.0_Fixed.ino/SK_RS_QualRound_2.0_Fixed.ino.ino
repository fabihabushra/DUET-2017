#define sensorNum 8
#define maxSpeed 200

int blackLimit[sensorNum]={450,450,450,450,450,450,450,450};
const int motorPin1=9,motorPin2=10;        //right motor
const int motorPin3=5,motorPin4=6;       //left motor

int sensorValue = 0;
int error,prevError=0;
int mappedValue,targetValue=0;


float kp=60;
float kd=15;


int motorResponse;
float correction;

int surface=0;

int leftSpeed,rightSpeed;

int allBlack=0;





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

  Serial.begin(9600);
}







void loop()
{

sensorRead();

sensorMapping();

if((mappedValue!=100))
{
 
  pid();

  motor(leftSpeed,rightSpeed);
  /*analogWrite(motorPin1,rightSpeed);
  analogWrite(motorPin2,0);
  analogWrite(motorPin3,leftSpeed);
  analogWrite(motorPin4,0); */
}
else if(mappedValue==100)
{
  
    
      while(mappedValue==100)
      {
        rotate();
        sensorRead();

        sensorMapping();
  
      }
      
      
  
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
else if((mappedValue==111)&&(surface==1))
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
else if((mappedValue==111)&&(surface==0)&&(allBlack==2))  //right turn for qualifying instead of cave
{
  
  mappedValue=60;
  pid();

  motor(leftSpeed,rightSpeed);
  /*analogWrite(motorPin1,rightSpeed);
  analogWrite(motorPin2,0);
  analogWrite(motorPin3,leftSpeed);
  analogWrite(motorPin4,0); */

}



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
    if (analogRead(A7-i) < blackLimit[i]) digitalValue = 1;
    else digitalValue = 0;

    sensorValue |= (digitalValue << (sensorNum-1-i));
  }
  Serial.println(sensorValue,BIN);
  
}


void sensorMapping(void)
{
  if (sensorValue == 0b00000000) mappedValue = 100;
  //else if (sensorValue == 0b11111111) mappedValue = 111;
  

  //detect surface
  if ((sensorValue == 0b11100111)
||(sensorValue == 0b11000111)
||(sensorValue == 0b11100011)
||(sensorValue == 0b10011111)
||(sensorValue == 0b11000111)
||(sensorValue == 0b11110001)
||(sensorValue == 0b11111001)) surface = 1;


  else if((sensorValue == 0b00011000)
||(sensorValue == 0b00111000)
||(sensorValue == 0b00011100)
||(sensorValue == 0b01100000)
||(sensorValue == 0b00111000)
||(sensorValue == 0b00001110)
||(sensorValue == 0b00000110)) surface = 0;

if(surface == 0)
{  

   //Zigzag right turn
   if (sensorValue == 0b00011010) mappedValue = 1050;      //changed from 60 to 70
  else if (sensorValue == 0b00011011) mappedValue = 1050;
  else if (sensorValue == 0b00111011) mappedValue = 1050;
  else if (sensorValue == 0b00011001) mappedValue = 1050;
  else if (sensorValue == 0b00111001) mappedValue = 1050; 

  //90 deg right turn
  else if (sensorValue == 0b00011111) mappedValue = 60;         
  else if (sensorValue == 0b00001111) mappedValue = 60;

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
  else if (sensorValue == 0b00011000) mappedValue = 0;
  else if (sensorValue == 0b11111111) mappedValue = -60;

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
  else if (sensorValue == 0b11111000) mappedValue = -60;    //changed due to track from -60 to 0      
  else if (sensorValue == 0b11110000) mappedValue = -60;

  //Zigzag left turn
  else if (sensorValue == 0b01011000) mappedValue = -1050;   //changed from 0 to -50
  else if (sensorValue == 0b11011000) mappedValue = -1050;
  else if (sensorValue == 0b11011100) mappedValue = -1050;
  else if (sensorValue == 0b10011000) mappedValue = -1050;
  else if (sensorValue == 0b10011100) mappedValue = -1050;

  else if (sensorValue == 0b11111111) allBlack++; 
}
else if(surface == 1)
{  
  //Zigzag right turn
   if (sensorValue == 0b11100101) mappedValue = 90;      //changed from 0 to 50
  else if (sensorValue == 0b11100100) mappedValue = 90; 

  //90 deg right turn
  else if (sensorValue == 0b11100000) mappedValue = 60;         
  
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
  else if (sensorValue == 0b00000111) mappedValue = -60;         
  
  //Zigzag left turn
  else if (sensorValue == 0b10100111) mappedValue = -90;   //changed from -60 to -70
  else if (sensorValue == 0b00100111) mappedValue = -90;
 
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

  if(motorResponse > 0)
  {
    rightSpeed=maxSpeed; 
    leftSpeed=maxSpeed-motorResponse;;
  }
  else
  {
    rightSpeed=maxSpeed+ motorResponse;
    leftSpeed=maxSpeed;
  }

}

void motor(int left, int right)
{
  analogWrite(motorPin1,right);
  analogWrite(motorPin2,0);
  analogWrite(motorPin3,left);
  analogWrite(motorPin4,0);

}

void rotate(void)
{
  analogWrite(motorPin1,maxSpeed);
  analogWrite(motorPin2,0);
  analogWrite(motorPin3,0);
  analogWrite(motorPin4,maxSpeed);
  
}

void goForward()
{
  analogWrite(motorPin1,maxSpeed);
  analogWrite(motorPin2,0);
  analogWrite(motorPin3,maxSpeed);
  analogWrite(motorPin4,0);
  
}

