//motorPins
int motorPin1 = 5;
int motorPin2 = 6;
int motorPin3 = 9;
int motorPin4 = 10;

const int fSonarTrig = 11;
const int fSonarEcho = 12;
const int rSonarTrig = 8;
const int rSonarEcho = 7;
const int lSonarTrig = 4;
const int lSonarEcho = 3;
const int dSonarTrig = 13;
const int dSonarEcho = 2;

int fDistance;
int rDistance;
int lDistance;
int dDistance;
int difference;

int max=50,min;

void setup() {
  // put your setup code here, to run once:
 //initializing motorPins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

    pinMode(fSonarTrig, OUTPUT);
  pinMode(fSonarEcho, INPUT);
  pinMode(rSonarTrig, OUTPUT);
  pinMode(rSonarEcho, INPUT);
  pinMode(lSonarTrig, OUTPUT);
  pinMode(lSonarEcho, INPUT);
  pinMode(dSonarTrig, OUTPUT);
  pinMode(dSonarEcho, INPUT);

  Serial.begin(9600);
}

void loop() {
goForward();
delay(1000);
  
  // put your main code here, to run repeatedly:
takeSonarReadings();

while (fDistance>=30)
{
goForward();
takeSonarReadings();
}

while (fDistance<30 && fDistance>=20)
{
  goForward();
  takeSonarReadings();
  dDistance= trigger(dSonarTrig,dSonarEcho);
} 

while(fDistance <20) {
  fDistance= trigger(fSonarTrig,fSonarEcho);
  dDistance= trigger(dSonarTrig,dSonarEcho);

  if(dDistance >20)
  {
  brake();
  fDistance= trigger(fSonarTrig,fSonarEcho);
  dDistance= trigger(dSonarTrig,dSonarEcho);
  }

  else if (dDistance <=20 )
  {
  curveRun();
  fDistance= trigger(fSonarTrig,fSonarEcho);
  dDistance= trigger(dSonarTrig,dSonarEcho);
   }

Serial.print("Front: ");
  Serial.println(fDistance);

  Serial.print("Down: ");
  Serial.println(dDistance);


}

}




//motor functions

void goForward()
{
  analogWrite(motorPin1, max);
  analogWrite(motorPin2 , 0);
  analogWrite(motorPin3, max);
  analogWrite(motorPin4, 0);

}

void goBackward()
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2 , max);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4,max);

}

void goRight(int beshi,int kom)
{
  analogWrite(motorPin1, beshi);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, kom);

}

void goLeft(int beshi,int kom)
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, kom);
  analogWrite(motorPin3, beshi);
  analogWrite(motorPin4, 0);

}

void brake()
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 0);

}


void rotate()
{
  analogWrite(motorPin1,40);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4,40);

}

//sonar functions
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
  analogWrite(motorPin1, 50);
  analogWrite(motorPin2 , 0);
  analogWrite(motorPin3, 50);
  analogWrite(motorPin4, 0);

}

void plannedRotate()
{
  analogWrite(motorPin1,40);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4,40);

}

void plannedLeft()
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 40);
  analogWrite(motorPin3, 100);
  analogWrite(motorPin4, 0);

}

void curveRun() {
plannedRotate();
delay(800);
plannedForward();
delay(2100);
plannedLeft();
delay(450);
plannedForward();
delay(2100);
plannedLeft();
delay(400);
plannedForward();
delay(1050);
plannedRotate();
delay(1000);
brake();

}


