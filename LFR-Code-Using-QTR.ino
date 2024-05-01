#include <L298NX2.h>
#include <QTRSensors.h>
QTRSensors qtr;

#define Kp 0.018
#define Kd 0.0001
#define MaxSpeed 180
#define BaseSpeed 150
#define SpeedTurn 150
#define CheckPoint 800

const uint8_t SensorCount = 8;
// IR Sensors Count 
uint16_t sensorValues[SensorCount];

//Motor A
int ENA = 3; int IN1 = 22; int IN2 = 24;

//Motor B
int ENB = 2; int IN3 = 34; int IN4 = 32;

L298NX2 motors(ENA, IN1, IN2, ENB, IN3, IN4);
int lastError = 0;

void setup()
{   
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A8, A9, A10, A11, A12, A13, A14, A15}, SensorCount);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    Serial.begin(9600);
    delay(500);
    
    pinMode(LED_BUILTIN, OUTPUT);

    // LED turns ON to indicate the start of the calibration
    digitalWrite(LED_BUILTIN, HIGH);
    delay(3000);
    
    for (int i = 0; i < 400; i++)
    {   
        motors.forwardA();
        motors.backwardB();
        motors.setSpeed(SpeedTurn);
        qtr.calibrate();
        delay(20);
    }

    digitalWrite(LED_BUILTIN, LOW);
    delay(3000); 
}  

void loop()
{  
  followLine();
}

void followLine(){

    uint16_t position = qtr.readLineBlack(sensorValues);  
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println(position);

    // too Right, Robot will go left
    if(position>3800)  
    {
        motors.setSpeed(BaseSpeed);
        motors.forwardA();
        motors.backwardB();
        return;    
    }

    // too Left, Robot will go right
    if(position<2900)
    {  
        motors.setSpeed(BaseSpeed);
        motors.forwardB();
        motors.backwardA();
        return;
    }

    int error = position - 3500;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    Serial.print("Error: ");
    Serial.println(error);

    int rightMotorSpeed = BaseSpeed + motorSpeed;
    int leftMotorSpeed = BaseSpeed - motorSpeed;
  
    if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; 
    if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0)rightMotorSpeed = 0;    
    if (leftMotorSpeed < 0)leftMotorSpeed = 0;
    
    motors.setSpeedA(rightMotorSpeed);
    motors.setSpeedB(leftMotorSpeed);
    motors.forward();
}



