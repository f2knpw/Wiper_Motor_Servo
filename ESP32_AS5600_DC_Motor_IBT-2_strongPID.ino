/*run a DC motor with IBT-2 driver and control loop with AS5600 hall encoder*/


#include "AS5600.h"
#include "Wire.h"

AS5600 as5600;
#define SDA_PIN 19
#define SCL_PIN 23

double nbRot;               //current number of full rotations
double targetRot, targetRot2;
int rawValue, prevRawValue ;
int factor = 0;

/*
  IBT-2 Motor Control Board driven by Arduino.
  Connection to the IBT-2 board:
  IBT-2 pins  7 (VCC) to ESP32 3.3V
  IBT-2 pin 8 (GND) to GND
  IBT-2 pins 5 (R_IS) and 6 (L_IS) not connected
*/
#define RPWM_PIN  18  // PWM output pin ; connect to IBT-2 pin 1 (RPWM)
#define LPWM_PIN  5   // PWM output pin ; connect to IBT-2 pin 2 (LPWM)
#define ENABLE_PIN_R 17 // IBT-2 Enable pins R ; connect to IBT-2 pin 3 (LPWM)
#define ENABLE_PIN_L 16 // IBT-2 Enable pins L ; connect to IBT-2 pin 4 (LPWM)

boolean CW = true;
int pwmSpeed ;    //PWM duty cycle to command the DC motor speed
long timeOut;     //used to change setpoint every 30s


//PID
#include <PID_v1.h>   //https://github.com/br3ttb/Arduino-PID-Library
double setpoint, input, output; //used by PID lib
//setpoint= nb Rotation of the motor shaft,
//input = current rotation,
//output is pwmSpeed of the motor

//Specify the links and initial tuning parameters
double Kp = 60., Ki = 1.2, Kd = 0.;                         //will be modified into the loop
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);  //declare the PID
#define DEADBAND 400          //if pwm < DEADBAND, it stalls
#define INERTIA  1155./4096. //inertia of motor when stopping (PID over shoot on sensor position)

void setup()
{
  Serial.begin(115200);

  //AS5600
  Wire.begin(SDA_PIN, SCL_PIN);
  //  as5600.begin(4);                          //  set direction pin. ==> not used
  //  as5600.setDirection(AS5600_CLOCK_WISE);   // ==> not used, default is Clock Wise
  int b = as5600.isConnected();
  Serial.print("AS5600 is connected: ");
  Serial.println(b);
  rawValue = as5600.readAngle();
  prevRawValue = rawValue;
  Serial.print("rawValue " );
  Serial.print("\t");
  Serial.println(rawValue );

  //IBT-2 DC motor driver
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(ENABLE_PIN_L, OUTPUT);
  pinMode(ENABLE_PIN_R, OUTPUT);

  timeOut = millis();    //used to change setpoint every 30s
  pwmSpeed = 0;           //motor stopped at startup (range -2047 to 2047)

  ledcAttachPin(RPWM_PIN, 0); // assign PWM pins to channels
  ledcAttachPin(LPWM_PIN, 1); // assign PWM pins to channels
  // Initialize channels : ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(0, 24000, 11); // 24 kHz PWM, 11-bit resolution (range 0-2047)
  ledcSetup(1, 24000, 11);
  digitalWrite(ENABLE_PIN_L, HIGH); //enable the half bridges
  digitalWrite(ENABLE_PIN_R, HIGH);
  nbRot =  -150 ;                 //initialize nbRot to what you want
  targetRot2 = getMotorPos();     //initialize where we exactly are (nbRot + decimal part)
  targetRot = 150.0 ;             //initialize the target number of rotations of the motor shaft (0 at startup)
   Serial.print("current position " );
  Serial.print("\t");
  Serial.println(targetRot2 );

  //turn the PID on
  setpoint =  targetRot - INERTIA; //setpoint is this target number corrected of the INERTIA bias
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-2047, 2047); //output of the PID is the pwm value. coded on 11 bits and forward/reverse
  myPID.SetSampleTime(1);             //compute the PID every 1ms
}


void loop()
{
  //AS5600

  //Serial.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);
  //Serial.println((as5600.getAngularSpeed(AS5600_MODE_DEGREES) * 60. / 360.)); //should give rpm
  rawValue = as5600.readAngle();
  if ((rawValue - prevRawValue) < -999)//apply hysteresis to detect each full rotation (4095 <--> 0)
  {
    nbRot++ ;
    CW = true;
  }
  if ((rawValue - prevRawValue) > 999)
  {
    nbRot-- ;
    CW = false;
  }

  if (abs(prevRawValue - rawValue) > 10) timeOut = millis();        //rearm timeout if motor is running

  //  if (prevRawValue != rawValue)
  //  {
  //    Serial.print(rawValue );
  //    Serial.print("\t");
  //    Serial.print(setpoint );
  //    Serial.print("\t");
  //    Serial.print(input );
  //    Serial.print("\t\t");
  //    Serial.println(pwmSpeed);
  //  }

  prevRawValue = rawValue; //save the rawValue for next iteration

  //PID stuff
  input = getMotorPos();

  Kp = 6000;                     // start with an almost fully proportionnal PID
  Ki = 20.;
  Kd = 100. ;

  myPID.SetTunings(Kp, Ki, Kd);
  myPID.Compute();


  //DC motor driver
  pwmSpeed = output;  //use output of the PID to drive the motor
  runMotor();         //will apply direction and pwm value to the driver

  if (millis() - timeOut > 2000)      //stop the motor then change setpoint every 6s
  {
    //Serial.print("motor driver stopped ");
    digitalWrite(ENABLE_PIN_L, LOW);  //disable the half bridges ==> motor fully stopped
    digitalWrite(ENABLE_PIN_R, LOW);
    if (millis() - timeOut > 4000)   //restart 3s later
    {
      Serial.print("setpoint, input, sensor ");  //print setpoint and current position reached
      Serial.print("\t");
      Serial.print(setpoint );
      Serial.print("\t");
      Serial.print(input);
      Serial.print("\t");
      Serial.println(rawValue);
      factor++;                 //just to add some rabdom factor to next position
      factor = factor % 3;
      if (setpoint < 0) setpoint = abs(setpoint) * (factor + 1);
      else setpoint =  -(abs(targetRot2) - INERTIA); //setpoint is this target number corrected of the INERTIA bias
      //setpoint = -setpoint;             //change the setpoint
      timeOut = millis();
      digitalWrite(ENABLE_PIN_L, HIGH); //enable the half bridges
      digitalWrite(ENABLE_PIN_R, HIGH);
    }
  }
}
double getMotorPos(void)  //return the current position nRot + decimal part(rawValue)
{
  double fract = double(rawValue) / 4096.;       //compute the decimal part of nbRot using the rawValue of the AS5600
  double fractRot;
  if (nbRot >= 0) fractRot = nbRot + fract;//easy when CW
  else fractRot = -( -nbRot + 1. - fract); //more tricky if CCW
  return fractRot;
}

void runMotor(void)
{
  if (pwmSpeed > 0)
  {
    ledcWrite(0, pwmSpeed);
    ledcWrite(1, 0);
  }
  else
  {
    ledcWrite(0, 0);
    ledcWrite(1, -pwmSpeed);
  }
}
