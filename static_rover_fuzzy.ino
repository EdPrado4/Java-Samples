// Be sure to include the eFFL library before compile the script
// Tools -> Include Library -> Add .zip library
#include <Wire.h>
#include <SPI.h>
#include <Mirf.h>
#include <MirfHardwareSpiDriver.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <FuzzyRule.h>
#include <FuzzyComposition.h>
#include <Fuzzy.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzyOutput.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzySet.h>
#include <FuzzyRuleAntecedent.h>

#define  sign(X) (X >= 0 ? 1 : -1)

MPU6050 accelgyro;
MPU6050 initialize;

#define pi 3.14159
unsigned long preTime, lastTime;
int timeChange;
int TN1 = 23;
int TN2 = 22;
int ENA = 5;
int TN3 = 24;
int TN4 = 25;
int ENB = 4;

//Defining an instance of fuzzy class fuzzy_rover
Fuzzy* fuzzy_rover = new Fuzzy();


void setup()
{
  float Y1, Y1Dot, U1;
  Serial2.begin(115200);   
  Wire.begin();

  TCCR3A = _BV(COM3A1) | _BV(WGM31) | _BV(WGM30); // TIMER_3 @1K Hz, fast pwm
  TCCR3B = _BV(CS31);
  TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); // TIMER_0 @1K Hz, fast pwm
  TCCR0B = _BV(CS01) | _BV(CS00);

  /* If the robot was turned on with the angle over 45(-45) degrees,the wheels
   will not spin until the robot is in right position. */
  accelgyro.initialize();
  //  for (int i = 0; i < 200; i++) // Looping 200 times to get the real gesture when starting
  //  {
  //      Read_Platform_Orientation(&Y1,&Y1Dot);
  //  }
  // H bridge left motor
  pinMode(TN1, OUTPUT);
  pinMode(TN2, OUTPUT);
  // H bridge right motor  
  pinMode(TN3, OUTPUT);
  pinMode(TN4, OUTPUT);  
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

// Definition of the fuzzy variables:

  // Defining Fuzzy Input error
  FuzzyInput* errorF = new FuzzyInput(1);
  // Defining fuzzy set values for input error:
  FuzzySet* negative = new FuzzySet(-60, -35, -15 ,0);          // Negative error
  errorF->addFuzzySet(negative);                                // Adding FuzzySet negative in errorF
  FuzzySet* zero = new FuzzySet(-0.8, -0.3, 0.3, 0.8);          // Zero error 
  errorF->addFuzzySet(zero);                                    // Adding FuzzySet zero in errorF
  FuzzySet* positive = new FuzzySet(0, 15, 35, 60);             // Positive error
  errorF->addFuzzySet(positive);                                // Adding FuzzySet positive in errorF
  // Adding Fuzzy Input errorF to fuzzy_rover
  fuzzy_rover->addFuzzyInput(errorF);

  // Defining fuzzy Input errorF_change:
  FuzzyInput* errorF_change = new FuzzyInput(2);
  // Defining fuzzy set values for input errorF_change:
  FuzzySet* negative_change = new FuzzySet(-250, -100, -50 ,0); // Negative error
  errorF_change->addFuzzySet(negative_change);                  // Adding FuzzySet negative in errorF
  FuzzySet* zero_change = new FuzzySet(-0.5, -0.2, 0.2, 0.5);   // Zero error_change 
  errorF_change->addFuzzySet(zero_change);                      // Adding FuzzySet zero in errorF
  FuzzySet* positive_change = new FuzzySet(0, 50, 100, 250);    // Positive error
  errorF_change->addFuzzySet(positive_change);                  // Adding FuzzySet positive in errorF
  // Adding Fuzzy Input errorF to fuzzy_rover
  fuzzy_rover->addFuzzyInput(errorF_change);

  // Defining FuzzyOutput controlF
  FuzzyOutput* controlF = new FuzzyOutput(1);
  // Defining fuzzy set values for output controlF
  FuzzySet* Large_Negative = new FuzzySet(-30, -30, -30, -30);  // Large_Negative Control law
  controlF->addFuzzySet(Large_Negative);                        // Adding FuzzySet Large_Negative in controlF
  FuzzySet* Small_Negative = new FuzzySet(-15, -15, -15, -15);  // Small Negative control law
  controlF->addFuzzySet(Small_Negative);                        // Adding FuzzySet Small_Negative in controlF
  FuzzySet* No_Control = new FuzzySet(-1, 0, 0, 1);             // No control Law
  controlF->addFuzzySet(No_Control);                            // Adding FuzzySet No_Control in ControlF
  FuzzySet* Large_Positive = new FuzzySet(30, 30, 30, 30);      // Large_Positive Control law
  controlF->addFuzzySet(Large_Positive);                        // Adding FuzzySet Large_Positive in controlF
  FuzzySet* Small_Positive = new FuzzySet(15, 15, 15, 15);      // Small Positive control law
  controlF->addFuzzySet(Small_Positive);                        // Adding FuzzySet Small_Positive in controlF
  // Adding Fuzzy Output errorF to fuzzy_rover
  fuzzy_rover->addFuzzyOutput(controlF);

  //----------------------------------------------------------------//------------------------------------//------------------------------------------------------------
  // Defining the rules for the fuzzy controller:

  // Rule #1: if error is NEG and error_change is NEG THEN control is NEG:
  // Antecedent: 
  FuzzyRuleAntecedent* iferrorNegANDerrchangeNeg = new FuzzyRuleAntecedent();
  iferrorNegANDerrchangeNeg->joinWithAND(negative, negative_change);
  // Consecuent: 
  FuzzyRuleConsequent* thenControlisLNeg = new FuzzyRuleConsequent();
  thenControlisLNeg->addOutput(Large_Negative);
  FuzzyRule* fuzzyRule1 = new FuzzyRule(1,  iferrorNegANDerrchangeNeg, thenControlisLNeg);
  fuzzy_rover->addFuzzyRule(fuzzyRule1);

  // Rule #2: if error is NEG and error_change is ZERO THEN control is NEG:
  // Antecedent: 
  FuzzyRuleAntecedent* iferrorNegANDerrchangeZer = new FuzzyRuleAntecedent();
  iferrorNegANDerrchangeZer->joinWithAND(negative, zero_change);
  // Consecuent: 
  FuzzyRuleConsequent* thenControlisSNeg = new FuzzyRuleConsequent();
  thenControlisSNeg->addOutput(Small_Negative);
  FuzzyRule* fuzzyRule2 = new FuzzyRule(2,  iferrorNegANDerrchangeZer, thenControlisSNeg);
  fuzzy_rover->addFuzzyRule(fuzzyRule2);

  // Rule #3: if error is NEG and error_change is POS THEN control is ZERO:
  // Antecedent: 
  FuzzyRuleAntecedent* iferrorNegANDerrchangePos = new FuzzyRuleAntecedent();
  iferrorNegANDerrchangePos->joinWithAND(negative, positive_change);
  // Consecuent: 
  FuzzyRuleConsequent* thenControlisZer = new FuzzyRuleConsequent();
  thenControlisZer->addOutput(No_Control);
  FuzzyRule* fuzzyRule3 = new FuzzyRule(3,  iferrorNegANDerrchangePos, thenControlisZer);
  fuzzy_rover->addFuzzyRule(fuzzyRule3);

  // Rule #4: if error is ZERO and error_change is NEG THEN control is NEG:
  // Antecedent: 
  FuzzyRuleAntecedent* iferrorZerANDerrchangeNeg = new FuzzyRuleAntecedent();
  iferrorZerANDerrchangeNeg->joinWithAND(zero, negative_change);
  // Consecuent: 
  FuzzyRule* fuzzyRule4 = new FuzzyRule(4,  iferrorZerANDerrchangeNeg, thenControlisSNeg);
  fuzzy_rover->addFuzzyRule(fuzzyRule4);

  // Rule #5: if error is ZERO and error_change is ZERO THEN control is ZERO:
  // Antecedent: 
  FuzzyRuleAntecedent* iferrorZerANDerrchangeZer = new FuzzyRuleAntecedent();
  iferrorZerANDerrchangeZer->joinWithAND(zero, zero_change);
  // Consecuent: 
  FuzzyRule* fuzzyRule5 = new FuzzyRule(5,  iferrorZerANDerrchangeZer, thenControlisZer);
  fuzzy_rover->addFuzzyRule(fuzzyRule5);

  // Rule #6: if error is ZERO and error_change is POS THEN control is POS:
  // Antecedent: 
  FuzzyRuleAntecedent* iferrorZerANDerrchangePos = new FuzzyRuleAntecedent();
  iferrorZerANDerrchangePos->joinWithAND(zero, positive_change);
  // Consecuent: 
  FuzzyRuleConsequent* thenControlisSPos = new FuzzyRuleConsequent();
  thenControlisSPos->addOutput(Small_Positive);
  FuzzyRule* fuzzyRule6 = new FuzzyRule(6,    iferrorZerANDerrchangePos, thenControlisSPos);
  fuzzy_rover->addFuzzyRule(fuzzyRule6);

  // Rule #7: if error is POS and error_change is NEG THEN control is ZERO:
  // Antecedent: 
  FuzzyRuleAntecedent* iferrorPosANDerrchangeNeg = new FuzzyRuleAntecedent();
  iferrorPosANDerrchangeNeg->joinWithAND(positive, negative_change);
  // Consecuent: 
  FuzzyRule* fuzzyRule7 = new FuzzyRule(7,  iferrorPosANDerrchangeNeg, thenControlisZer);
  fuzzy_rover->addFuzzyRule(fuzzyRule7);

  // Rule #8: if error is POS and error_change is ZERO THEN control is POS:
  // Antecedent: 
  FuzzyRuleAntecedent* iferrorPosANDerrchangeZer = new FuzzyRuleAntecedent();
  iferrorPosANDerrchangeZer->joinWithAND(positive, zero_change);
  // Consecuent: 
  FuzzyRule* fuzzyRule8 = new FuzzyRule(8,  iferrorPosANDerrchangeZer, thenControlisSPos);
  fuzzy_rover->addFuzzyRule(fuzzyRule8);

  // Rule #9: if error is POS and error_change is POS THEN control is POS:
  // Antecedent: 
  FuzzyRuleAntecedent* iferrorPosANDerrchangePos = new FuzzyRuleAntecedent();
  iferrorPosANDerrchangePos->joinWithAND(positive, positive_change);
  // Consecuent: 
  FuzzyRuleConsequent* thenControlisLPos = new FuzzyRuleConsequent();
  thenControlisLPos->addOutput(Large_Positive);
  FuzzyRule* fuzzyRule9 = new FuzzyRule(9,  iferrorPosANDerrchangePos, thenControlisLPos);
  fuzzy_rover->addFuzzyRule(fuzzyRule9);
}
void loop()
{    
  // Setpoints, outputs and control laws
  float R1 = 0, Y1, Y1Dot, U1;
  unsigned long int TBegin; 
  
  TBegin = micros();
  R1     = 0;
  Read_Platform_Orientation(&Y1,&Y1Dot);
  //Y1 = 10; Y1Dot = 0;
  if  (abs(Y1) < 45) {
    //Y1Dot = abs(Y1Dot) < 0.2 ? 0 : Y1Dot;
    Fuzzy_Control_Law(R1-Y1,Y1Dot,&U1);
    Write_Control_Law(U1);
  }
  else {
        digitalWrite(TN1, HIGH);
        digitalWrite(TN2, HIGH);
        digitalWrite(TN3, HIGH);
        digitalWrite(TN4, HIGH);
       }
// Uncomment when working on data adquisition interface
  //Serial2.println(R1);
  //Serial2.println(U1);
  //Serial2.println(Y1);
  //Serial2.println(Y1Dot);  

  wait_until_next_sampling_time(TBegin);
}

 // The offset of the accelerator
 #define Angle_offset 1.4  
 // From integer values to deg/seg
 #define Gyr_Gain     131.07
 //The offset of the gyro
 #define Gry_offset   -0.3

void Read_Platform_Orientation(float *Y1, float *Y1Dot) {
  // Accelerometer readings [-32768,+32767] maps to [-2g,2g] m/sec^2 
  // g being the gravitational acceleration
  int16_t      ax, ay, az;
  // Gyro readings [-32768,+32767] maps to [-250,250] deg/sec 
  int16_t      gx, gy, gz;
  
  // Angles
  static unsigned long int preTime = 0;
  static float Angle_Filtered = 0, Angle_Recursive = 0;
  float        Angle_Delta, Angle_Raw, Angle_Confidence, Omega, dt;

  // Read the accelerometer
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Raw Angle
  Angle_Raw          = (atan2(ay, az) * 180 / pi + Angle_offset);
  // Transformation for the angular velocity from integer values to deg/sec 
  // X axis of the giro is the rotation axis
  Omega              = gx / Gyr_Gain + Gry_offset;
  // Filter datas to get the real gesture
  unsigned long now  = micros();
  timeChange         = now - preTime;
  preTime            = now;
  // Converting from microseconds to seconds
  dt                 = timeChange * 1E-6;
  // Accel Gain: 0.64
  Angle_Delta        = (Angle_Raw - Angle_Filtered) * 0.64;
  // Cummulative of estimated orientation
  Angle_Recursive    = Angle_Delta * dt + Angle_Recursive;
  Angle_Confidence   = Angle_Recursive + (Angle_Raw - Angle_Filtered) * 1.6 + Omega;
  Angle_Filtered     = Angle_Confidence * dt + Angle_Filtered;
  // Platform orientation
  *Y1                = Angle_Filtered;
  *Y1Dot             = Omega;
}

 // Wait until next sampling time
void wait_until_next_sampling_time(unsigned long int TBegin) {
      while (micros() - TBegin < (unsigned long int)10000);  
}

// PID Control Law
void Fuzzy_Control_Law(float Error,float ErrorDot,float *U1)
{
  static float ErrSum = 0;  
  // Calculating the output values using the gesture values and the PID values.
  ErrSum            += Error;
  //Defining the Gains for the error and error change and control Law:
  float  GU = 11.00,GE = 0.70, GCE = 0.0010;

  fuzzy_rover->setInput(1, GE*Error);
  fuzzy_rover->setInput(2, GCE*ErrorDot);
  fuzzy_rover->fuzzify();
  //Fuzzy PD Control Law:
  *U1                = GU*fuzzy_rover->defuzzify(1); 
}

// If TN2 is high and TN1 is low then the left motor  is turning forward
// If TN4 is high and TN3 is low then the right motor is turning forward
void Write_Control_Law(float U1)
{
  float MotorOffset = 5.2;
  float LOutput, ROutput;
  
  LOutput            = U1 - MotorOffset;
  ROutput            = U1 - MotorOffset;  
  if (LOutput < 0)
  {
    digitalWrite(TN2, HIGH);
    digitalWrite(TN1, LOW);
  }
  else if (LOutput > 0)
  {
    digitalWrite(TN2, LOW);
    digitalWrite(TN1, HIGH);
  }
  else
  {
    OCR3A = 0;
  }
  if (ROutput < 0)
  {
    digitalWrite(TN4, HIGH);
    digitalWrite(TN3, LOW);
  } 
  else if (ROutput > 0)
  {
    digitalWrite(TN4, LOW);
    digitalWrite(TN3, HIGH);
  }
  else
  {
    OCR0B = 0;
  }
  // Timer/Counter3 is a general purpose 16-bit Timer/Counter module
  OCR3A = min(1023, 4*(abs(ROutput)) ); 
  // Timer/Counter0 is a general purpose 8-bit Timer/Counter module
  OCR0B = min(255,  abs(LOutput)   ); 
}
