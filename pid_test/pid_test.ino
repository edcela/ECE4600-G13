//#include <Wire.h>
//Gyroscope
float ratePitch, rateRoll, rateYaw;
float rateCalPitch,rateCalRoll,rateCalYaw;
int rateCalNum;

//Receiver
//#include <PulsePosition.h>
PulsePositionInput recieveInput(RISING);
float receiveValue[] = {0,0,0,0,0,0,0,0}
int channelNum = 0;

//Power Switch (Power Monitoring)
float volt,curr,battRem,battStart;
float currConsumed = 0;
float battDefault = 1300;

//Control loop length
uint32_t loopTime;

//PID variables and parameters
float desiredRollRate,desiredPitchRate,desiredYawRate;
float rateErrorRoll,rateErrorPitch,rateErrorYaw;
float inRoll,inThrottle,inPitch,inYaw;
float prevRateErrorRoll,prevRateErrorPitch,prevRateErrorYaw;
float prevItermRateRoll,prevItermRatePitch,prevItermRateYaw;
float PIDreturn[] = {0,0,0};

float PrateRoll = 0.6; PratePitch=PrateRoll;
float PrateYaw = 2;
float IrateRoll = 3.5; IratePitch=IrateRoll;
float IrateYaw = 12;
float DrateRoll = 0.03; DratePitch=DrateRoll;
float DrateYaw = 0;

//Motor input variables
float motorIn1, motorIn2, motorIn3, motorIn4;

//Power Switch Function
void batt_voltage(void){
  volt = (float)analogRead(15)/62;
  curr = (float)analogRead(21)*0.089;
}

//Receiver Function
void read_receiver(void){
  channelNum = receiveInput.available();
  if(channelNum>0){
    for(int i=1; i<=channelNum;i++){
      receiveValue[i-1]=receiveInput.read(i);
    }
  }
}

//Gyroscope Function
void gyro_signal(void){
/*reading signals from the gyroscope
.
.
.
.
.
*/

  //writing values into variables
  int16_t gyroX = gyro.read()<<8 | gyro.read();
  int16_t gyroY = gyro.read()<<8 | gyro.read();
  int16_t gyroZ = gyro.read()<<8 | gyro.read();

  rateRoll = (float)gyroX/65.5;
  ratePitch = (float)gyroY/65.5;
  rateYaw = (float)gyroZ/65.5;

}

//PID Function
void PID_eqtn(float error, float P, float I, fload D, float prevError, float prevIterm){
  float termP = P * error;
  float termI = prevIterm + I*(error + prevError)*0.004/2;
  if (termI >400) termI = 400;                              //the if and elseif statements are used to avoid integral windup
  else if (termI <-400) termI = -400;
  float termD = D*(error-prevError)/0.004;
  float outputPID = termP + termI + termD;
  if(outputPID >400) outputPID = 400;
  else if (outputPID <-400) outputPID = -400;

  //return output of the PID function
  PIDreturn[0] = outputPID;
  PIDreturn[1] = error;
  PIDreturn[2] = termI;
}

//PID reset function
void PID_reset(void){
  prevRateErrorRoll = 0; prevRateErrorPitch = 0; prevRateErrorYaw = 0;
  prevItermRateRoll = 0; prevItermRatePitch = 0; prevItermRateYaw = 0;
}

//setup phase visualization
void setup(){
  pinMode(5,OUTPUT);
  digitalWrite(5,HIGH);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);

//gyroscope communication
/*
.
.
.
.
.
*/

  //gyro calibration
  for(rateCalNum=0;rateCalNum<2000;rateCalNum++){
    gyro_signal();
    rateCalRoll += rateRoll;
    rateCalPitch += ratePitch;
    rateCalYaw += rateYaw;
    delay(1);
    }

  rateCalRoll /= 2000;
  rateCalPitch /= 2000;
  rateCalYaw /= 2000;

  //motor control
  analogWriteFrequency(1,250);
  analogWriteFrequency(1,250);
  analogWriteFrequency(1,250);
  analogWriteFrequency(1,250);
  analogWriteResolution(12);

  //end of setup process + determine initial battery volt %
  pinMode(6,OUTPUT);
  digitalWrite(6,HIGH);
  batt_voltage();
  if (volt > 8.3){
    digitalWrite(5,LOW);
    battStart = battDefault;
  }
  else if (volt<7.5){
    battStart = 30/100*battDefault;
  }
  else{
    digitalWrite(5,LOW);
     battStart = (82*volt*580)/100*battDefault;
  }
   
  //to avoid accidental take off after setup
  receiveInput.begin(14);
  while(receiveValue[2]<1020 || receiveValue[2]>1050){
    read_receiver();
    delay(4);
  }

  loopTimer = micros();         //timer start
}

void loop(){
  gyro_signal();
  rateRoll-= rateCalRoll;
  ratePitch -= rateCalPitch;
  rateYaw -= rateCalYaw;
  read_receiver();

  //calculate the desired roll pitch and yaw rates
  desiredRollRate = 0.15*(receiveValue[0]-1500);
  desiredPitchRate = 0.15*(receiveValue[1]-1500);
  inThrottle = receiveValue[2];
  desiredYawRate = 0.15*(receiveValue[3]-1500);

  //calculate PID calculation errors
  rateErrorRoll = desiredRollRate - rateRoll;
  rateErrorPitch = desiredPitchRate - ratePitch;
  rateErrorYaw = desiredYawRate - rateYaw;

  //execute PID calculations
  PID_eqtn(rateErrorRoll,PrateRoll,IrateRoll,DrateRoll,prevRateErrorRoll,prevItermRateRoll);
  inRoll = PIDReturn[0]; prevRateErrorRoll = PIDReturn[1]; prevItermRateRoll = PIDReturn[2];
  PID_eqtn(rateErrorPitch,PratePitch,IratePitch,DratePitch,prevRateErrorPitch,prevItermRatePitch);
  inPitch = PIDReturn[0]; prevRateErrorPitch = PIDReturn[1]; prevItermRatePitch = PIDReturn[2];
  PID_eqtn(rateErrorYaw,PrateYaw,IrateYaw,DrateYaw,prevRateErrorYaw,prevItermRateYaw);
  inYaw = PIDReturn[0]; prevRateErrorYaw = PIDReturn[1]; prevItermRateYaw = PIDReturn[2];

  //limit throttle output to 80%
  if (inThrottle>1800) inThrottle = 1800;

  //Quadcopter motors dynamics equations
  motorIn1 = 1.024*(inThrottle-inRoll-inPitch-inYaw);         //convert in ms to their 12bit equivalent by multiplying by 1024
  motorIn2 = 1.024*(inThrottle-inRoll+inPitch+inYaw);
  motorIn3 = 1.024*(inThrottle+inRoll+inPitch-inYaw);
  motorIn4 = 1.024*(inThrottle+inRoll-inPitch+inYaw);

  //limit the max power commands sent to motors to avoid overloading
  if (motorIn1>2000) motorIn1 = 1999;
  if (motorIn2>2000) motorIn2 = 1999;
  if (motorIn3>2000) motorIn3 = 1999;
  if (motorIn4>2000) motorIn4 = 1999;

  //keep motors running at min 18% power during flight
  int throttleIdle = 1180;
  if(motorIn1<throttleIdle) motorIn1 = throttleIdle;
  if(motorIn2<throttleIdle) motorIn2 = throttleIdle;
  if(motorIn3<throttleIdle) motorIn3 = throttleIdle;
  if(motorIn4<throttleIdle) motorIn4 = throttleIdle;

  //make sure motors can be turned off
  int cutOffThrottle = 1000;
  if(receiverValue[2]<1050){
    motorIn1 = cutOffThrottle;
    motorIn2 = cutOffThrottle;
    motorIn3 = cutOffThrottle;
    motorIn4 = cutOffThrottle;
    PID_reset();
  }

  //send the commands to the motors
  analogWrite(1,motorIn1);
  analogWrite(2,motorIn2);
  analogWrite(3,motorIn3);
  analogWrite(4,motorIn4);

  //keep watch of battery percentage
  batt_voltage();
  currConsumed = curr*1000*0.004/3600+currConsumed;
  battRem = (battStart-currConsumed)/battDefault*100;
  if(battRem <=30) digitalWrite(5,HIGH);
  else digitalWrite(5,LOW);

  //finish the control loop
  while (micros()-loopTimer <4000);
  loopTimer = micros();
}
