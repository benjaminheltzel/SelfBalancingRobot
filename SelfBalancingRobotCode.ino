#include <Wire.h>

//MPU6050 
long accelX;
long accelY;
long accelZ;

float gForceX;
float gForceY;
float gForceZ;

float offsetSum_gForceX;
float offsetSum_gForceY;
float offsetSum_gForceZ;

float offset_gForceX;
float offset_gForceY;
float offset_gForceZ;

float calibrated_gForceX;
float calibrated_gForceY;
float calibrated_gForceZ;

long gyroX;
long gyroY;
long gyroZ;

float rotX;
float rotY;
float rotZ;

float offsetSum_rotX;
float offsetSum_rotY;
float offsetSum_rotZ;

float offset_rotX;
float offset_rotY;
float offset_rotZ;

float calibrated_rotX;
float calibrated_rotY;
float calibrated_rotZ;

int CalibrationPointsCounter;

int CalibrationPoints;

//kalman filter variables
float measuredTheta;

float kalmanGain;

float kalmanTheta = 0;
float kalmanThetaUncertainty = 2*2; //2*2

float timestep = 0.004; //time it takes for one loop to complete

float gyroVariance = 4;  //4 guesstimate from youtubevideo

float accelVariance = 3; //3 guesstimate from youtubevideo

float kalmanOutput[] = {0, 0};

//complementary filter
float angle_acc;
float angle_gyro;

//ports
const int stepRight = PD2; //PIN2
const int stepLeft = PD4; //PIN4
const int dirRight = PD3; //PIN3
const int dirLeft = PD5; //PIN5

const int sleepRight = 11;
const int sleepLeft = 12;


//measure timestep
long int t1;
long int t2;

//PID
float Kp =6;
float Kd = 0;
float Ki = 0.01;

float PID_I;
float error;
float previous_error;
float PID_value;
float setpoint = 2.5;


//linearize step motor
float PID_value_lin;
int motor_speed;


//stepper pulses
int pulse_delay_counter;
volatile int pulse_delay;
int previous_pulse_delay;

int direction = 1;


unsigned long loop_timer = 4000;


void setup() {

  CalibrationPoints = 250;

  Serial.begin(9600);
  Wire.begin();
  setupMPU();
  
  DDRD |= (1 << stepRight); //sets pin 2 as output  
  DDRD |= (1 << stepLeft); //sets pin 4 as output 
  DDRD |= (1 << dirRight); //sets pin 3 as output  
  DDRD |= (1 << dirLeft); //sets pin 5 as output                                                      //Configure digital poort 2 as output
  
  pinMode(sleepLeft, OUTPUT);
  pinMode(sleepRight, OUTPUT);

  //puts stepper motors tor sleep so they dont mess up the calibration by moving randomly
  digitalWrite(sleepLeft, HIGH);
  digitalWrite(sleepRight, HIGH);

  delay(5000);

// sum all values over #calibrationPoints
 for(CalibrationPointsCounter = 0; CalibrationPointsCounter<CalibrationPoints; CalibrationPointsCounter++){
   recordAccelRegistersNOTcalibrated();
   recordGyroRegistersNOTcalibrated();

   offsetSum_gForceX += gForceX;
   offsetSum_gForceY += gForceY;
   offsetSum_gForceZ += gForceZ;
   
   offsetSum_rotX += rotX;
   offsetSum_rotY += rotY;
   offsetSum_rotZ+= rotZ;

   Serial.println(CalibrationPointsCounter);

  delay(50);

 }
 
 offset_gForceX = offsetSum_gForceX/CalibrationPoints;
 offset_gForceY = offsetSum_gForceY/CalibrationPoints;
 offset_gForceZ = offsetSum_gForceZ/CalibrationPoints;

 offset_rotX = offsetSum_rotX/CalibrationPoints;
 offset_rotY = offsetSum_rotY/CalibrationPoints;
 offset_rotZ = offsetSum_rotZ/CalibrationPoints;



  // set CTC mode so TIMER1 goes to 0 after compare value is reached
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |= (1 << WGM12);

  //reset TIMER1 control register A
  TCCR1A = 0;

  //set prescaler of TIMER1 to 8 by setting register to 010
  TCCR1B &= ~(1 << CS12); // 0
  TCCR1B |= (1 << CS11);  // 1
  TCCR1B &= ~(1 << CS10); // 0

  //TIMER1 counts from 0 up to 39, which means the interrupt is triggered every 20us 
  TCNT1 = 0;
  OCR1A = 39;

  //enable TIMER1 compare interrupt
  TIMSK1 = (1 << OCIE1A);

  //enable global interrupts
  sei();

  digitalWrite(sleepRight, LOW);
  digitalWrite(sleepLeft, LOW);

}

void loop() {

  t1 = micros();

 //measure accelerations in X-axis and Z-axis
 recordAccelRegisterscalibrated();
 //measure angular velocity around Y-Axis
 recordGyroRegisterscalibrated();

 //angle measured from acceleration sensor
 angle_acc = atan(calibrated_gForceZ/calibrated_gForceX)*57.296; // *180/3,1416

 //angle from calibrating the angular speed around the y-axis
 angle_gyro = angle_gyro + calibrated_rotY*0.004;

 // complementary filter
 angle_gyro = angle_gyro * 0.98 + angle_acc * 0.02;

 //kalman filter
 //kalman(kalmanTheta, kalmanThetaUncertainty, calibrated_rotY, measuredTheta);

 //kalmanTheta = kalmanOutput[0];
 //kalmanThetaUncertainty = kalmanOutput[1];


  // PID
  error =  angle_gyro - setpoint; //for computation with angle from kalman filter, use kalmanTheta instead of angle_gyro

  //compute integral value
  PID_I = PID_I + Ki * error; 

  // limit I value
  if (PID_I > 400) {
    PID_I = 400;

  } else if (PID_I < -400) {
    PID_I = -400;
  }

  PID_value = Kp*error + PID_I + Kd*(error - previous_error);
  
  // limit speed
  if (PID_value > 400) {
    PID_value = 400; 
    
  } else if (PID_value < -400) {
    PID_value = -400;
  }


  previous_error = error;


 if (angle_gyro > 30 || angle_gyro < -30) {           
    PID_value = 0;     
    PID_I = 0; 
 } 


  PID_value_lin = PID_value;                          
  

  // make PID_value logarithmic to linearize the steppermotor behaviour
  if (PID_value_lin > 0) {
    PID_value_lin = 405 - (1 / (PID_value_lin + 9)) * 5500;

  } else if (PID_value_lin < 0) {
    PID_value_lin = -405 - (1 / (PID_value_lin - 9)) * 5500;
  }


  if (PID_value_lin > 0){
    motor_speed = 400 - PID_value_lin;

  } else if (PID_value_lin < 0) {
    motor_speed = -400 - PID_value_lin;
  
  } else{
    motor_speed = 0;
  }


  // direction of motor
  if(motor_speed > 0) {  
     PORTD &= ~(1 << dirLeft);
     PORTD |= (1 << dirRight); 

  } else if(motor_speed < 0) {
    motor_speed = motor_speed*(-1);
    PORTD |= (1 << dirLeft);
    PORTD &= ~(1 << dirRight); 
  }

  pulse_delay = motor_speed;
  //set loop to 4000us
  while(loop_timer > micros() - t1);


}

//interrupt service routine

ISR(TIMER1_COMPA_vect) {
  pulse_delay_counter ++;
  //sets new delay between stepper pulses after the previous delay is reached
  if (pulse_delay_counter > previous_pulse_delay) {
    pulse_delay_counter = 0;                                    
    previous_pulse_delay = pulse_delay; 

  } else if (pulse_delay_counter == 1) {
    //set Motor high
    PORTD |= (1 << stepLeft);           
    PORTD |= (1 << stepRight);

  } else if (pulse_delay_counter == 2) {
    //set Motor low
    PORTD &= ~(1 << stepLeft);         
    PORTD &= ~(1 << stepRight); 
  }
}


void kalman(float kalmanState, float kalmanUncertainty, float kalmanInput, float kalmanMeasurement) {
  
  kalmanState = kalmanState + timestep*kalmanInput;

  kalmanUncertainty = kalmanUncertainty + timestep*timestep*gyroVariance*gyroVariance;

  kalmanGain = kalmanUncertainty/(kalmanUncertainty + accelVariance*accelVariance);

  kalmanState = kalmanState + kalmanGain*(kalmanMeasurement - kalmanState);

  kalmanUncertainty = (1 - kalmanGain)*kalmanUncertainty;

  kalmanOutput[0] = kalmanState;
  kalmanOutput[1] = kalmanUncertainty;

}

void setupMPU(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x6B);
  Wire.write(0b00000000); // sleep set to 0
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);
  Wire.write(0b00000000); //gyro set to +- 250 deg/s
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);
  Wire.write(0b00000000); //accel set to +- 2g
  Wire.endTransmission();
  // set digital filter of the MPU6050 to 43Hz
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x1A);                           
  Wire.write(0x03);                          
  Wire.endTransmission();                     

}

void recordAccelRegistersNOTcalibrated(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read(); //store first two bytes
  accelY = Wire.read() << 8 | Wire.read(); //store middle two bytes
  accelZ = Wire.read() << 8 | Wire.read(); //store last two bytes
  
  gForceX = accelX/16384.0;
  gForceY = accelY/16384.0;
  gForceZ = accelZ/16384.0;
}


void recordGyroRegistersNOTcalibrated(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available() < 6);
  gyroX = Wire.read() << 8 | Wire.read(); //store first two bytes
  gyroY = Wire.read() << 8 | Wire.read(); //store middle two bytes
  gyroZ = Wire.read() << 8 | Wire.read(); //store last two bytes
  
  rotX = gyroX/131.0;
  rotY = gyroY/131.0;
  rotZ = gyroZ/131.0;
}

void recordAccelRegisterscalibrated(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read(); //store first two bytes
  accelY = Wire.read() << 8 | Wire.read(); //store middle two bytes
  accelZ = Wire.read() << 8 | Wire.read(); //store last two bytes
  
  calibrated_gForceX = accelX/16384.0 - offset_gForceX + 1; //IMU is rotated
  calibrated_gForceZ = accelZ/16384.0 - offset_gForceZ; 
}

void recordGyroRegisterscalibrated(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available() < 6);
  gyroX = Wire.read() << 8 | Wire.read(); //store first two bytes
  gyroY = Wire.read() << 8 | Wire.read(); //store middle two bytes
  gyroZ = Wire.read() << 8 | Wire.read(); //store last two bytes
  

  calibrated_rotY = gyroY/131.0 - offset_rotY;

}