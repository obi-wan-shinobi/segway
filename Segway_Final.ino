#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
MPU6050 mpu;

//define motor parameters:
#define MpwmBL 3   //BOTTOM LEFT
#define MpwmBR 4   //BOTTOM RIGHT

#define MdirectionR1 5

#define MdirectionL1 7

#define Pot_pin A0

// proportional gain
float Kp=7.1;   //org 7.5
// deferential gain
float Kd=0.4;   //org 0.5
// integral gain
float Ki=10;  //12.5
String dir="";
float error=0;
float Aangle=0;
float Lastangle=0;
float LLastangle=0;
// Desired angle
float Dangle=0;
#define ModeAngle 50

// PID controller command
float COcommand =0;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int DMSR=0;
int DMSL=0;



#define DEBUG
#ifdef DEBUG
//#define DPRINT(args...)  Serial.print(args)             //OR use the following syntax:
#define DPRINTSTIMER(t)    for (static unsigned long SpamTimer; (unsigned long)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define  DPRINTSFN(StrSize,Name,...) {char S[StrSize];Serial.print("\t");Serial.print(Name);Serial.print(" "); Serial.print(dtostrf((float)__VA_ARGS__ ,S));}//StringSize,Name,Variable,Spaces,Percision
#define DPRINTLN(...)      Serial.println(__VA_ARGS__)
#else
#define DPRINTSTIMER(t)    if(false)
#define DPRINTSFN(...)     //blank line
#define DPRINTLN(...)      //blank line
#endif

#define LED_PIN 13 // 
int MPUOffsets[6] = {333 ,-2576 ,1084  ,12  ,-27  ,-39 }; //GY-521
//int MPUOffsets[6] = {333 ,-2576 ,1084  ,12  ,-27  ,-39 };  //GY-88
//int MPUOffsets[6] = {-304 ,239 ,1102  ,37  ,25  ,-9 };  //GY-88 //author cali values

void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
int FifoAlive = 0; // tests if the interrupt is triggering
int IsAlive = -20;     // counts interrupt start at -20 to get 20+ good values before assuming connected
// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
byte StartUP = 100; // lets get 100 readings from the MPU before we start trusting them (Bot is not trying to balance at this point it is just starting up.)

void MPU6050Connect() {
  static int MPUInitCntr = 0;
  // initialize device
  mpu.initialize(); // same
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();// same

  if (devStatus != 0) {
 
    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; //only try 10 times
    delay(1000);
    MPU6050Connect(); // Lets try again
    return;
  }
  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);

  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  // enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  Serial.print("mpu.getInterruptDrive=  "); Serial.println(mpu.getInterruptDrive());
  attachInterrupt(0, dmpDataReady, RISING); //pin 2 on the Uno
  mpuIntStatus = mpu.getIntStatus(); // Same
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  delay(1000); // Let it Stabalize
  mpu.resetFIFO(); // Clear fifo buffer
  mpu.getIntStatus();
  mpuInterrupt = false; // wait for next interrupt
}

// ================================================================
// ===                    MPU DMP Get Data                      ===
// ================================================================
void GetDMP() { // Best version I have made so far
  // Serial.println(F("FIFO interrupt at:"));
  // Serial.println(micros());
  static unsigned long LastGoodPacketTime;
  mpuInterrupt = false;
  FifoAlive = 1;
  fifoCount = mpu.getFIFOCount();
  if ((!fifoCount) || (fifoCount % packetSize)) { // we have failed Reset and wait till next time!
    digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    mpu.resetFIFO();// clear the buffer and start over
  } else {
    while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    LastGoodPacketTime = millis();
    MPUMath(); // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
  }
}


// ================================================================
// ===                        MPU Math                          ===
// ================================================================
float Yaw, Pitch, Roll;

  void SetMotorSpeed(int  DMSR, int DMSL, boolean state) {
 DMSR=constrain(DMSR, -100, 100);
 DMSL=constrain(DMSL, -100, 100);

     if (DMSR>0) {
       // set forward
     digitalWrite(MdirectionR1,LOW);
     
     dir="Forward";
     }
     else{
     // set reverse
     digitalWrite(MdirectionR1,HIGH);
     
     dir="Reverse";
      // change baluse to positive
      DMSR=-DMSR;           
   }
     
      if (DMSL>0) {
       // set forward
     digitalWrite(MdirectionL1,LOW);
     
      }
     else {
     // set reverse
     digitalWrite(MdirectionL1,HIGH);
    
      // change valuse to positive
     DMSL=-DMSL;
     }
     
     if (state) {
       analogWrite(MpwmBR, DMSR);
       analogWrite(MpwmBL, DMSL);
     }
       else{
       analogWrite(MpwmBR, DMSR);
       analogWrite(MpwmBL, DMSL);
       }
   }


void Mcommand () {
  // PID type C controller
  
  LLastangle=Lastangle;
  Lastangle=Aangle;
  Aangle=Roll;   //pitch angle
  // error is the Desired angle - actual angle
  error=Dangle-Aangle;
  
  // PID controller at 50 HZ
  COcommand=COcommand-Kp*(Aangle-Lastangle)+Ki/50*error-Kd*50*(Aangle-2*Lastangle+LLastangle);
  // constrain the Control low command to a valid range
  COcommand=constrain(COcommand, -255, 255);

   DMSR= (int)   COcommand + 0;    
   DMSL=  (int)   COcommand + 0;   
   SetMotorSpeed(DMSR, DMSL, true);
}

void MPUMath() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Yaw = (ypr[0] * 180.0 / M_PI);
  Pitch = (ypr[1] *  180.0 / M_PI);
  Roll = (ypr[2] *  180.0 / M_PI);
  
  Mcommand();
  
  DPRINTSTIMER(100) {
//    DPRINTSFN(10, "\tYaw:", Yaw, 6, 1);
//    DPRINTSFN(10, "\tPitch:", Pitch, 6, 1);
    DPRINTSFN(10, "\tRoll:", Roll, 6, 1);
    DPRINTSFN(10, "\tDMSL:", DMSL, 6, 1);
    DPRINTSFN(10, "\tError:", error, 6, 1);
Serial.print("\t");
Serial.print(dir);   
    DPRINTLN();
  }
}


// ================================================================
// ===                         Setup                            ===
// ================================================================
void setup() {

  Serial.begin(115200); //115200
  while (!Serial);
  Serial.println("i2cSetup");
  i2cSetup();
  Serial.println("MPU6050Connect");
  MPU6050Connect();
  Serial.println("Setup complete");
  pinMode(LED_PIN, OUTPUT);

  // declare direction pins as aoutup
  pinMode(MdirectionR1, OUTPUT);
  
  pinMode(MdirectionL1, OUTPUT);
  

  pinMode(MpwmBL, OUTPUT);
  pinMode(MpwmBR, OUTPUT);

}
 
// ================================================================
// ===                          Loop                            ===
// ================================================================
void loop() 
{
  if (mpuInterrupt ) // wait for MPU interrupt or extra packet(s) available
  { 
     if(Roll < 0)         //Roll is negative for moving forward. If Roll becomes positive change it to Roll > 0
     {
      int Left_speed =0,Right_speed = 0;
      int Pot = analogRead(Pot_pin);
      if(Pot < 505 || Pot > 518)      //If handle is bent left or right
      {
        
        if(Pot < 512)      //Turn left
        {
          int turn_speed = map(Pot,255,511,150,0);      //Turn speed mapped according to Pot
          int temp_right = DMSR + turn_speed;
          Right_speed = constrain(temp_right, 0,255);
          int temp_left = DMSL - turn_speed;
          Left_speed = constrain(temp_left,0,255);
          digitalWrite(MdirectionL1, HIGH);
          digitalWrite(MdirectionR1, LOW);
         /*if(Roll >0)
         {
          digitalWrite(MdirectionL1,LOW);
          digitalWrite(MdirectionR1,HIGH);   
         }*/
         analogWrite(MpwmBR, Right_speed);
         analogWrite(MpwmBL, Left_speed);
        }
        else if(Pot > 512)      //Turn Right
        {
          int turn_speed = map(Pot, 512,768,0,150);
          int temp_right = DMSR - turn_speed;
          Right_speed = constrain(temp_right, 0,255);
          int temp_left = DMSL + turn_speed;
          Left_speed = constrain(temp_left,0,255);
          digitalWrite(MdirectionL1, LOW);
          digitalWrite(MdirectionR1,HIGH);
         /*if(Roll >0)
         {
          digitalWrite(MdirectionL1,HIGH);
          digitalWrite(MdirectionR1,LOW);   
         }*/
          analogWrite(MpwmBL, Left_speed);
          analogWrite(MpwmBR, Right_speed);
        }
        else
          ;
      }
     }
    GetDMP();
  }
//   mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

}
