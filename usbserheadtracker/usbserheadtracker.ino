#include <Wire.h>

// requires package MPU6050
#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0
// = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
int16_t ax, ay, az;
int16_t gx, gy, gz;
double gx0 = 0;
double gy0 = 0;
double gz0 = 0;
float rotx = 0;
float roty = 0;
float rotz = 0;
float rotscale = 0.0609756;
int32_t qw32,qx32,qy32,qz32;
int32_t rotx32,roty32,rotz32;

Quaternion q;
VectorInt16 aa;  // [x, y, z]            accel sensor measurements
VectorInt16
  aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16
  aaWorld;            // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector

unsigned long rt_uint = 0;
unsigned long rt_uint_prev = 0;
unsigned long dt_uint = 0;
double rt_double = 0;
double dt_double = 0;

bool b_calibrating = false;
bool b_calibinit = true;

uint16_t calib_cnt = 0;

void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment
  // this line if having compilation difficulties
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(187);
  mpu.setYGyroOffset(25);
  mpu.setZGyroOffset(-3);
  mpu.setZAccelOffset(1688);  // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    // set our DMP Ready flag so the main loop() function knows it's okay to use
    // it
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

void loop() {
  if (!dmpReady)
    return;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  // check for overflow (this should never happen unless our code is too
  // inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    // otherwise, check for DMP data ready interrupt (this should happen
    // frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO, then clear the buffer
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    // calculate period time:
    rt_uint = micros();
    dt_uint = rt_uint - rt_uint_prev;
    rt_uint_prev = rt_uint;
    rt_double = 1e-6 * rt_uint;
    dt_double = 1e-6 * dt_uint;
    if (b_calibinit || (digitalRead(15) && (!b_calibrating))) {
      b_calibinit = false;
      b_calibrating = true;
      Serial.println("C1");
      calib_cnt = 400;
      mpu.setXGyroOffset(0);
      mpu.setYGyroOffset(0);
      mpu.setZGyroOffset(0);
      gx0 = 0;
      gy0 = 0;
      gz0 = 0;
    }
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    if (calib_cnt) {
      gx0 += gx;
      gy0 += gy;
      gz0 += gz;
      calib_cnt--;
      if (calib_cnt == 0) {
        b_calibrating = false;
        gx0 /= 400.0;
        gy0 /= 400.0;
        gz0 /= 400.0;
        rotx = 0;
        roty = 0;
        rotz = 0;
        // by some reason unknown to me, the offset has to be scaled by -2 to achieve correct values:
        mpu.setXGyroOffset(-2 * gx0);
        mpu.setYGyroOffset(-2 * gy0);
        mpu.setZGyroOffset(-2 * gz0);
        Serial.println("C0");
        Serial.print('O');
        Serial.print(mpu.getXGyroOffset());
        Serial.print(',');
        Serial.print(mpu.getYGyroOffset());
        Serial.print(',');
        Serial.print(mpu.getZGyroOffset());
        Serial.print(',');
        Serial.print(gx0);
        Serial.print(',');
        Serial.print(gy0);
        Serial.print(',');
        Serial.println(gz0);
      }
    } else {
      if ((dt_uint > 0)&&(dt_uint < (1<<30))) {
        rotx += gx * dt_double * rotscale;
        roty += gy * dt_double * rotscale;
        rotz += gz * dt_double * rotscale;
      }
    }
    if ((dt_uint > 0)&&(dt_uint < (1<<30))) {
      qw32 = (1<<16)*q.w;
      qx32 = (1<<16)*q.x;
      qy32 = (1<<16)*q.y;
      qz32 = (1<<16)*q.z;
      rotx32 = (1<<7)*rotx;
      roty32 = (1<<7)*roty;
      rotz32 = (1<<7)*rotz;
      Serial.write("TSCH",4);
      Serial.write((char*)(&rt_uint),sizeof(rt_uint));
      Serial.write((char*)(&(qw32)),sizeof(qw32));
      Serial.write((char*)(&(qx32)),sizeof(qx32));
      Serial.write((char*)(&(qy32)),sizeof(qy32));
      Serial.write((char*)(&(qz32)),sizeof(qz32));
      Serial.write((char*)(&rotx32),sizeof(rotx32));
      Serial.write((char*)(&roty32),sizeof(roty32));
      Serial.write((char*)(&rotz32),sizeof(rotz32));
    }
  }
}

/*
   Local Variables:
   mode: c++
   c-basic-offset: 2
   End:
*/
