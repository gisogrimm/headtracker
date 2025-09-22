#include <OSCBoards.h>
#include <OSCData.h>
#include <OSCMatch.h>
#include <OSCMessage.h>
#include <OSCTiming.h>

// use ESP8266WiFi.h on ESP8266 board, e.g., Wemos D1 R2 & mini:
#include <ESP8266WiFi.h>
// use WiFi.h on ESP32 board:
//#include <WiFi.h>
#include <WiFiUdp.h>
#include <ADS1115_WE.h>
#include <Wire.h>

#define DEBUG
#include <MPU6050_6Axis_MotionApps20.h>

//#include "ads1115_glab.h"
double lsbsize = 0.0000078125;  // Volt/bit

MPU6050 mpu;

uint16_t remote_port = 0;
uint16_t remote_port_eog = 0;
uint16_t remote_port_raw = 0;
IPAddress next_remote_ip;
IPAddress remote_ip;
IPAddress remote_ip_eog;
IPAddress remote_ip_raw;
char remote_path[1024];
char remote_path_eog[1024];
char remote_path_raw[1024];
char remote_path_accel[1024];
char remote_path_accelLinear[1024];
char remote_path_accelLinearInWorld[1024];
int adc_mode = 3;

char ext_WLAN_SSID[1024];
char ext_WLAN_pass[1024];
char ext_IP[1024];

OSCMessage msg_data("/headtrack");
OSCMessage msg_eog("/eog");
OSCMessage msg_raw("/raw");
OSCMessage msg_dmpAccel("/accel");
OSCMessage msg_dmpAccelLinear("/accelLinear");
OSCMessage msg_dmpAccelLinearInWorld("/accelLinearInWorld");

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
double rotx = 0;
double roty = 0;
double rotz = 0;
double rotscale = 0.0609756;

Quaternion q;
VectorInt16 aa;  // [x, y, z]            accel sensor measurements
VectorInt16
  aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16
  aaWorld;            // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector

double rt = 0;
double rtp = 0;
double dt = 0;

bool b_calibrating = false;
bool b_calibinit = true;

uint16_t calib_cnt = 0;

WiFiUDP Udp;

#define I2C_ADDRESS 0x48
ADS1115_WE adc1 = ADS1115_WE(I2C_ADDRESS);
ADS1115_WE adc2 = ADS1115_WE(I2C_ADDRESS + 1);

byte mac[6];

char ssid[32];                //  your network SSID (name)
char pass[] = "headtracker";  // your network password
IPAddress local_IP(192, 168, 100, 1);
IPAddress gateway(192, 168, 100, 1);
IPAddress subnet(255, 255, 255, 0);

void setup() {
  msg_data.add(1.0).add(1.0f).add(1.0f).add(1.0f).add(1.0f).add(1.0f).add(1.0f).add(1.0f);
  msg_eog.add(1.0).add(1.0f).add(1.0f);
  msg_raw.add(1.0).add(1).add(1).add(1).add(1).add(1).add(1);
  msg_dmpAccel.add(1.0).add(1.0f).add(1.0f).add(1.0f);
  msg_dmpAccelLinear.add(1.0).add(1.0f).add(1.0f).add(1.0f);
  msg_dmpAccelLinearInWorld.add(1.0).add(1.0f).add(1.0f).add(1.0f);
  WiFi.macAddress(mac);
  sprintf(ssid, "head-%02x%02x%02x%02x", mac[2], mac[3], mac[4], mac[5]);
  WiFi.mode(WIFI_AP);
  WiFi.disconnect();
  delay(500);
  WiFi.softAP(ssid, pass);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  Udp.begin(9999);

  pinMode(15, INPUT);
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
  adc1.init();
  adc2.init();
  adc1.setAlertPinMode(ADS1115_DISABLE_ALERT);
  adc2.setAlertPinMode(ADS1115_DISABLE_ALERT);
  
  adc1.setVoltageRange_mV(ADS1115_RANGE_0256);
  adc2.setVoltageRange_mV(ADS1115_RANGE_0256);
  adc1.setConvRate(ADS1115_64_SPS);
  adc2.setConvRate(ADS1115_64_SPS);
  adc1.setMeasureMode(ADS1115_CONTINUOUS);
  adc2.setMeasureMode(ADS1115_CONTINUOUS);
  adc1.setCompareChannels(ADS1115_COMP_2_3);
  adc2.setCompareChannels(ADS1115_COMP_2_3);
}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc1.setCompareChannels(channel);
  adc1.startSingleMeasurement();
  while (adc1.isBusy()) {}
  voltage = adc1.getResult_V();  // alternative: getResult_mV for Millivolt
  return voltage;
}

void osc_connect_wlan(OSCMessage &msg) {
  // optionally: disconnect from WLAN, create AP:
  if (msg.size() == 0) {
    WiFi.mode(WIFI_AP);
    WiFi.disconnect();
    delay(500);
    WiFi.softAP(ssid, pass);
    WiFi.softAPConfig(local_IP, gateway, subnet);
  }
  // parameter: SSID, password, target IP address, port number
  if ((msg.size() == 4) && msg.isString(0) && msg.isString(1) && msg.isString(2) && msg.isInt(3)) {
    msg.getString(0, ext_WLAN_SSID);
    msg.getString(1, ext_WLAN_pass);
    msg.getString(2, ext_IP);
    int port = msg.getInt(3);
    if (strlen(ext_WLAN_SSID) == 0) {
      WiFi.mode(WIFI_AP);
      WiFi.disconnect();
      delay(500);
      WiFi.softAP(ssid, pass);
      WiFi.softAPConfig(local_IP, gateway, subnet);
    } else {
      WiFi.mode(WIFI_STA);
      WiFi.disconnect();
      delay(500);
      WiFi.begin(ext_WLAN_SSID, ext_WLAN_pass);
      remote_port = port;
      remote_port_eog = port;
      remote_port_raw = port;
      sprintf(remote_path, "/%02x%02x%02x%02x/quatrot", mac[2], mac[3], mac[4], mac[5]);
      msg_data.setAddress(remote_path);
      sprintf(remote_path_eog, "/%02x%02x%02x%02x/eog", mac[2], mac[3], mac[4], mac[5]);
      msg_eog.setAddress(remote_path_eog);
      sprintf(remote_path_raw, "/%02x%02x%02x%02x/raw", mac[2], mac[3], mac[4], mac[5]);
      msg_raw.setAddress(remote_path_raw);
      sprintf(remote_path_accel, "/%02x%02x%02x%02x/accel", mac[2], mac[3], mac[4], mac[5]);
      msg_dmpAccel.setAddress(remote_path_accel);
      sprintf(remote_path_accelLinear, "/%02x%02x%02x%02x/accelLinear", mac[2], mac[3], mac[4], mac[5]);
      msg_dmpAccelLinear.setAddress(remote_path_accelLinear);
      sprintf(remote_path_accelLinearInWorld, "/%02x%02x%02x%02x/accelLinearInWorld", mac[2], mac[3], mac[4], mac[5]);
      msg_dmpAccelLinearInWorld.setAddress(remote_path_accelLinearInWorld);
      remote_ip.fromString(ext_IP);
      remote_ip_eog.fromString(ext_IP);
      remote_ip_raw.fromString(ext_IP);
    }
  }
}

void osc_connect(OSCMessage &msg) {
  if ((msg.size() == 2) && msg.isInt(0) && msg.isString(1)) {
    remote_port = msg.getInt(0);
    remote_ip = next_remote_ip;
    msg.getString(1, remote_path);
    msg_data.setAddress(remote_path);
  }
}

void osc_connect_eog(OSCMessage &msg) {
  if ((msg.size() == 2) && msg.isInt(0) && msg.isString(1)) {
    remote_port_eog = msg.getInt(0);
    remote_ip_eog = next_remote_ip;
    msg.getString(1, remote_path_eog);
    msg_eog.setAddress(remote_path_eog);
  }
}

void osc_connect_raw(OSCMessage &msg) {
  if ((msg.size() == 2) && msg.isInt(0) && msg.isString(1)) {
    remote_port_raw = msg.getInt(0);
    remote_ip_raw = next_remote_ip;
    msg.getString(1, remote_path_raw);
    msg_raw.setAddress(remote_path_raw);
    msg_dmpAccel.setAddress("/dmp/accel");
    msg_dmpAccelLinear.setAddress("/dmp/accelLinear");
    msg_dmpAccelLinearInWorld.setAddress("/dmp/accelLinearInWorld");
  }
}

void osc_disconnect(OSCMessage &msg) {
  remote_port = 0;
}

void osc_disconnect_eog(OSCMessage &msg) {
  remote_port_eog = 0;
}

void osc_disconnect_raw(OSCMessage &msg) {
  remote_port_raw = 0;
}

void osc_set_srate(OSCMessage &msg) {
  if ((msg.size() == 1) && msg.isInt(0)) {
    int srate = msg.getInt(0);
    if (srate < 16) {
      adc1.setConvRate(ADS1115_8_SPS);
      adc2.setConvRate(ADS1115_8_SPS);
    } else if (srate < 32) {
      adc1.setConvRate(ADS1115_16_SPS);
      adc2.setConvRate(ADS1115_16_SPS);
    } else if (srate < 64) {
      adc1.setConvRate(ADS1115_32_SPS);
      adc2.setConvRate(ADS1115_32_SPS);
    } else if (srate < 128) {
      adc1.setConvRate(ADS1115_64_SPS);
      adc2.setConvRate(ADS1115_64_SPS);
    } else if (srate < 250) {
      adc1.setConvRate(ADS1115_128_SPS);
      adc2.setConvRate(ADS1115_128_SPS);
    } else if (srate < 475) {
      adc1.setConvRate(ADS1115_250_SPS);
      adc2.setConvRate(ADS1115_250_SPS);
    } else if (srate < 860) {
      adc1.setConvRate(ADS1115_475_SPS);
      adc2.setConvRate(ADS1115_475_SPS);
    } else {
      adc1.setConvRate(ADS1115_860_SPS);
      adc2.setConvRate(ADS1115_860_SPS);
    }
  }
}

void osc_set_urange(OSCMessage &msg) {
  if ((msg.size() == 1) && msg.isFloat(0)) {
    float rg = msg.getFloat(0);
    ADS1115_RANGE RG = ADS1115_RANGE_0256;
    if (rg < 0.257)
      RG = ADS1115_RANGE_0256;
    else if (rg < 0.513)
      RG = ADS1115_RANGE_0512;
    else if (rg < 1.025)
      RG = ADS1115_RANGE_1024;
    adc1.setVoltageRange_mV(RG);
    adc2.setVoltageRange_mV(RG);
  }
}

void osc_set_adcmode(OSCMessage &msg) {
  if ((msg.size() == 1) && msg.isInt(0)) {
    adc_mode = msg.getInt(0);
    if (adc_mode > 2)
      adc_mode = 2;
    adc1.init();
    adc2.init();
    adc1.setVoltageRange_mV(ADS1115_RANGE_0256);
    adc2.setVoltageRange_mV(ADS1115_RANGE_0256);
    adc1.setConvRate(ADS1115_64_SPS);
    adc2.setConvRate(ADS1115_64_SPS);
    adc1.setMeasureMode(ADS1115_CONTINUOUS);
    adc2.setMeasureMode(ADS1115_CONTINUOUS);
    adc1.setCompareChannels(ADS1115_COMP_2_3);
    adc2.setCompareChannels(ADS1115_COMP_2_3);
    switch (adc_mode) {
      case 0:
        adc1.setMeasureMode(ADS1115_SINGLE);
        break;
      case 1:
        adc1.setMeasureMode(ADS1115_CONTINUOUS);
        adc1.setCompareChannels(ADS1115_COMP_0_1);
        break;
      case 2:
        adc1.setMeasureMode(ADS1115_CONTINUOUS);
        adc1.setCompareChannels(ADS1115_COMP_2_3);
        break;
      case 3:
        adc1.setMeasureMode(ADS1115_CONTINUOUS);
        adc2.setMeasureMode(ADS1115_CONTINUOUS);
        adc1.setCompareChannels(ADS1115_COMP_2_3);
        adc2.setCompareChannels(ADS1115_COMP_2_3);
        break;
    }
  }
}

void osc_calib(OSCMessage &msg) {
  b_calibinit = true;
}

void proc_osc() {
  int size = Udp.parsePacket();
  if (size > 0) {
    OSCMessage msg;
    while (size--) {
      msg.fill(Udp.read());
      next_remote_ip = Udp.remoteIP();
    }
    if (!msg.hasError()) {
      msg.dispatch("/connect", osc_connect);
      msg.dispatch("/disconnect", osc_disconnect);
      msg.dispatch("/wlan/connect", osc_connect_wlan);
      msg.dispatch("/eog/srate", osc_set_srate);
      msg.dispatch("/eog/adcmode", osc_set_adcmode);
      msg.dispatch("/eog/urange", osc_set_urange);
      msg.dispatch("/eog/connect", osc_connect_eog);
      msg.dispatch("/eog/disconnect", osc_disconnect_eog);
      msg.dispatch("/raw/connect", osc_connect_raw);
      msg.dispatch("/raw/disconnect", osc_disconnect_raw);
      msg.dispatch("/calib", osc_calib);
    }
  }
}

void send_msg() {
  if (remote_port > 0) {
    Udp.beginPacket(remote_ip, remote_port);
    msg_data.send(Udp);
    Udp.endPacket();
  }
}

void send_msg_eog() {
  if (remote_port_eog > 0) {
    Udp.beginPacket(remote_ip_eog, remote_port_eog);
    msg_eog.send(Udp);
    Udp.endPacket();
  }
}

void send_msg_raw() {
  if (remote_port_raw > 0) {
    Udp.beginPacket(remote_ip_raw, remote_port_raw);
    msg_raw.send(Udp);
    Udp.endPacket();
    Udp.beginPacket(remote_ip_raw, remote_port_raw);
    msg_dmpAccel.send(Udp);
    Udp.endPacket();
    Udp.beginPacket(remote_ip_raw, remote_port_raw);
    msg_dmpAccelLinear.send(Udp);
    Udp.endPacket();
    Udp.beginPacket(remote_ip_raw, remote_port_raw);
    msg_dmpAccelLinearInWorld.send(Udp);
    Udp.endPacket();
  }
}

void loop() {
  proc_osc();
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
    rt = 0.001 * millis();
    dt = rt - rtp;
    rtp = rt;
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
      if (dt > 0) {
        rotx += gx * dt * rotscale;
        roty += gy * dt * rotscale;
        rotz += gz * dt * rotscale;
      }
    }
    if (dt > 0) {
      msg_data.set(0, rt).set(1, q.w).set(2, q.x).set(3, q.y).set(4, q.z).set(5, rotx).set(6, roty).set(7, rotz);
      send_msg();
      if (remote_port_eog > 0) {
        //ads1115_configure(false);
        //delay(4);
        //float U1 = lsbsize * (double)ads1115_read();
        float U1 = 0;
        float U2 = 0;
        if (adc_mode == 0) {
          U1 = readChannel(ADS1115_COMP_0_1);
          U2 = readChannel(ADS1115_COMP_2_3);
        } else if (adc_mode == 3) {
          U1 = adc1.getResult_V();
          U2 = adc2.getResult_V();
        } else {
          U1 = adc1.getResult_V();
        }
        //ads1115_configure(true);
        //delay(4);
        //float U2 = lsbsize * (double)ads1115_read();
        msg_eog.set(0, rt).set(1, U1).set(2, U2);
        send_msg_eog();
      }
      if (remote_port_raw > 0) {
        msg_raw.set(0, rt).set(1, ax).set(2, ay).set(3, az).set(4, gx).set(5, gy).set(6, gz);
        msg_dmpAccel.set(0,rt).set(1,aa.x).set(2,aa.y).set(3,aa.z);
        msg_dmpAccelLinear.set(0,rt).set(1,aaReal.x).set(2,aaReal.y).set(3,aaReal.z);
        msg_dmpAccelLinearInWorld.set(0,rt).set(1,aaWorld.x).set(2,aaWorld.y).set(3,aaWorld.z);
        send_msg_raw();
      }
    }
  }
}

/*
   Local Variables:
   mode: c++
   c-basic-offset: 2
   End:
*/
