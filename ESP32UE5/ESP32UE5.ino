#include <Wire.h>
#include <stdbool.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <WiFi.h>
#include "ArduinoJson.h"
#include "AsyncUDP.h"

#define BNO055_DELAY_MS (100)
//bno055의 값의 변화딜레이가 적어야한다.

// WiFi Credentials
//컴퓨터에 연결된 와이파이 ssid 이름 과 비밀번호를 정해야한다.
#define WIFI_NETWORK "Jerrrrrrrrrrrrrrrrr"
#define WIFI_PASSWORD "***********"
#define WIFI_TIMEOUT_MS 20000

// UDP Server Details
// 무선 LAN 어댑터 Wi-Fi ip주소 가져와야함 
//즉, 컴퓨터또한 와이파이연결 , 연결된 와이파이의 주소를 가져와야하고 아래 서버에 주소를 붙여 넣어야 한다. 포트는 임시 포트 번호 설정

// wifi ip
#define SERVER "192.***.***.***"
#define PORT 3002


Adafruit_BNO055 sensor = Adafruit_BNO055();

float filterPOld, filterPNew = 0; // Precision variables, Derived in setup function

float thetaM; // Measured inclination based on accelerometer X axis.
float thetaFOld=0; // Filtered Old Value - Initial value is 0
float thetaFNew; // Filtered New Value, will derived and will replace the old value for next iteration

float phiM;
float phiFOld=0; // Filtered Old Value - Initial value is 0
float phiFNew; // Filtered New Value, will derived and will replace the old value for next iteration

unsigned long timePrevious; // Variable to store the old time in milliseconds.
float timeDiff; // variable to capture the time difference
float thetaG = 0; // Angular distance in terms of angle on x axis
float phiG = 0; // Angular distance in terms of angle on y axis

float roll = 0; // Complementary filter variable
float pitch = 0; // Complementary filter variable
float trust = 0.95; // Trust percentage for complementary filter on gyroscope

float yaw = 0; // Yaw measurement angle based on mangetometer 자력계 z축 방향회전
float pitchRad; // 쏠림기울임
float rollRad; // 좌우회전
float Xm; // x축 milli
float Ym; // y축 milli

String json; // Hold json string
AsyncUDP udpClient; // Async UDPClient object

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Connect to WiFI first
  connectToWiFi ();

  sensor.begin();
  delay(1000);
  int8_t temp=sensor.getTemp();
  sensor.setExtCrystalUse(true);
  // Derive the filter precision for the old and new value
  filterPOld = 0.95;// FILTER_PRECISION in percentage / 100;
  filterPNew = 1 - filterPOld;
  timePrevious = millis(); // Initializing the time in milliseconds
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Reading accelerometer data 
  imu::Vector<3> acc = sensor.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
  // Reading Gyroscope data
  imu::Vector<3> gyr = sensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // Reading magnetometer data
  imu::Vector<3> mag = sensor.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  
  uint8_t system, gyro, accel, mg = 0;
  sensor.getCalibration(&system, &gyro, &accel, &mg);
  // Calculating the inclinationation based on accelerometer data x axis
  thetaM=atan2(acc.x()/9.8,acc.z()/9.8)*180/3.14159265359;
  // Calculate the New theta value based on the measured theta value and the old value
  thetaFNew=filterPOld*thetaFOld + filterPNew*thetaM;
  // Calculating the the inclination based on accelerometer data y axis
  phiM=atan2(acc.y()/9.8,acc.z()/9.8)*180/3.14159265359;
  // Calculate the New phi value based on the measured phi value and the old value
  phiFNew=filterPOld*phiFOld + filterPNew*phiM;

  // Evaluate Angular acceleration based on gyro data
  // Time difference calculation (in sec)
  timeDiff = (millis() - timePrevious)/1000.0;
  timePrevious=millis(); // Resetting the time with current time

  thetaG = thetaG - gyr.y() * timeDiff;
  phiG = phiG + gyr.x() * timeDiff;

  // Complementary filter implementation roll estimation
  roll=(roll + gyr.x() * timeDiff) * trust + phiM * (1 - trust);
  // Complementary filter implementation pitch estimation
  pitch=(pitch - gyr.y() * timeDiff) * trust + thetaM * (1 - trust);
  // Yaw measurement base in the mangetometer direction
   // Yaw measurement base in the mangetometer direction
  rollRad=roll/180*3.14159265359;
  pitchRad=pitch/180*3.14159265359;
  Xm=mag.x()*cos(pitchRad)-mag.y()*sin(rollRad)*sin(pitchRad)+mag.z()*cos(rollRad)*sin(pitchRad);
  Ym=mag.y()*cos(rollRad)+mag.z()*sin(rollRad);
  yaw = atan2(Ym,Xm)*180/3.14159265359;

  Serial.print(system);
  Serial.print(",");
  Serial.print(accel);
  Serial.print(",");
  Serial.print(gyro);
  Serial.print(",");
  Serial.print(mg);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(yaw);
  // Prepare the JSON
  json = GetJSONString ("hand", pitch, roll, yaw);
  Serial.println(json);
  // Se4nd the data to the Server
  UDPSendData(json);

  thetaFOld=thetaFNew;
  phiFOld=phiFNew;
  
  delay(BNO055_DELAY_MS);

}

void connectToWiFi (){
  Serial.print ("Connecting to WiFI");
  WiFi.mode (WIFI_STA);
  WiFi.begin (WIFI_NETWORK, WIFI_PASSWORD);
  unsigned long startAttemptTime = millis ();
  while (WiFi.status () != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS){
    Serial.print (".");
    delay(100);
  }
  // TODO: Error Handling
  if (WiFi.status () != WL_CONNECTED){
    Serial.print ("Failed!");
  } else {
    Serial.print ("Connected!");
    Serial.print (WiFi.localIP());
  }
}

String GetJSONString(String sensorName, float pitch, float roll, float yaw) 
{
    StaticJsonDocument<1024> staticJsonDocument;
    staticJsonDocument["name"] = sensorName;
    staticJsonDocument["pitch"] = pitch;
    staticJsonDocument["roll"] = roll;
    staticJsonDocument["yaw"] = yaw;
    
    char docBuf[1024];
    serializeJson(staticJsonDocument, docBuf);

    return String(docBuf);
}

void UDPConnect()
{
    IPAddress ipAddress = IPAddress();
    ipAddress.fromString(SERVER);
    udpClient.connect(ipAddress, PORT);   
    Serial.println ("Server Connected");
}

void UDPSendData(String message)
{
    char charBuffer[1024];
    message.toCharArray(charBuffer, 1024);

    if (udpClient.connected()){
      udpClient.broadcastTo(charBuffer, PORT);
    } else {
      Serial.println ("Server Not Connected");
      UDPConnect();
    }
}
