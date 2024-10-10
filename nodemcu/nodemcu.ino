#define BLYNK_TEMPLATE_ID "TMPL6JCiytuQm"
#define BLYNK_TEMPLATE_NAME "checkpoint2"
#define BLYNK_AUTH_TOKEN "wdrW8_b1d7mnZyL_84XsoC134A-shWEi"

// รวมไลบรารีที่จำเป็น
#include <BlynkSimpleEsp8266.h> // ไลบรารี Blynk สำหรับ ESP8266
#include <Wire.h> // ไลบรารีสำหรับการสื่อสาร I2C

// กำหนดค่า auth สำหรับ Blynk
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Redmi"; // ชื่อเครือข่าย Wi-Fi
char pass[] = "1111122222"; // รหัสผ่าน Wi-Fi

// ที่อยู่ I2C สำหรับ Arduino
#define ARDUINO_ADDRESS 8

// สถานะของปุ่ม
bool forwardButtonState = false; // สถานะปุ่มเดินหน้า
bool backwardButtonState = false; // สถานะปุ่มถอยหลัง
bool rightButtonState = false; // สถานะปุ่มเลี้ยวขวา
bool leftButtonState = false; // สถานะปุ่มเลี้ยวซ้าย
bool relayButtonState = false; // สถานะปุ่ม Relay
bool modeButtonState = false; // สถานะปุ่มโหมด

void setup() {
  Serial.begin(115200); // เปิด Serial สำหรับ Debugging
  Wire.begin(); // เริ่มต้นการสื่อสาร I2C
  Blynk.begin(auth, ssid, pass); // เชื่อมต่อกับ Blynk
}

// ฟังก์ชันสำหรับควบคุมรถไปข้างหน้า (ส่งค่า 1)
BLYNK_WRITE(V0) {
  int value = param.asInt(); // อ่านค่าที่ส่งมาจาก Blynk
  forwardButtonState = (value == 1); // กำหนดสถานะปุ่มเดินหน้า
}

// ฟังก์ชันสำหรับถอยหลัง (ส่งค่า 2)
BLYNK_WRITE(V1) {
  int value = param.asInt(); // อ่านค่าที่ส่งมาจาก Blynk
  backwardButtonState = (value == 1); // กำหนดสถานะปุ่มถอยหลัง
}

// ฟังก์ชันสำหรับเลี้ยวขวา (ส่งค่า 3)
BLYNK_WRITE(V2) {
  int value = param.asInt(); // อ่านค่าที่ส่งมาจาก Blynk
  rightButtonState = (value == 1); // กำหนดสถานะปุ่มเลี้ยวขวา
}

// ฟังก์ชันสำหรับเลี้ยวซ้าย (ส่งค่า 4)
BLYNK_WRITE(V3) {
  int value = param.asInt(); // อ่านค่าที่ส่งมาจาก Blynk
  leftButtonState = (value == 1); // กำหนดสถานะปุ่มเลี้ยวซ้าย
}

// ฟังก์ชันสำหรับควบคุม Relay (ส่งค่า 5)
BLYNK_WRITE(V4) {
  int value = param.asInt(); // อ่านค่าที่ส่งมาจาก Blynk
  relayButtonState = (value == 1); // กำหนดสถานะปุ่ม Relay
}

// ฟังก์ชันสำหรับควบคุมโหมด (ส่งค่า 0, 1, หรือ 2)
BLYNK_WRITE(V5) {
  int value = param.asInt(); // อ่านค่าที่ส่งมาจาก Blynk
  
  // ส่งค่าตามโหมดที่ถูกเลือก (0 = Mode 1, 1 = Mode 2, 2 = Mode 3)
  Wire.beginTransmission(ARDUINO_ADDRESS); // เริ่มต้นการส่งข้อมูล I2C
  
  if (value == 0) {
    Wire.write("1");  // ส่งค่า "1" สำหรับ Mode 1
  } else if (value == 1) {
    Wire.write("2");  // ส่งค่า "2" สำหรับ Mode 2
  } else if (value == 2) {
    Wire.write("3");  // ส่งค่า "3" สำหรับ Mode 3
  }
  
  Wire.endTransmission(); // จบการส่งข้อมูล I2C
}

void loop() {
  Blynk.run();  // รัน Blynk เพื่อให้สามารถควบคุมผ่านแอปได้

  // ส่งข้อมูลผ่าน I2C ตามสถานะของปุ่มที่กดค้าง
  Wire.beginTransmission(ARDUINO_ADDRESS); // เริ่มต้นการส่งข้อมูล I2C

  // ส่งค่าโหมดไปยัง Arduino หากปุ่มโหมดถูกกด
  if (modeButtonState) {
    Wire.write("M");  // ส่งค่า "M" เมื่อปุ่มโหมดถูกกด
  }

  if (forwardButtonState) {
    Wire.write("F");  // ส่งค่า "F" เมื่อปุ่มเดินหน้าโดนกดค้าง
  }

  if (backwardButtonState) {
    Wire.write("B");  // ส่งค่า "B" เมื่อปุ่มถอยหลังโดนกดค้าง
  }

  if (rightButtonState) {
    Wire.write("R");  // ส่งค่า "R" เมื่อปุ่มเลี้ยวขวาโดนกดค้าง
  }

  if (leftButtonState) {
    Wire.write("L");  // ส่งค่า "L" เมื่อปุ่มเลี้ยวซ้ายโดนกดค้าง
  }

  if (relayButtonState) {
    Wire.write("P");  // ส่งค่า "P" เมื่อปุ่ม Relay โดนกดค้าง
  }

  // ถ้าไม่มีปุ่มไหนโดนกด
  if (!forwardButtonState && !backwardButtonState && !rightButtonState && !leftButtonState && !relayButtonState && !modeButtonState) {
    Wire.write("S");  // ส่งค่า "S" เพื่อหยุดการเคลื่อนไหว
  }

  Wire.endTransmission(); // จบการส่งข้อมูล I2C
  
  delay(100); // ลดการส่งคำสั่งบ่อยเกินไป
}
