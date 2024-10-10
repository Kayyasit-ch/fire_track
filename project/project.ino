#include <Wire.h>
#include <NewPing.h>
#include <Servo.h>
#include <avr/interrupt.h>

// กำหนดพินสำหรับเซนเซอร์และมอเตอร์
#define TRIGGER_PIN  9    
#define ECHO_PIN     8   
#define MAX_DISTANCE 200

#define MF_1         13   // มอเตอร์ 1 เดินหน้า (IN1)
#define MB_1         12   // มอเตอร์ 1 ถอยหลัง (IN2)
#define MF_2         11   // มอเตอร์ 2 เดินหน้า (IN3)
#define MB_2         10   // มอเตอร์ 2 ถอยหลัง (IN4)
#define SWITCH_PIN   3    
#define FLAME_PIN    2    // เซนเซอร์ตรวจจับไฟ (INT0)
#define RELAY_PIN    A0   // รีเลย์
#define ENA_PIN      3    // PWM ควบคุมความเร็ว มอเตอร์ 1
#define ENB_PIN      5    // PWM ควบคุมความเร็ว มอเตอร์ 2
#define POT_PIN      A2   // โพเทนชิโอมิเตอร์ปรับความเร็ว

// การสื่อสาร I2C
#define ARDUINO_ADDRESS 8

// สร้างอ็อบเจกต์
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo myservo; // เซอร์โวสำหรับควบคุม
Servo myservo2;  // เซอร์โวตัวที่สอง

// ตัวแปรต่างๆ

bool inMode3 = false; // ตัวแปรสำหรับการตรวจสอบโหมด 3
int state = 0; // สถานะการทำงานของหุ่นยนต์
int maxSensor = 30; // ค่าตั้งต้นสูงสุดสำหรับเซนเซอร์
volatile int mode = 1;  // เริ่มต้นในโหมด 1 (Sleep)
char previousCommand = 'S';  // เริ่มต้นคำสั่งเป็น 'S' สำหรับหยุด
int servo2Pos = 90;   // ตำแหน่งเริ่มต้นของเซอร์โว 2
int servo2Step = 2;  // กำหนดการเคลื่อนไหวของเซอร์โว 2
volatile bool flameDetectedFlag = false;  // ธงสำหรับตรวจจับไฟ

// ตัวแปรควบคุมเวลา
unsigned long previousMillis = 0;
const long interval = 20; // ช่วงเวลาสำหรับเซอร์โว 2

// ฟังก์ชันรีเซ็ต Watchdog Timer

void setup() {
  Serial.begin(9600); // เริ่มต้นการสื่อสาร Serial
  myservo.attach(7); // เชื่อมต่อเซอร์โวที่พิน 7
  myservo2.attach(4);  // เชื่อมต่อเซอร์โว 2 ที่พิน 4

  // ตั้งค่า INPUT/OUTPUT
  pinMode(MF_1, OUTPUT); // ตั้งค่าพินสำหรับมอเตอร์ 1
  pinMode(MB_1, OUTPUT); // ตั้งค่าพินสำหรับมอเตอร์ 1 ถอยหลัง
  pinMode(MF_2, OUTPUT); // ตั้งค่าพินสำหรับมอเตอร์ 2
  pinMode(MB_2, OUTPUT); // ตั้งค่าพินสำหรับมอเตอร์ 2 ถอยหลัง
  pinMode(SWITCH_PIN, INPUT_PULLUP); // ตั้งค่าพินสำหรับสวิตช์
  pinMode(RELAY_PIN, OUTPUT); // ตั้งค่าพินสำหรับรีเลย์
  pinMode(FLAME_PIN, INPUT); // ตั้งค่าพินสำหรับเซนเซอร์ไฟ
  pinMode(ENA_PIN, OUTPUT); // ตั้งค่าพินสำหรับ PWM มอเตอร์ 1
  pinMode(ENB_PIN, OUTPUT); // ตั้งค่าพินสำหรับ PWM มอเตอร์ 2
  pinMode(POT_PIN, INPUT); // ตั้งค่าพินสำหรับโพเทนชิโอมิเตอร์

  // ปิดรีเลย์ตั้งแต่เริ่มต้น
  digitalWrite(RELAY_PIN, HIGH);     
  myservo.write(90);  // กำหนดตำแหน่งเริ่มต้นของเซอร์โว
  myservo2.write(servo2Pos);  

  // เปิดใช้งาน interrupt สำหรับเซนเซอร์ไฟ
  sei(); // เปิดใช้งาน global interrupts
  EIMSK |= (1 << INT0);  // เปิดใช้งาน interrupt 0
  EICRA &= ~(1 << ISC01); // ตั้งค่าการทำงานของ interrupt
  EICRA &= ~(1 << ISC00);  

  // เปิดใช้งานการสื่อสาร I2C
  Wire.begin(ARDUINO_ADDRESS); // เริ่มต้น I2C
  Wire.onReceive(receiveData); // ตั้งค่าฟังก์ชันสำหรับรับข้อมูล

  // เปิดใช้งาน Watchdog Timer
}

// ฟังก์ชันจัดการการตรวจจับเปลวไฟ
ISR(INT0_vect) {  
  int flameState = digitalRead(FLAME_PIN); // อ่านสถานะเซนเซอร์ไฟ
  
  // ถ้าตรวจจับเปลวไฟและไม่อยู่ในโหมด 3
  if (flameState == LOW && mode != 3) {  
    digitalWrite(RELAY_PIN, LOW);  // เปิดรีเลย์
    stop();  // หยุดมอเตอร์
    digitalWrite(RELAY_PIN, HIGH);  // ปิดรีเลย์
  }
}

// ฟังก์ชันควบคุมความเร็วของมอเตอร์
void setMotorSpeed() {
  analogWrite(ENA_PIN, 80);  // กำหนดความเร็วมอเตอร์ 1
  analogWrite(ENB_PIN, 80);  // กำหนดความเร็วมอเตอร์ 2
}

// ฟังก์ชันการเคลื่อนไหวของหุ่นยนต์
void font() {
  digitalWrite(MF_1, HIGH); // มอเตอร์ 1 เดินหน้า
  digitalWrite(MB_1, LOW);
  digitalWrite(MF_2, HIGH); // มอเตอร์ 2 เดินหน้า
  digitalWrite(MB_2, LOW);
}

void back() {
  digitalWrite(MF_1, LOW); // มอเตอร์ 1 ถอยหลัง
  digitalWrite(MB_1, HIGH);
  digitalWrite(MF_2, LOW); // มอเตอร์ 2 ถอยหลัง
  digitalWrite(MB_2, HIGH);
}

void lift() {
  analogWrite(ENA_PIN, 90);  // กำหนดความเร็วมอเตอร์ 1
  analogWrite(ENB_PIN, 90);
  digitalWrite(MF_1, HIGH); // ยกหุ่นยนต์
  digitalWrite(MB_1, LOW);
  digitalWrite(MF_2, LOW);
  digitalWrite(MB_2, HIGH);
}

void right() {
  analogWrite(ENA_PIN, 150);  // กำหนดความเร็วมอเตอร์ 1
  analogWrite(ENB_PIN, 150);
  digitalWrite(MF_1, LOW); // เลี้ยวขวา
  digitalWrite(MB_1, HIGH);
  digitalWrite(MF_2, HIGH);
  digitalWrite(MB_2, LOW);
}

void stop() {
  digitalWrite(MF_1, LOW); // หยุดมอเตอร์
  digitalWrite(MB_1, LOW);
  digitalWrite(MF_2, LOW);
  digitalWrite(MB_2, LOW);
  setMotorSpeed(); // ตั้งค่าความเร็วมอเตอร์
}

// ฟังก์ชันรับข้อมูล I2C
void receiveData(int byteCount) {
  while (Wire.available()) { // ตรวจสอบว่ามีข้อมูลอยู่ในบัฟเฟอร์
    char command = Wire.read(); // อ่านข้อมูล
    
    // ถ้าคำสั่งคือ 1, 2 หรือ 3 ให้เปลี่ยนโหมด
    if (command == '1' || command == '2' || command == '3') {
      mode = command - '0';  // เปลี่ยนโหมด 
    } else {
      previousCommand = command;  // เก็บคำสั่งก่อนหน้า
    }
  }
}

// ฟังก์ชันหลัก
void loop() {
  // อ่านค่าที่รับได้
  receiveData(0); // เรียกใช้ฟังก์ชันเพื่ออ่านข้อมูลที่เข้ามา

  unsigned long currentMillis = millis(); // เก็บเวลาในปัจจุบัน
  int Sona = sonar.ping_cm();  // อ่านค่าจากเซนเซอร์ระยะ

  setMotorSpeed();  // กำหนดความเร็วของมอเตอร์

  // ถ้าไม่มีการอ่านค่าจากเซนเซอร์ ให้กำหนดระยะทางเป็นค่ามากที่สุด
  if (Sona == 0) {
    Sona = MAX_DISTANCE;  
  }

  switch (mode) {
    case 1:  // โหมด 1: Sleep
      stop();  // หยุดมอเตอร์
      break;

    case 2:  // โหมด 2: ทำงานอัตโนมัติ
      if (currentMillis - previousMillis >= interval) { // ตั้งเวลาให้เซอร์โว
        previousMillis = currentMillis;
        servo2Pos += servo2Step; // ปรับตำแหน่งเซอร์โว
        // เปลี่ยนทิศทางเซอร์โวถ้าถึงตำแหน่งสุดท้าย
        if (servo2Pos >= 145 || servo2Pos <= 25) {
          servo2Step = -servo2Step;  
        }
        myservo2.write(servo2Pos);  // กำหนดตำแหน่งเซอร์โว
      }

      switch (state) {
        case 0: // สถานะ 0: ตรวจสอบระยะ
          if (Sona > maxSensor) { // ถ้าระยะมากกว่าค่าตั้งต้น
            font(); // มอเตอร์เดินหน้า
          } else {
            stop(); // หยุด
            delay(500);
            myservo.write(0); // หมุนเซอร์โว
            delay(1000);
            state = 1; // เปลี่ยนสถานะเป็น 1
          }
          break;

        case 1: // สถานะ 1: ย้ายไปยังตำแหน่ง
          if (Sona >= maxSensor) {
            state = 2; // เปลี่ยนสถานะเป็น 2
          } else {
            myservo.write(180); // หมุนเซอร์โวไปที่ 180 องศา
            delay(1000);
            state = 3; // เปลี่ยนสถานะเป็น 3
          }
          break;

        case 2: // สถานะ 2: ถอยหลังและยก
          if (Sona >= maxSensor) {
            back(); // ถอยหลัง
            delay(200);
            lift(); // ยก
            delay(600);
            myservo.write(90); // คืนตำแหน่งเซอร์โว
            state = 0; // เปลี่ยนสถานะเป็น 0
          }
          break;

        case 3: // สถานะ 3: ถอยหลังและเลี้ยวขวา
          if (Sona >= maxSensor) {
            back(); // ถอยหลัง
            delay(200);
            right(); // เลี้ยวขวา
            delay(600);
            myservo.write(90); // คืนตำแหน่งเซอร์โว
            state = 0; // เปลี่ยนสถานะเป็น 0
          } else {
            back(); // ถอยหลัง
            delay(2000);
            lift(); // ยก
            delay(600);
          }
          break;
      }
      break;

    case 3:  // โหมด 3: ควบคุมด้วยคำสั่ง
      EIMSK &= ~(1 << INT0);  // ปิดการใช้ interrupt สำหรับเซนเซอร์ไฟ
      myservo2.write(90); // คืนตำแหน่งเซอร์โว 2

      switch (previousCommand) {
        case 'F':
          font(); // มอเตอร์เดินหน้า
          break;
        case 'B':
          back(); // มอเตอร์ถอยหลัง
          break;
        case 'R':
          right(); // มอเตอร์เลี้ยวขวา
          break;
        case 'L':
          lift(); // ยก
          break;
        case 'P':
          digitalWrite(RELAY_PIN, LOW);  // เปิดรีเลย์
          break;
        default:
          stop(); // หยุด
          digitalWrite(RELAY_PIN, HIGH);  // ปิดรีเลย์
          break;
      }

      EIMSK |= (1 << INT0);  // เปิดใช้งาน interrupt อีกครั้ง
      break;
  }
}