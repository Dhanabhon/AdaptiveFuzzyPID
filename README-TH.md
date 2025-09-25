# Adaptive Fuzzy PID

Adaptive Fuzzy PID (AFPID) เป็นไลบรารีที่ครอบคลุมสำหรับเพิ่มประสิทธิภาพการควบคุมความเร็ว DC มอเตอร์ผ่านการปรับพารามิเตอร์อัจฉริยะ โดยการรวมการควบคุม PID แบบดั้งเดิมเข้ากับเทคนิคตรรกศาสตร์แบบคลุมเครือ AFPID จะปรับพารามิเตอร์การควบคุมแบบไดนามิกแบบเรียลไทม์ตามพฤติกรรมของระบบและลักษณะข้อผิดพลาด

## คุณสมบัติ

**การใช้งานที่สมบูรณ์**
- ตัวควบคุม adaptive fuzzy PID ที่ทำงานได้เต็มรูปแบบ
- ฟังก์ชันสมาชิกหลายประเภท (Triangle, Trapezoid, Gaussian, Bell-shaped, S-shaped, Z-shaped, Singleton)
- โหมดการอนุมานที่กำหนดได้ (Mamdani Max-Min, Max-Product, TSK, SAM)
- การปรับพารามิเตอร์แบบเรียลไทม์ตามข้อผิดพลาดและอัตราข้อผิดพลาด

**การกำหนดค่าที่ยืดหยุ่น**
- เวลาตัวอย่างที่ปรับได้ (1ms - 30s)
- ช่วงอินพุต/เอาต์พุตที่กำหนดได้
- ฟังก์ชันสมาชิกหลายประเภทสำหรับอินพุตและเอาต์พุต
- การตรวจสอบพารามิเตอร์อย่างครอบคลุม

**ประสิทธิภาพที่ปรับปรุง**
- การทำงานแบบไม่บล็อกโดยใช้ `millis()` ของ Arduino
- การป้องกัน anti-windup สำหรับเทอมอินทิเกรต
- การเปลี่ยนแปลงพารามิเตอร์แบบราบรื่น
- เอนจินการอนุมานแบบคลุมเครือที่มีประสิทธิภาพ

**เอกสารประกอบที่ดี**
- ตัวอย่างที่ครอบคลุมรวมอยู่ด้วย
- เอกสาร API ที่ละเอียด
- แนวทางปฏิบัติที่ดีและรูปแบบการใช้งาน

## เริ่มต้นใช้งานอย่างรวดเร็ว

### การติดตั้ง

1. ดาวน์โหลดไลบรารีเป็นไฟล์ ZIP
2. ใน Arduino IDE ไปที่ **Sketch** → **Include Library** → **Add .ZIP Library**
3. เลือกไฟล์ ZIP ที่ดาวน์โหลด
4. รวมไลบรารีในสเก็ตช์ของคุณ:

```cpp
#include <AdaptiveFuzzyPID.h>
```

### การใช้งานพื้นฐาน

```cpp
#include <AdaptiveFuzzyPID.h>

// สร้างตัวควบคุมพร้อมพารามิเตอร์ PID เริ่มต้น
AdaptiveFuzzyPID controller(2.0, 1.0, 0.1);

void setup() {
  // กำหนดค่าตัวควบคุม
  controller.setSampleTime(50);           // เวลาตัวอย่าง 50ms
  controller.setInputRange(-100, 100);    // ช่วงข้อผิดพลาด
  controller.setOutputRange(-255, 255);   // ช่วงเอาต์พุต PWM

  // ตั้งค่าประเภทฟังก์ชันสมาชิก
  controller.setMembershipFunctionInputType(Triangle);
  controller.setMembershipFunctionOutputType(Triangle);
  controller.setInferenceMode(MamdaniMaxMin);
}

void loop() {
  double setpoint = 100.0;  // ความเร็วที่ต้องการ
  double input = readCurrentSpeed(); // การอ่านเซนเซอร์ของคุณ

  // อัปเดตตัวควบคุมและรับเอาต์พุต
  double output = controller.update(setpoint, input);

  // ใช้เอาต์พุตกับมอเตอร์
  setMotorSpeed(output);

  delay(10);
}
```

## เอกสารอ้างอิง API

### Constructor
```cpp
AdaptiveFuzzyPID(double kp = 1.0, double ki = 0.0, double kd = 0.0);
```

### เมธอดการกำหนดค่า
```cpp
void setTunings(double kp, double ki, double kd);           // ตั้งค่าพารามิเตอร์ PID
void setSampleTime(unsigned long sampleTimeMs);            // ตั้งค่าช่วงการอัปเดต
void setInputRange(double min, double max);                // ตั้งค่าช่วงข้อผิดพลาด
void setOutputRange(double min, double max);               // ตั้งค่าช่วงเอาต์พุต
void setMembershipFunctionInputType(MembershipFunctionType type);
void setMembershipFunctionOutputType(MembershipFunctionType type);
void setInferenceMode(InferenceMode mode);
```

### เมธอดควบคุม
```cpp
double update(double setpoint, double input);              // ฟังก์ชันควบคุมหลัก
```

### เมธอด Getter
```cpp
long getSampleTime();
double getKp();
double getKi();
double getKd();
```

### Enums

#### MembershipFunctionType
- `Triangle` - ฟังก์ชันสมาชิกสามเหลี่ยม
- `Trapezoid` - ฟังก์ชันสมาชิกรูปสี่เหลี่ยมคางหมู
- `Gaussian` - ฟังก์ชันสมาชิกแบบเกาส์เซียน (เส้นโค้งระฆัง)
- `BellShaped` - ฟังก์ชันสมาชิกรูประฆังทั่วไป
- `SShaped` - ฟังก์ชันสมาชิกรูป S
- `ZShaped` - ฟังก์ชันสมาชิกรูป Z
- `Singleton` - ฟังก์ชันสมาชิกแบบซิงเกิลตัน

#### InferenceMode
- `MamdaniMaxMin` - การอนุมานแมมดานิด้วยตัวดำเนินการ min/max
- `MamdaniMaxProduct` - การอนุมานแมมดานิด้วยตัวดำเนินการผลิตภัณฑ์
- `TSK` - การอนุมานแบบ Takagi-Sugeno-Kang
- `SAM` - แบบจำลองการบวกมาตรฐาน

## ตัวอย่าง

ไลบรารีมีตัวอย่างที่ครอบคลุม:

1. **BasicUsage**: การตั้งค่าการควบคุมมอเตอร์แบบง่าย
2. **AdvancedConfiguration**: ตัวควบคุมหลายตัวด้วยการกำหนดค่าที่แตกต่างกัน

## วิธีการทำงาน

ตัวควบคุม Adaptive Fuzzy PID ทำงานใน 3 ขั้นตอนหลัก:

1. **Fuzzification**: แปลงค่าข้อผิดพลาดและอัตราข้อผิดพลาดที่ชัดเจนเป็นเซตคลุมเครือโดยใช้ฟังก์ชันสมาชิก
2. **Inference**: ใช้กฎคลุมเครือเพื่อกำหนดการปรับการควบคุมตามตัวแปรภาษา (Negative Large, Negative Medium ฯลฯ)
3. **Defuzzification**: แปลงเอาต์พุตคลุมเครือกลับเป็นค่าที่ชัดเจนสำหรับการปรับพารามิเตอร์ PID

ตัวควบคุมปรับพารามิเตอร์ของมันอย่างต่อเนื่องตามประสิทธิภาพของระบบ ให้การควบคุมที่ดีกว่าตัวควบคุม PID แบบพารามิเตอร์คงที่แบบดั้งเดิม

## การประยุกต์ใช้

- การควบคุมความเร็ว DC มอเตอร์
- ระบบการวางตำแหน่งเซอร์โว
- การควบคุมอุณหภูมิ
- การประยุกต์ใช้การควบคุมกระบวนการ
- ระบบหุ่นยนต์
- โปรเจกต์ระบบอัตโนมัติ

## สัญญาอนุญาต

ไลบรารีนี้ออกใต้ GNU Lesser General Public License v2.1 ดูไฟล์ LICENSE สำหรับรายละเอียด

## การมีส่วนร่วม

ยินดีรับการมีส่วนร่วม! โปรดอย่าลังเลที่จะส่งปัญหา คำขอคุณสมบัติ หรือ pull requests

## ผู้เขียน

Tom Dhanabhon - [dev@dhanabhon.com](mailto:dev@dhanabhon.com)

## ประวัติเวอร์ชัน

- **v0.0.1** - รีลีสเริ่มต้นพร้อมการใช้งาน fuzzy PID ที่สมบูรณ์