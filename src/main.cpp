/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Auchai                                               */
/*    Created:      12/27/2025                                                */
/*    Description:  V5 Competition - Push Back 2025-2026                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// ============================================================================
// GLOBAL INSTANCES
// ============================================================================

competition Competition;

// --- Brain ---
brain Brain;

// --- Controller ---
controller Controller1 = controller(primary);

// --- Drive Motors (6-motor drive) ---
motor Lf = motor(PORT1, ratio6_1, true);   // Left Front
motor Lm = motor(PORT2, ratio6_1, false);  // Left Middle
motor Lb = motor(PORT3, ratio6_1, true);   // Left Back
motor Rf = motor(PORT8, ratio6_1, false);  // Right Front
motor Rm = motor(PORT9, ratio6_1, true);   // Right Middle
motor Rb = motor(PORT10, ratio6_1, false); // Right Back

// --- Motor Groups ---
motor_group LM = motor_group(Lf, Lm, Lb);
motor_group RM = motor_group(Rf, Rm, Rb);

// --- Mechanism Motors ---
motor intake = motor(PORT15, ratio6_1, true);
motor top = motor(PORT16, ratio18_1, false);
motor mid = motor(PORT17, ratio18_1, false);

// --- Pneumatics ---
digital_out descoreArm = digital_out(Brain.ThreeWirePort.B);
digital_out SlopeArm = digital_out(Brain.ThreeWirePort.C);
digital_out blocker = digital_out(Brain.ThreeWirePort.A);

// --- Sensors ---
distance dist = distance(PORT14);
gps GPS18 = gps(PORT18, -304.80, 88.90, mm, -90);

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

// Reset Motor คือการ reset ตัว encoder ของ motor ให้เป็นค่า 0
void rstm()
{
  // Lf.setPosition(0, degrees);
  // Lm.setPosition(0, degrees);
  // Lb.setPosition(0, degrees);
  // Rf.setPosition(0, degrees);
  // Rm.setPosition(0, degrees);
  // Rb.setPosition(0, degrees);

  // Group Motor ก็มีคำสั่ง setPosition เหมือนกันนะครับ
  LM.setPosition(0, degrees);
  RM.setPosition(0, degrees);
  LM.stop();
  RM.stop();

  // ต้องมี wait เพราะต้องรอให้ motor หยุดหมุนจริงๆ ซะก่อน
  // ซึ่งถ้าไม่ wait เลย มอเตอร์จริงอาจยังไม่หยุดหมุนจริง จะรอมากกว่านี้ก็ได้
  wait(200, msec);
}

// สั่งให้ Motor หมุน -> robot เคลื่อนที่ตาม speed แต่่ไม่มีหยุด
void move(float l, float r)
{
  if (l == 0)
  {
    LM.stop();
  }
  else
  {
    LM.spin(forward, l, percent);
  }

  if (r == 0)
  {
    RM.stop();
  }
  else
  {
    RM.spin(forward, r, percent);
  }
}

void moveDeg(int SpeedL, int SpeedR, float deg)
{
  while ((abs(Lm.position(degrees)) + abs(Rm.position(degrees))) / 2 < deg)
  {
    move(-SpeedL, -SpeedR);
  }
}

void take(int outSpeed, int midSpeed, int inSpeed)
{
  intake.spin(forward, inSpeed, percent);
  top.spin(forward, outSpeed, percent);
  mid.spin(forward, midSpeed, percent);
}

// ฟังก์ชั่นคำนวนการหมุน โดยใช้ GPS Heading -> Target Heading
// turnToTargetHeading(ทิศที่ต้องการ, ความเร็ว);
void turnToTargetHeading(int targetHeading, int speed)
{
  double error;
  while (true)
  {
    error = targetHeading - GPS18.heading();

    // แก้ปัญหาข้าม 0 องศา จะได้หมุนไปในทิศทางที่ใกล้ที่สุดได้ โดยยึดจากครึ่งนึงของวงกลม
    // ก็คือ 180 ถึงจะรู้ว่าควรจะหมุนซ้าย หรือหมุนขวาดี
    if (error > 180)
    {
      error -= 360;
    }
    else if (error < -180)
    {
      error += 360;
    }

    // เช็คว่าถึงเป้าหรือยัง (+-2 องศา = ถึงเป้าตามที่คุยกันไว้)
    if (abs(error) < 2)
    {
      break;
    }

    // คำสั่งให้หมุน
    if (error > 0)
    {
      // LM > RM = หมุนขวา
      move(speed, -speed);
    }
    else
    {
      // LM < RM = หมุนซ้าย
      move(-speed, speed);
    }

    // มี wait เพราะต้องสร้าง delay ให้ GPS อ่านค่าหลังจากสั่งหมุน
    // ถ้าไม่งั้น มันจะเช็คไวมาก เพราะคำสั่งหมุน มันจะหมุน แล้ววนูปต่อเลย ไม่รอให้หมุนเสร็จ
    wait(20, msec);
  }

  move(0, 0); // หยุดเดิน
}

void acc(int topSpeed)
{
  if (topSpeed > 0)
  {
    for (int speed = 20; speed <= topSpeed; speed += 4)
    {
      move(-speed, -speed);
      wait(25, msec);
    }
  }
  else
  {
    for (int speed = -20; speed >= topSpeed; speed -= 4)
    {
      move(-speed, -speed);
      wait(25, msec);
    }
  }
}

void deacc(int from, int to)
{
  if (from > to)
  {
    for (int i = from; i >= to; i -= 4)
    {
      move(-i, -i);
      wait(25, msec);
    }
  }
  else
  {
    for (int i = from; i <= to; i += 4)
    {
      move(-i, -i);
      wait(25, msec);
    }
  }
}

void pnue(bool slopeState, bool descState, bool bump)
{
  SlopeArm.set(slopeState);
  descoreArm.set(descState);
  blocker.set(bump);
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{
  LM.setStopping(brake);
  RM.setStopping(brake);
  pnue(0, 1, 1);
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void)
{
  // --- Phase 1: Initial run ---
  rstm();
  take(100, 100, 100);
  acc(60);
  moveDeg(60, 60, 600);
  deacc(60, 20);
  moveDeg(33, 14, 1650);
  moveDeg(10, 10, 1900);
  rstm();

  // --- Phase 2: Intake block ---
  moveDeg(40, -40, 150);
  moveDeg(15, -15, 320);
  rstm();

  // --- Phase 3: Score mid goal ---
  moveDeg(-40, -40, 450);
  take(-100, 100, 100);
  moveDeg(-20, -20, 550);
  rstm();
  wait(1400, msec);

  // --- Phase 4: Move to next position ---
  acc(60);
  moveDeg(60, 60, 1000);
  moveDeg(20, 20, 1150);
  moveDeg(25, 45, 2000);
  rstm();
  take(0, 0, 0);
  wait(400, msec);

  // --- Phase 5: GPS turn to 269° ---
  moveDeg(40, -40, 120);
  int target = 269;

  while (!(GPS18.heading() < target + 1 && GPS18.heading() > target - 1))
  {
    if (GPS18.heading() > target)
    {
      move(-5, 5);
    }
    else
    {
      move(5, -5);
    }
  }

  int timeInit = Brain.Timer.time(msec);
  while (timeInit + 500 > Brain.Timer.time(msec))
  {
    if (GPS18.heading() > target + 1)
    {
      move(-5, 5);
    }
    else if (GPS18.heading() < target - 1)
    {
      move(5, -5);
    }
    else
    {
      move(0, 0);
    }
  }
  rstm();

  // --- Phase 6: Approach with distance sensor ---
  take(100, 100, 100);
  pnue(1, 1, 1);

  while (dist.objectDistance(mm) > 300)
  {
    move(-40, -40);
  }
  while (dist.objectDistance(mm) > 165)
  {
    move(-20, -20);
  }
  rstm();

  moveDeg(-20, -20, 50);
  rstm();
  moveDeg(20, 20, 50);
  rstm();
  wait(200, msec);

  // --- Phase 7: Back away ---
  acc(-60);
  while (dist.objectDistance(mm) < 400)
  {
    move(60, 60);
  }
  deacc(-60, -20);
  while (dist.objectDistance(mm) < 800)
  {
    move(20, 20);
  }

  pnue(0, 1, 0);
  move(20, 20);
  wait(1200, msec);
  rstm();

  // --- Phase 8: Final push ---
  moveDeg(40, 40, 200);
  moveDeg(20, 20, 300);
  pnue(0, 1, 1);
  rstm();
  move(40, 40);
  wait(1000, msec);
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void usercontrol(void)
{
  // Variables for toggle states
  int intakeDir = 0;
  int slopeArm = 0;
  int blockerArm = 0;
  int scorerDir = 0;

  while (1)
  {
    // === ARCADE DRIVE ===
    int x = (Controller1.Axis1.position() * 0.6);
    int y = -Controller1.Axis3.position();

    if (x <= 10 && x >= -10 && y <= 10 && y >= -10)
    {
      LM.stop();
      RM.stop();
    }
    else
    {
      LM.setVelocity((y + x), percent);
      RM.setVelocity((y - x), percent);
      LM.spin(forward);
      RM.spin(forward);
    }

    // === INTAKE CONTROL (R1/R2) ===
    if (Controller1.ButtonR1.pressing())
    {
      if (intakeDir == 0)
      {
        intakeDir = 1;
        top.spin(forward, 100, percent);
        mid.spin(forward, 100, percent);
        intake.spin(forward, -100, percent);
      }
      else
      {
        intakeDir = 0;
        top.stop();
        mid.stop();
        intake.stop();
      }
      wait(300, msec);
    }

    if (Controller1.ButtonR2.pressing())
    {
      if (intakeDir == 0)
      {
        intakeDir = -1;
        top.spin(forward, -100, percent);
        mid.spin(forward, -100, percent);
        intake.spin(forward, 100, percent);
      }
      else
      {
        intakeDir = 0;
        top.stop();
        mid.stop();
        intake.stop();
      }
      wait(300, msec);
    }

    // === DESCORE ARM (L1/L2) ===
    if (Controller1.ButtonL1.pressing())
    {
      descoreArm.set(1);
      wait(300, msec);
    }
    if (Controller1.ButtonL2.pressing())
    {
      descoreArm.set(0);
      wait(300, msec);
    }

    // === SLOPE ARM (Y) ===
    if (Controller1.ButtonY.pressing())
    {
      slopeArm = (slopeArm == 0) ? 1 : 0;
      SlopeArm.set(slopeArm);
      wait(300, msec);
    }

    // === BLOCKER (Right) ===
    if (Controller1.ButtonRight.pressing())
    {
      blockerArm = (blockerArm == 0) ? 1 : 0;
      blocker.set(blockerArm);
      wait(300, msec);
    }

    // === SCORER (Left = long goal, Up = mid goal) ===
    if (Controller1.ButtonLeft.pressing())
    {
      if (scorerDir == 0)
      {
        scorerDir = 1;
        top.spin(forward, 100, percent);
        mid.spin(forward, 100, percent);
        intake.spin(forward, -100, percent);
      }
      else
      {
        scorerDir = 0;
        top.stop();
        mid.stop();
        intake.stop();
      }
      wait(300, msec);
    }

    if (Controller1.ButtonUp.pressing())
    {
      if (scorerDir == 0)
      {
        scorerDir = 1;
        top.spin(forward, -100, percent);
        mid.spin(forward, 100, percent);
        intake.spin(forward, -100, percent);
      }
      else
      {
        scorerDir = 0;
        top.stop();
        mid.stop();
        intake.stop();
      }
      wait(300, msec);
    }

    wait(20, msec);
  }
}

/*---------------------------------------------------------------------------*/
/*                                    Main                                   */
/*---------------------------------------------------------------------------*/

int main()
{
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true)
  {
    wait(100, msec);
  }
}