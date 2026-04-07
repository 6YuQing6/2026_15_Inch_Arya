#include "intake.h"
#include "robot-config.h"

extern bool thirdStageOverrideActive;

IntakeMode currentIntakeMode = IntakeMode::STOPPED;
vex::event IntakeFull;

void intakeTask() {
  bool topStopped = false;
  bool middleStopped = false;
  while (true) {
    switch (currentIntakeMode) {
      case IntakeMode::INTAKE:
        Stopper.set(true);
        if (middleStopped && topStopped && (OpticalBottom1.isNearObject() || OpticalBottom.isNearObject())) {
          ZeroStage.stop(coast);
          IntakeFull.broadcast();
        } else if (OpticalBottom1.isNearObject() || OpticalBottom.isNearObject()) {
          ZeroStage.spin(vex::forward, 50, vex::percent);
        } else {
          ZeroStage.spin(vex::forward, 100, vex::percent);
        }
        if (MiddleSensor.objectDistance(inches) < 2 && topStopped) {
          FirstStage.stop(coast);
          middleStopped = true;
        } else {
          FirstStage.spin(vex::forward, 100, vex::percent);
          middleStopped = false;
        }
        if (OpticalTop.isNearObject()) {
          SecondStage.stop(vex::coast);
          ThirdStage.stop(vex::coast);
          topStopped = true;
        } else {
          SecondStage.spin(vex::forward, 100, vex::percent);
          ThirdStage.spin(vex::forward, 100, vex::percent);
          topStopped = false;
        }
        break;

      case IntakeMode::INTAKE_SLOW:
        Stopper.set(true);
        ZeroStage.spin(vex::forward, 60, vex::percent);
        FirstStage.spin(vex::forward, 60, vex::percent);
        if (OpticalTop.isNearObject()) {
          SecondStage.stop(vex::coast);
          ThirdStage.stop(vex::coast);
        } else {
          SecondStage.spin(vex::forward, 60, vex::percent);
          ThirdStage.spin(vex::forward, 60, vex::percent);
        }
        break;

      case IntakeMode::OUTTAKE_BOTTOM:
        ColorSort.set(false);
        ZeroStage.spin(vex::reverse, 100, vex::percent);
        FirstStage.spin(vex::reverse, 100, vex::percent);
        SecondStage.spin(vex::reverse, 100, vex::percent);
        ThirdStage.spin(vex::reverse, 100, vex::percent);
        break;

      case IntakeMode::OUTTAKE_TOP_MIDDLE:
      case IntakeMode::OUTTAKE_TOP:
        Stopper.set(false);
        if (OpticalBottom1.isNearObject() || OpticalBottom.isNearObject()) {
          ZeroStage.spin(vex::forward, 10, vex::percent);
        } else {
          ZeroStage.spin(vex::forward, 100, vex::percent);
        }
        FirstStage.spin(vex::forward, 100, vex::percent);
        SecondStage.spin(vex::forward, 100, vex::percent);
        if (!thirdStageOverrideActive) {
          vex::directionType dir = (currentIntakeMode == IntakeMode::OUTTAKE_TOP_MIDDLE)
                                       ? vex::reverse : vex::forward;
          ThirdStage.spin(dir, 12000, vex::voltageUnits::mV);
        }
        break;

      case IntakeMode::STOPPED:
        ZeroStage.stop(vex::coast);
        FirstStage.stop(vex::brake);
        SecondStage.stop(vex::brake);
        ThirdStage.stop(vex::brake);
        break;
    }

    vex::task::sleep(100);
  }
}

void intakeBalls()       { currentIntakeMode = IntakeMode::INTAKE; }
void intakeBallsSlow()   { currentIntakeMode = IntakeMode::INTAKE_SLOW; }
void outakeBallsBottom() { currentIntakeMode = IntakeMode::OUTTAKE_BOTTOM; }
void outakeBallsTop()    { currentIntakeMode = IntakeMode::OUTTAKE_TOP; }
void outakeBallsMiddle() { currentIntakeMode = IntakeMode::OUTTAKE_TOP_MIDDLE; }
void stopIntake()        { currentIntakeMode = IntakeMode::STOPPED; }
