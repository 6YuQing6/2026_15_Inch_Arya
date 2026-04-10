#pragma once
#include "vex.h"

enum class IntakeMode {
  STOPPED,
  INTAKE,
  INTAKE_SLOW,
  OUTTAKE_BOTTOM,
  OUTTAKE_TOP_MIDDLE,
  OUTTAKE_TOP
};

extern vex::event IntakeFull;

extern IntakeMode currentIntakeMode;

void intakeTask();

void intakeBalls();
void intakeBallsSlow();
void outakeBallsBottom();
void outakeBallsTop();
void outakeBallsMiddle();
void stopIntake();
