#include "color.h"
#include "robot-config.h"
#include "ai_functions.h"

using namespace vex;

timer ColorSortTimer;

directionType thirdStageDefaultDir = forward;
bool thirdStageOverrideActive = false;

int activeTeamColor = BallBlue;

static int detectBallFromHue(int hue) {
  // if ((hue >= 300 && hue <= 359) || (hue >= 0 && hue <= 50)) {
  if ((hue >= 0 && hue <= 11)) {
    return BallRed;
  }
  if (hue >= 140 && hue <= 290) {
    return BallBlue;
  }
  return BallUndefined;
}

enum ColorState {
  COLOR_IDLE = 0,
  COLOR_EJECTING,
  COLOR_RESET
};

static int colorState = COLOR_IDLE;
static int colorTimer = 0;

int onBottomDetectedThread() {
  OpticalBottom.setLightPower(100, percent);
  OpticalBottom1.setLightPower(100, percent);
  OpticalBottom.setLight(ledState::on);
  OpticalBottom1.setLight(ledState::on);
  ColorSortTimer.reset();

  while (true) {
    if (OpticalBottom.isNearObject() || OpticalBottom1.isNearObject()) {
      int hue0 = OpticalBottom.hue();
      int hue1 = OpticalBottom1.hue();
      int avgHue = (hue0 + hue1) / 2;
      int detected = BallUndefined;

    printf("Prox0: %d Prox1: %d\n", 
    vexOpticalProximityGet(OpticalBottom.index()),
    vexOpticalProximityGet(OpticalBottom1.index()));

      int d0 = OpticalBottom.isNearObject() ? detectBallFromHue(hue0) : BallUndefined;
      int d1 = OpticalBottom1.isNearObject() ? detectBallFromHue(hue1) : BallUndefined;

      printf("OpticalBottom0Hue: %d \n", hue0);
      printf("OpticalBottom1Hue: %d \n", hue1);

      if (d0 == d1 && (d0 != BallUndefined && d1 != BallUndefined)) {
        detected = d0;
      } else if (d0 == BallBlue || d1 == BallBlue) {
        detected = BallBlue;
      } else if (d0 != BallUndefined && d1 == BallUndefined) {
        detected = d0;
      } else if (d1 != BallUndefined && d0 == BallUndefined) {
        detected = d1;
      } else {
        detected = detectBallFromHue(avgHue);
      }

      printf("Detected Ball Color: %d \n", detected);
      // printf("Active team color: %d \n", activeTeamColor);

      if (detected != activeTeamColor) {
        colorState = COLOR_EJECTING;
        printf("Color Eject \n");
      } else {
        colorState = COLOR_IDLE;
        printf("Color No Eject \n");
      }
    }

    switch (colorState) {
      case COLOR_IDLE:
        ColorSort.set(false);
        break;

      case COLOR_EJECTING:
        ColorSort.set(true);
        colorTimer = ColorSortTimer.time();
        colorState = COLOR_RESET;
        break;

      case COLOR_RESET: {
        int currentTime = ColorSortTimer.time();
        if (currentTime - colorTimer > 100) {
          ColorSort.set(false);
          colorTimer = 0;
          colorState = COLOR_IDLE;
        }
        break;
      }
    }

    this_thread::sleep_for(15);
  }
  return 0;
}

static int topState = 0;
static int topColorTimer = 0;

int onTopDetectedThread() {
  OpticalTop.setLightPower(100, percent);
  OpticalTop.setLight(ledState::on);

  while (true) {
    switch (topState) {
      case COLOR_IDLE: {
        thirdStageOverrideActive = false;
        if (OpticalTop.isNearObject()) {
          int hue = OpticalTop.hue();
          int detected = detectBallFromHue(hue);
          if (detected != activeTeamColor) {
            topState = COLOR_EJECTING;
          }
        }
        break;
      }

      case COLOR_EJECTING: {
        wait(500, msec);
        thirdStageOverrideActive = true;
        if (thirdStageDefaultDir == forward) {
          ThirdStage.spin(reverse, 12000, voltageUnits::mV);
        } else {
          ThirdStage.spin(forward, 12000, voltageUnits::mV);
        }
        topColorTimer = ColorSortTimer.time();
        topState = COLOR_RESET;
        break;
      }

      case COLOR_RESET: {
        int currentTime = ColorSortTimer.time();
        if (currentTime > topColorTimer + 1500) {
          ThirdStage.spin(thirdStageDefaultDir, 12000, voltageUnits::mV);
          topState = COLOR_IDLE;
        }
        break;
      }
    }
    this_thread::sleep_for(20);
  }
  return 0;
}
