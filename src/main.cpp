/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "ai_functions.h"
#include "astar.h"
#include "field_map.h"
#include "Localizer.h"
#include "DualGPS.h"
#include "brain.h"
#include <cmath>
#include <cstdlib>
#include <vector>

using namespace vex;

brain Brain;
controller Controller;
/*// Robot configuration code (flipped front/back - GPS positions and angles updated)
// GPS sensors: 3 inches forward from center = 7.62 cm
// x_offset: left/right from center (LGPS on right +12.5, RGPS on left -12.5)
// y_offset: forward from center
// heading_offset: swapped 90<->270 after front/back flip
gps LGPS = gps(PORT19, -12.5, 13.5, distanceUnits::cm, 270);
gps RGPS = gps(PORT14, 12.5, 5.5, distanceUnits::cm, 90);  // y adjusted from 13.5 to 5.5 (8cm correction)*/

// Robot configuration code - 15 inch robot
gps LGPS = gps(PORT9, -9, -6, distanceUnits::cm, 270);
gps RGPS = gps(PORT10, 11.5, -7.5, distanceUnits::cm, 90);
// DualGPS GPS = DualGPS(LGPS, RGPS, Inertial, vex::distanceUnits::cm);
// Left Drive
motor LeftDriveA = motor(PORT16, ratio6_1, false);
motor LeftDriveB = motor(PORT15, ratio6_1, false);
motor LeftDriveC = motor(PORT14, ratio6_1, false);
motor_group LeftDrive = motor_group(LeftDriveA, LeftDriveB, LeftDriveC);
// Right Drive
motor RightDriveA = motor(PORT20, ratio6_1, true);
motor RightDriveB = motor(PORT19, ratio6_1, true);
motor RightDriveC = motor(PORT18, ratio6_1, true);
motor_group RightDrive = motor_group(RightDriveA, RightDriveB, RightDriveC);

// Intake motors
motor FirstStage = motor(PORT8, ratio6_1, false);
motor SecondStage = motor(PORT11, ratio6_1, false);
motor ThirdStage = motor(PORT12, ratio6_1, false);
motor ZeroStage = motor(PORT1, ratio6_1, false);

// Inertial sensor
inertial Inertial = inertial(PORT17);

// Smartdrive
smartdrive Drivetrain = smartdrive(LeftDrive, RightDrive, Inertial, 13.5, 13.5, 0.0, distanceUnits::in, 0.75);



// Sensors
optical OpticalTop = optical(PORT4);
optical OpticalBottom = optical(PORT2);
optical OpticalBottom1 = optical(PORT6);
distance MiddleSensor = distance(PORT3);

// A global instance of competition
competition Competition;

// create instance of jetson class to receive location and other
// data from the Jetson nano
//
ai::jetson  jetson_comms;

Drive chassis(

  //Pick your drive setup from the list below:
  //ZERO_TRACKER_NO_ODOM
  //ZERO_TRACKER_ODOM
  //TANK_ONE_FORWARD_ENCODER
  //TANK_ONE_FORWARD_ROTATION
  //TANK_ONE_SIDEWAYS_ENCODER
  //TANK_ONE_SIDEWAYS_ROTATION
  //TANK_TWO_ENCODER
  //TANK_TWO_ROTATION
  //HOLONOMIC_TWO_ENCODER
  //HOLONOMIC_TWO_ROTATION
  //
  //Write it here:
  TANK_TWO_ROTATION,  // Using both forward and sideways rotation sensor trackers
  
  //Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
  //You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".
  
  //Left Motors:
  motor_group(LeftDriveA, LeftDriveB, LeftDriveC),
  
  //Right Motors:
  motor_group(RightDriveA, RightDriveB, RightDriveC),
  
  //Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
  PORT17,
  
  //Input your wheel diameter. (4" omnis are actually closer to 4.125"):
  3.25,
  
  //External ratio, must be in decimal, in the format of input teeth/output teeth.
  //If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
  //If the motor drives the wheel directly, this value is 1:
  0.75,
  
  //Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
  //For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
  362,    // Inertial scale, value that reads after turning robot a full 360
  
  /*---------------------------------------------------------------------------*/
  /*                                  PAUSE!                                   */
  /*                                                                           */
  /*  The rest of the drive constructor is for robots using POSITION TRACKING. */
  /*  If you are not using position tracking, leave the rest of the values as  */
  /*  they are.                                                                */
  /*---------------------------------------------------------------------------*/
  
  //If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.
  
  //FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
  //LF:      //RF:    
  PORT1,     -PORT2,
  
  //LB:      //RB: 
  PORT3,     -PORT4,
  
  //Forward Tracker: PORT7, rotation sensor
  //Negative diameter to reverse direction (forward = negative rotations on sensor)
  PORT7,
  
  //Forward Tracker diameter (NEGATIVE - sensor counts negative when driving forward)
  -2.0,
  
  //Forward Tracker center distance (positive = right of center, negative = left of center)
  //0.1 inches to the LEFT of center = -0.1 - changed to 0.0 to center the robot
  0.0,
  
  //Sideways Tracker: PORT5, rotation sensor
  //Negative diameter to reverse direction (right shift = decreasing rotations)
  PORT5,
  
  //Sideways tracker diameter (NEGATIVE - sensor counts negative when shifting right)
  -2.0,
  
  //Sideways tracker center distance (positive = behind center, negative = in front)
  //4 inches BEHIND of center = 4.0
  4.0
  
);
DualGPS GPS = DualGPS(LGPS, RGPS, chassis.Gyro, vex::distanceUnits::cm);

// Global Localizer instance - EKF fusion of odometry, IMU, and dual GPS
Localizer localizer(GPS, chassis, chassis.Gyro);

// Global match brain (state machine)
MatchBrain matchBrain;

// Configure chassis PID/voltage limits with optimized tuning
void configureChassis(){
  //SUNNY VALUES
  // chassis.set_turn_constants(10, 0.35, 0, 2.4, 15);
  // chassis.set_turn_constants(10, 0.48, 0.02, 5.8, 15);
  chassis.set_turn_constants(10, 0.26, 0.01, 2.5, 0);
  // chassis.set_turn_constants(10, 0.34, 0.0, 2.45, 0);

  chassis.set_heading_constants(6, 0.4, 0, 1, 0);
  // chassis.set_drive_constants(12, 1.5, 0, 10, 0);
  chassis.set_swing_constants(12, .3, .001, 2, 15);
  // Tighter exit conditions for better accuracy
  // drive: 1.0 inch settle error (~2.5cm)
  chassis.set_drive_exit_conditions(1.0, 300, 1500);
  // turn: 2.0 degree settle error (tightened from 3.0)
  chassis.set_turn_exit_conditions(1.0, 500, 1500);
  chassis.set_swing_exit_conditions(2.0, 200, 2000); 
  // Boomerang controller constants for drive_to_pose
  // lead: 0.3 = carrot point leads by 30% of distance (lower = more direct path)
  // setback: 1.5 inches = tighter approach
  chassis.set_boomerang_constants(0.3, 1.5);

  //ARYA OG VALUES
  // chassis.set_drive_constants(12, 0.550, 0.040, 0.570, 0);
  chassis.set_drive_constants(12, 0.55, 0.04, 3.9, 0);

  // chassis.set_heading_constants(4, 0.15, 0.0, 0.02, 0);
  // chassis.set_turn_constants(8, 0.090, 0.120, 0.260, 0);
  // chassis.set_swing_constants(8, 0.090, 0.120, 0.260, 0);
  // chassis.set_drive_exit_conditions(1.0, 200, 4000);
  // chassis.set_turn_exit_conditions(2.0, 200, 2000);
  // chassis.set_swing_exit_conditions(2.0, 200, 2000);
  // chassis.set_boomerang_constants(0.3, 1.5);
}

/*----------------------------------------------------------------------------*/
// Create a robot_link on PORT1 using the unique name robot_32456_1
// The unique name should probably incorporate the team number
// and be at least 12 characters so as to generate a good hash
//
// The Demo is symetrical, we send the same data and display the same status on both
// manager and worker robots
// Comment out the following definition to build for the worker robot
#define  MANAGER_ROBOT    1

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT13, "robot_32456_1", linkType::manager );
#else
#pragma message("building for the worker")
ai::robot_link       link( PORT13, "robot_32456_1", linkType::worker );
#endif

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Auto_Isolation Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous isolation  */
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void isolation_Left() {
  // start with bot backwards, top left park zone black aligns with bottom right of bot
  chassis.drive_distance(-36);
  if (activeTeamColor == BallRed) {
    chassis.turn_to_point(-24, 24);
  } else if (activeTeamColor == BallBlue) {
    chassis.turn_to_point(24, -24);
  }
  intakeBalls();
  chassis.drive_max_voltage = 6;
  chassis.drive_distance(12);
  chassis.drive_max_voltage = 10;
  MatchLoader.set(true);
  chassis.drive_distance(-14);
  outakeBallsMiddle();
  MatchLoader.set(false);
  wait(1000, msec);

  chassis.turn_settle_time = 300;
  intakeBalls();
  chassis.drive_settle_time = 800;
  chassis.drive_max_voltage = 8;
  MatchLoader.set(true);
  if (activeTeamColor == BallRed) {
    chassis.drive_to_point(-48, 48);
  } else if (activeTeamColor == BallBlue) {
    chassis.drive_to_point(48, -48);
  }
  chassis.drive_settle_time = 500;
  if (activeTeamColor == BallRed) {
    chassis.turn_to_angle(270);
  } else if (activeTeamColor == BallBlue) {
    chassis.turn_to_angle(90);
  }
  chassis.drive_max_voltage = 5;
  chassis.drive_distance(17);
  chassis.drive_max_voltage = 10;
  wait(2, sec);

  chassis.drive_timeout = 200;
  chassis.drive_distance(-10);
  if (activeTeamColor == BallRed) {
    chassis.turn_to_angle(270);
  } else if (activeTeamColor == BallBlue) {
    chassis.turn_to_angle(90);
  }
  // chassis.turn_to_point(-100, 47);
  chassis.drive_max_voltage = 12;
  chassis.drive_distance(-18);
  outakeBallsTop();
}

void isolation_Right() {
  chassis.drive_distance(-36);
  if (activeTeamColor == BallRed) {
    chassis.turn_to_point(-24, -24);
  } else if (activeTeamColor == BallBlue) {
    chassis.turn_to_point(24, 24);
  }
  intakeBalls();
  chassis.drive_max_voltage = 6;
  chassis.drive_distance(12);
  chassis.drive_max_voltage = 10;
  MatchLoader.set(true);
  chassis.turn_to_angle(0,0);
  MatchLoader.set(false);
  chassis.drive_distance(14);
  outakeBallsBottom();
  wait(1000, msec);
  chassis.drive_distance(-10);
  chassis.turn_settle_time = 300;
  intakeBalls();
  chassis.drive_settle_time = 800;
  chassis.drive_max_voltage = 8;
  MatchLoader.set(true);
  if (activeTeamColor == BallRed) {
    chassis.turn_to_point(-48, -48);
    chassis.drive_to_point(-48, -48);
  } else if (activeTeamColor == BallBlue) {
    chassis.turn_to_point(48, 48);
    chassis.drive_to_point(48, 48);
  }
  chassis.drive_settle_time = 500;
  if (activeTeamColor == BallRed) {
    chassis.turn_to_angle(270);
  } else if (activeTeamColor == BallBlue) {
    chassis.turn_to_angle(90);
  }

  chassis.drive_max_voltage = 5;
  chassis.drive_distance(17);
  chassis.drive_max_voltage = 10;
  wait(2, sec);

  chassis.drive_timeout = 200;
  chassis.drive_distance(-10);
  if (activeTeamColor == BallRed) {
    chassis.turn_to_angle(270);
  } else if (activeTeamColor == BallBlue) {
    chassis.turn_to_angle(90);
  }
  // chassis.turn_to_point(-100, 47);
  chassis.drive_max_voltage = 12;
  chassis.drive_distance(-18);
  outakeBallsTop();
}

// starts facing loader
void isolation_Right_New_RED() {
    chassis.set_coordinates(-49.5, -16.8, chassis.get_absolute_heading());
    MatchLoader.set(true);
    // chassis.turn_to_angle(180);

    // drives to loader
    chassis.drive_to_point(-49.5, -48);
    // chassis.drive_distance(32);

    // intakes from loader
    // chassis.turn_to_point(-100, -48);
    chassis.turn_settle_error = 2.0;
    chassis.turn_settle_time = 300;
    chassis.turn_timeout = 1000;
    chassis.turn_to_angle(-90);
    chassis.turn_settle_time = 500;
    chassis.turn_timeout = 1500;
    chassis.turn_settle_error = 1.0;
    intakeBalls();
    chassis.drive_timeout = 500;
    chassis.drive_settle_time = 0;
    chassis.drive_distance(11);
    wait(3.2, sec);
    
    // scores in long goal
    // chassis.drive_timeout = 500;
    // chassis.drive_distance(-15);
    // chassis.turn_to_point(-100, -47);
    // chassis.turn_to_angle(-90);
    chassis.drive_max_voltage = 10;
    // chassis.drive_to_point(-28, -47);
    chassis.drive_timeout = 900;
    chassis.drive_settle_time = 0;
    chassis.drive_distance(-39.0);
    chassis.drive_max_voltage = 12;
    chassis.drive_settle_time = 300;
    // chassis.drive_distance(-24.0);
    chassis.drive_timeout = 1000;
    outakeBallsTop();
    wait(1.9, sec);

    // turns to middle goal
    // chassis.turn_to_point(-100, -48);
    intakeBalls();
    // chassis.turn_to_angle(-90);
    MatchLoader.set(false);

    // drives to middle goal, pushes tower of balls away
    chassis.drive_to_point(-49.5,-48);
    // chassis.drive_distance(22);
    // chassis.turn_to_point(-24,-24);
    chassis.turn_to_angle(45);
    // stopIntake();
    MatchLoader.set(true);
    // chassis.drive_to_point(-24, -24);

    // drives into bottom goal and scores
    chassis.drive_settle_time = 0;
    // chassis.drive_timeout = 00;
    // chassis.drive_distance(36);
    chassis.drive_distance(40);
    outakeBallsBottom();
    // chassis.turn_to_point(0,0);
    // chassis.turn_to_angle(45);
    chassis.drive_max_voltage = 6;
    MatchLoader.set(false);
    chassis.drive_distance(15);
}

// starts facing loader
void isolation_Right_New_BLUE() {
    chassis.set_coordinates(49.5, 16.8, chassis.get_absolute_heading());
    MatchLoader.set(true);
    // chassis.turn_to_angle(180);

    // drives to loader
    chassis.drive_to_point(49.5, 47);
    // chassis.drive_distance(32);

    // intakes from loader
    // chassis.turn_to_point(-100, -48);
    chassis.turn_settle_error = 2.0;
    chassis.turn_settle_time = 300;
    chassis.turn_timeout = 1000;
    chassis.turn_to_angle(90);
    chassis.turn_settle_time = 500;
    chassis.turn_timeout = 1500;
    chassis.turn_settle_error = 1.0;
    intakeBalls();
    chassis.drive_timeout = 500;
    chassis.drive_settle_time = 0;
    chassis.drive_distance(11);
    wait(3.2, sec);
    
    // scores in long goal
    chassis.drive_max_voltage = 10;
    chassis.drive_timeout = 900;
    chassis.drive_settle_time = 0;
    chassis.drive_distance(-39.0);
    chassis.drive_max_voltage = 12;
    chassis.drive_settle_time = 300;
    chassis.drive_timeout = 1000;
    outakeBallsTop();
    wait(1.9, sec);

    // turns to middle goal
    intakeBalls();
    MatchLoader.set(false);

    // drives to middle goal, pushes tower of balls away
    chassis.drive_to_point(49.5,47);
    chassis.turn_to_angle(225);
    MatchLoader.set(true);

    // drives into bottom goal and scores
    chassis.drive_settle_time = 0;
    chassis.drive_distance(40);
    outakeBallsBottom();
    chassis.drive_max_voltage = 6;
    MatchLoader.set(false);
    chassis.drive_distance(16);
}

// starts facing loader
void isolation_Right_New() {
    // Red: negative quadrant (-49.5, -16.8), Blue: positive quadrant (49.5, 16.8)
    double xSign = (activeTeamColor == BallRed) ? -1.0 : 1.0;

    chassis.set_coordinates(xSign * 49.5, xSign * 16.8, chassis.get_absolute_heading());
    MatchLoader.set(true);

    // drives to loader
    chassis.drive_to_point(xSign * 49.5, xSign * 48);

    // intakes from loader
    chassis.turn_settle_error = 2.0;
    chassis.turn_settle_time = 300;
    chassis.turn_timeout = 1000;
    chassis.turn_to_angle((activeTeamColor == BallRed) ? -90 : 90);
    chassis.turn_settle_time = 500;
    chassis.turn_timeout = 1500;
    chassis.turn_settle_error = 1.0;
    intakeBalls();
    chassis.drive_timeout = 500;
    chassis.drive_settle_time = 0;
    chassis.drive_distance(11);
    wait(3.2, sec);

    // scores in long goal
    chassis.drive_max_voltage = 10;
    chassis.drive_timeout = 900;
    chassis.drive_settle_time = 0;
    chassis.drive_distance(-39.0);
    chassis.drive_max_voltage = 12;
    chassis.drive_settle_time = 300;
    chassis.drive_timeout = 1000;
    outakeBallsTop();
    wait(1.9, sec);

    // turns to middle goal
    intakeBalls();
    MatchLoader.set(false);

    // drives to middle goal
    chassis.drive_to_point(xSign * 49.5, xSign * 48);
    chassis.turn_to_angle((activeTeamColor == BallRed) ? 45 : 225);
    MatchLoader.set(true);

    // drives into bottom goal and scores
    chassis.drive_settle_time = 0;
    chassis.drive_distance(40);
    outakeBallsBottom();
    chassis.drive_max_voltage = 6;
    MatchLoader.set(false);
    chassis.drive_distance(15);
}

void auto_Isolation(void) {
  // activeTeamColor = BallRed;
  isolation_Right_New_BLUE();
  // chassis.set_heading(0);
  // chassis.turn_to_angle(90);
  // chassis.turn_to_angle(180);
  // chassis.turn_to_angle(0);
  // chassis.set_coordinates(0,0,0);
  // chassis.drive_to_point(0, 12);
  // chassis.drive_to_point(0, 24);
  // chassis.drive_to_point(0, 48);
  // chassis.drive_to_point(0,0);
  // chassis.turn_to_point(0, 90);
  // chassis.turn_to_point(90, 90);
  // chassis.turn_to_point(90, 0);
  // chassis.turn_to_point(0, -90);
  // chassis.turn_to_point(0, 90);




  // chassis.turn_to_point(-48,-48);
  // chassis.turn_to_point(-24,-48);
  // chassis.turn_to_point(0,0);


  // chassis.turn_to_point(0, 0);
  // int autonCase = 1;
  // switch (autonCase) {
  //   case 0:
  //     isolation_Left();
  //     break;
  //   case 1:
  //     isolation_Right();
  //     break;
  // }
  // Calibrate GPS Sensor
  // GPS.calibrate();
  // // Optional wait to allow for calibration
  // waitUntil(!(GPS.isCalibrating()));

  // goToObject(OBJECT::BallBlue);
  // runIntakeForRotations(directionType::fwd, 3, true);
  // goToGoal();
  // Drivetrain.driveFor(directionType::rev, 115, distanceUnits::cm);
  // SecondStage.setVelocity(70, pct);
  // runIntakeForRotations(directionType::fwd, 5, false);
  // // Back off from the goal
  // Drivetrain.driveFor(directionType::fwd, 30, distanceUnits::cm);

}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                        Auto_Interaction Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous interaction*/
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void auto_Interaction(void) {
  // Add functions for interaction phase
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          AutonomousMain Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/

bool firstAutoFlag = true;

// Simple bearing helper (math coords -> navigation heading)
static double bearingDegrees(double fromX, double fromY, double toX, double toY) {
  double dx = toX - fromX;
  double dy = toY - fromY;
  double rad = atan2(dy, dx);
  double deg = rad * 180.0 / M_PI;
  // convert to navigation: 0° = north, clockwise positive
  double nav = fmod(90.0 - deg, 360.0);
  if (nav < 0) nav += 360.0;
  return nav;
}

// Path plan and follow using A* (cm coordinates), modeled after testPathPlanning
bool planAndFollowPathCm(double targetX_cm, double targetY_cm, float finalHeading, bool /*useSmooth*/) {
  double curr_x = GPS.xPosition();
  double curr_y = GPS.yPosition();
  double curr_h = GPS.heading();

  if ((curr_x == 0.0 && curr_y == 0.0) || std::isnan(curr_x) || std::isnan(curr_y) || std::isnan(curr_h)) {
    return false;
  }

  FieldMap fieldMap;
  fieldMap.populateStandardField();

  const double robot_width_in = 13.5;
  const double robot_radius_cm = robot_width_in * 2.54 * 0.5; // ~17.1 cm
  const double safety_margin_cm = 0.0;                        // tune as needed
  const double grid_resolution_cm = 60.96 / 2.0;              // 12 in / 2

  std::vector<astar::Point> path = astar::findPath(
      fieldMap, curr_x, curr_y, targetX_cm, targetY_cm,
      grid_resolution_cm, robot_radius_cm, safety_margin_cm);

  if (path.empty()) {
    return false;
  }

  for (size_t i = 0; i < path.size(); i++) {
    double wp_x = path[i].first;
    double wp_y = path[i].second;

    curr_x = GPS.xPosition();
    curr_y = GPS.yPosition();
    curr_h = GPS.heading();

    if ((curr_x == 0.0 && curr_y == 0.0) || std::isnan(curr_x) || std::isnan(curr_y)) {
      return false;
    }

    double bearing = bearingDegrees(curr_x, curr_y, wp_x, wp_y);
    double dx = wp_x - curr_x;
    double dy = wp_y - curr_y;
    double dist_cm = sqrt(dx * dx + dy * dy);

    if (dist_cm < 5.0) {
      continue; // already close enough
    }

    double dist_in = dist_cm / 2.54;
    chassis.set_heading(curr_h);
    chassis.turn_to_angle(bearing);
    chassis.drive_distance(dist_in);
  }

  // Final heading if requested
  if (finalHeading >= 0) {
    chassis.turn_to_angle(finalHeading);
  }

  return true;
}

// ---------------------------------------------------------------------------
// Phase helpers for 105s autonomous: Phase 1 random roaming, Phase 2 to (-120,0)
// ---------------------------------------------------------------------------

static bool isSafeRandomTarget(double x, double y) {
  // Keep targets within field bounds and away from the center obstacle
  if (fabs(x) > 170 || fabs(y) > 170) return false;
  if (fabs(x) + fabs(y) < 80) return false;
  return true;
}

static bool pickRandomTarget(double &x, double &y) {
  static bool seeded = false;
  if (!seeded) {
    srand(static_cast<unsigned>(Brain.Timer.system()));
    seeded = true;
  }
  for (int i = 0; i < 20; i++) {
    double rx = -170 + (rand() % 341);  // [-170,170]
    double ry = -170 + (rand() % 341);
    if (isSafeRandomTarget(rx, ry)) {
      x = rx;
      y = ry;
      return true;
    }
  }
  return false;
}

static void runPhase1(int phaseMillis) {
  int phaseStart = Brain.Timer.system();
  while (Brain.Timer.system() - phaseStart < phaseMillis) {
    double tx = 0, ty = 0;
    if (!pickRandomTarget(tx, ty)) break;

    int moveStart = Brain.Timer.system();
    bool ok = planAndFollowPathCm(tx, ty, -1, true);

    // Bail if the move fails or runs long to keep phase responsive
    if (!ok || Brain.Timer.system() - moveStart > 8000) {
      chassis.drive_with_voltage(0, 0);
    }
  }
}

static void runPhase2() {
  planAndFollowPathCm(-120.0, 0.0, -1, true);
}

/*
void autonomousMain(void) {
  // ..........................................................................
  // The first time we enter this function we will launch our Isolation routine
  // When the field goes disabled after the isolation period this task will die
  // When the field goes enabled for the second time this task will start again
  // and we will enter the interaction period. 
  // ..........................................................................

  if(firstAutoFlag)
    auto_Isolation();
  else 
    auto_Interaction();

  firstAutoFlag = false;
}
*/

// New autonomous: brain state machine handles isolation -> collecting -> endgame.
// Legacy random-path approach kept below in case brain needs to be bypassed.
void autonomousMain(void) {
  matchBrain.run();
}

/*
// Legacy autonomous: 105s total. Phase 1 (85s) random paths, Phase 2 (last ~20s) drive to (-120,0).
static void autonomousMainLegacy(void) {
  const int totalMs = 105000;
  const int phase1Ms = 85000;
  int start = Brain.Timer.system();

  runPhase1(phase1Ms);
  Controller.rumble("---");

  if (Brain.Timer.system() - start < totalMs - 5000) {
    runPhase2();
  }
}
*/

// Task wrapper to run autonomous from driver control
int runAutonTask() {
  autonomousMain();
  return 0;
}

// Thread wrapper for running brain in background from driverControl
static int brainThreadFunc() {
  matchBrain.run();
  return 0;
}


void onIntakeFull() {
  Controller.Screen.clearLine();
  Controller.Screen.print("Intake Full");
  Controller.rumble(".");
}

bool mbool = false;
bool cbool = false;
bool stopbool = false;
void usercontrol(void) {
  // change placeholder function to do whatever when intake is full
  IntakeFull(onIntakeFull);
  while (1) {
    if (Controller.ButtonR1.pressing()) {
      intakeBalls();
    } else if (Controller.ButtonR2.pressing()) {
      // outakes middle goal bottom, no sort
      outakeBallsBottom();
    } else if (Controller.ButtonL1.pressing()) {
      // outakes long goal
      outakeBallsMiddle();
    } else if (Controller.ButtonL2.pressing()) {
      // outakes middle goal top
      outakeBallsTop();
    } else if (Controller.ButtonA.pressing()) {
      intakeBallsSlow();
    } else {
      stopIntake();
    }

    if (Controller.ButtonA.pressing()) {
      mbool = !mbool;
      waitUntil(!Controller.ButtonA.pressing());
    }
    MatchLoader.set(mbool);

    // if (Controller.ButtonY.pressing()) {
    //   stopbool = !stopbool;
    //   waitUntil(!Controller.ButtonY.pressing());
    // }
    // Stopper.set(stopbool);

    chassis.control_arcade();

    wait(20, msec);
  }
}

// =============================================================================
// LOCALIZER DEBUG FUNCTION
// Call this from the main loop to test/tune localizer behavior
// Press L2 to reset odometry to test position, prints ODOM vs LOC vs GPS
// =============================================================================
void debugLocalizerTest() {
    static bool l2WasPressed = false;
    static int debugCounter = 0;
    
    // L2 button handling - reset odometry to test position
    bool l2Pressed = Controller.ButtonL2.pressing();
    bool l2Rising = l2Pressed && !l2WasPressed;
    l2WasPressed = l2Pressed;
    
    if (l2Rising) {
        // Reset odometry to fixed position for calibration test
        // Change these values to test different positions
        float testX_cm = 121.0f;
        float testY_cm = 121.0f;
        Pose currentPose = localizer.getPose();
        float testX_in = testX_cm / 2.54f;
        float testY_in = testY_cm / 2.54f;
        chassis.set_coordinates(testX_in, testY_in, currentPose.heading_deg);
        printf("\n========== LOCALIZER DEBUG START ==========\n");
        printf("Odometry set to: (%.0f, %.0f) H:%.1f\n", testX_cm, testY_cm, currentPose.heading_deg);
        printf("Format: ODOM | LOC | L_GPS | R_GPS\n\n");
        Controller.Screen.clearScreen();
        Controller.Screen.print("Odom: %.0f,%.0f", testX_cm, testY_cm);
    }
    
    // Print comparison every 10 iterations (~100ms at 10ms loop)
    if (debugCounter++ % 10 == 0) {
        // Raw odometry (convert inches to cm)
        float odomX_cm = chassis.get_X_position() * 2.54f;
        float odomY_cm = chassis.get_Y_position() * 2.54f;
        float odomH = chassis.get_absolute_heading();
        
        // Localizer (EKF fused) and debug info
        Pose locPose = localizer.getPose();
        LocalizerDebug locDebug = localizer.getDebug();
        
        // Raw GPS readings
        GpsReading leftGps, rightGps;
        GPS.getLeft(leftGps);
        GPS.getRight(rightGps);
        
        // Print to terminal: ODOM | LOC | L_GPS | R_GPS | Variance
        printf("ODOM:(%.1f,%.1f,H:%.1f) LOC:(%.1f,%.1f,H:%.1f) L:(%.1f,%.1f,q%d,v%.1f) R:(%.1f,%.1f,q%d,v%.1f)\n",
               odomX_cm, odomY_cm, odomH,
               locPose.x_cm, locPose.y_cm, locPose.heading_deg,
               leftGps.x_cm, leftGps.y_cm, leftGps.quality, locDebug.variance_left_cm,
               rightGps.x_cm, rightGps.y_cm, rightGps.quality, locDebug.variance_right_cm);
        
        // Show on controller screen
        Controller.Screen.clearScreen();
        Controller.Screen.print("OD:%.0f,%.0f LC:%.0f,%.0f", odomX_cm, odomY_cm, locPose.x_cm, locPose.y_cm);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("L:%.0f,%.0f R:%.0f,%.0f", leftGps.x_cm, leftGps.y_cm, rightGps.x_cm, rightGps.y_cm);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("vL:%.1f vR:%.1f", locDebug.variance_left_cm, locDebug.variance_right_cm);
    }
}

void pre_auton() {
  activeTeamColor = BallBlue;
  MatchLoader.set(false);
  Aligner.set(true);
    // Ensure PID constants are initialized before any autonomous/path calls
  configureChassis();
  
  // Wait for GPS to get valid reading, then sync IMU heading to GPS heading
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Waiting for GPS...");
  
  // Wait up to 3 seconds for GPS quality
  int gpsWaitStart = Brain.Timer.system();
  while (Brain.Timer.system() - gpsWaitStart < 3000) {
    if (LGPS.quality() >= 90 || RGPS.quality() >= 90) break;
    wait(50, msec);
  }
  
  // Sync IMU/chassis heading to ACTUAL GPS sensor heading (not DualGPS which returns IMU)
  // Use whichever GPS has better quality, or average if both good
  float gpsHeading;
  int lQual = LGPS.quality();
  int rQual = RGPS.quality();
  if (lQual >= 90 && rQual >= 90) {
    // Average both (handle wrap-around)
    float lH = LGPS.heading();
    float rH = RGPS.heading();
    // Simple average works if headings are close (both should be similar)
    float diff = rH - lH;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    gpsHeading = lH + diff / 2.0f;
    if (gpsHeading < 0) gpsHeading += 360;
    if (gpsHeading >= 360) gpsHeading -= 360;
  } else if (lQual >= 90) {
    gpsHeading = LGPS.heading();
  } else if (rQual >= 90) {
    gpsHeading = RGPS.heading();
  } else {
    gpsHeading = 0;  // Fallback
  }
  
  chassis.set_heading(gpsHeading);
  Inertial.setHeading(gpsHeading, degrees);

  printf("Heading: %f", gpsHeading);
  
  // Initialize Localizer with current GPS position
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Init Localizer...");
  
  if (localizer.initFromGPS(2000)) {
    Pose p = localizer.getPose();
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Pos: %.0f,%.0f H:%.0f", p.x_cm, p.y_cm, p.heading_deg);
  } else {
    // Fallback: use GPS center position directly
    float gpsX = GPS.xPosition();
    float gpsY = GPS.yPosition();
    localizer.resetPose(gpsX, gpsY, gpsHeading);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Fallback: %.0f,%.0f", gpsX, gpsY);
  }
  
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("GPS H: %.1f", gpsHeading);
  wait(500, msec);

  // Spawn background color-sort and intake threads
  thread colorTopThread(onTopDetectedThread);
  thread colorBottomThread(onBottomDetectedThread);
  colorBottomThread.setPriority(11);
  colorTopThread.setPriority(11);
  thread intakeThread(intakeTask);
  intakeThread.setPriority(10);

}

void mock_send_link_data() {
    float test_x = 0.0;

    while(true) {
        // send fake position that moves over time so you can see it changing
        link.set_remote_location( test_x, 0.5f, 90.0f, 1, false );
        test_x += 0.1f;
        if( test_x > 1.8f ) test_x = -1.8f;  // bounce within field bounds

        // send some fake detections too
        DETECTION_OBJECT fake_detections[2];
        fake_detections[0].mapLocation.x = 0.5f;
        fake_detections[0].mapLocation.y = 0.5f;
        fake_detections[0].mapLocation.z = 0.0f;
        fake_detections[1].mapLocation.x = -0.5f;
        fake_detections[1].mapLocation.y = -0.5f;
        fake_detections[1].mapLocation.z = 0.0f;
        link.set_remote_detections( fake_detections, 2 );

        this_thread::sleep_for(100);
    }
}

void mock_recieve_link_data() {
  while(true) {
    float remote_x, remote_y, remote_heading;
    bool is_stuck;
    link.get_remote_location( remote_x, remote_y, remote_heading, is_stuck );
    printf( "x: %.2f  y: %.2f  h: %.2f  stuck: %d\n", 
            remote_x, remote_y, remote_heading, (int)is_stuck );

    // also print detections
    ai::robot_link::detection_pos dets[MAX_LINK_DETECTIONS];
    int32_t det_count = 0;
    link.get_remote_detections( dets, det_count );
    printf( "detections: %d\n", det_count );
    for( int i = 0; i < det_count; i++ ) {
        printf( "  [%d] x: %.2f  y: %.2f  z: %.2f\n", i, dets[i].x, dets[i].y, dets[i].z );
    }

    this_thread::sleep_for(100);
  }
}

int main() {
  pre_auton();

  // local storage for latest data from the Jetson Nano
  static AI_RECORD local_map;

  // Run at about 15Hz
  // int32_t loop_time = 33;

  // start the status update display
  thread t1(dashboardTask);

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(auto_Isolation);
  Competition.drivercontrol(usercontrol);

  // mock data test
  if (MANAGER_ROBOT) {
    mock_send_link_data();
  } else {
    mock_recieve_link_data();
  }

  // this_thread::sleep_for(loop_time);

  // // Jetson packet watchdog state
  // int32_t lastPacketCount = 0;
  // int     staleCycles     = 0;
  // bool    jetsonAlerted   = false;

  // while (true) {
  //   printf("x: %f, y: %f, h: %f \n", chassis.get_X_position(), chassis.get_Y_position(), Inertial.heading());
  // }
  // while(1) {
  //     // get last map data
  //     jetson_comms.get_data( &local_map );

  //     // set our location to be sent to partner robot
  //     link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status );

  //     // Update Jetson with our localizer position (EKF-fused odometry + GPS)
  //     // This position is used by Jetson for object map position calculations
  //     Pose currentPose = localizer.getPose();
  //     jetson_comms.set_localizer_position(currentPose.x_cm, currentPose.y_cm, currentPose.heading_deg);

  //     // request new data    
  //     // NOTE: This request should only happen in a single task.    
  //     jetson_comms.request_map();

  //     // --- Jetson packet watchdog ---
  //     int32_t currentPackets = jetson_comms.get_packets();
  //     if (currentPackets > 10) {
  //         if (currentPackets == lastPacketCount) {
  //             staleCycles++;
  //         } else {
  //             staleCycles = 0;
  //             if (jetsonAlerted) {
  //                 jetsonAlerted = false;
  //                 Controller.Screen.clearLine(3);
  //                 Controller.Screen.setCursor(3, 1);
  //                 Controller.Screen.print("Jetson: OK");
  //             }
  //         }
  //         // ~2 seconds of no new packets triggers alert (loop_time ~33ms)
  //         if (staleCycles > 60 && !jetsonAlerted) {
  //             jetsonAlerted = true;
  //             Controller.rumble(".-.-.-.-.-.");
  //             Controller.Screen.clearLine(3);
  //             Controller.Screen.setCursor(3, 1);
  //             Controller.Screen.print("!! JETSON DISCONNECT !!");
  //         }
  //     }
  //     lastPacketCount = currentPackets;

  //     // Allow other tasks to run
  //     this_thread::sleep_for(loop_time);
  // }
}