/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.h                                              */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Header for AI robot movement functions                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#pragma once

#include <vex.h>
#include <robot-config.h>
#include <vector>
#include <utility>
#include "astar.h"
#include "intake.h"

enum OBJECT {
    BallBlue,
    BallRed,
    BallUndefined
};

using namespace vex;

// Calculates the distance to a given target (x, y)
double distanceTo(double target_x, double target_y);

// Moves the robot to a specified position and orientation
void moveToPosition(double target_x, double target_y, double target_theta);

// Finds a target object based on the specified type
DETECTION_OBJECT findTarget(OBJECT type);

// Drives to the closest specified object
void goToObject(OBJECT type);

// Turns the robot to a specific angle with given tolerance and speed
void turnTo(double angle, int tolerance, int speed);

// Drives the robot in a specified heading for a given distance and speed
void driveFor(int heading, double distance, int speed);

// Legacy rotational intake (blocking spinFor with optional drive)
void runIntakeForRotations(vex::directionType dir, int rotations, bool driveForward);

void goToGoal();

void emergencyStop();

// Read obstacle status reported by Jetson via synthetic detection
// (classID == 100). Returns true if data is present; distance_m is
// obstacle depth in meters; stop is true when distance_m <= 0.35m.
bool getObstacleStatus(double &distance_m, bool &stop);

// Simple forward-drive safety test:
// - Drives slowly forward
// - Continuously polls getObstacleStatus
// - Stops drive + rumbles controller if STOP condition is hit.
void testObstacleStopForward();

// Tests the path planning algorithm
void testPathPlanning();

// PID performance test: 6 consecutive 20-inch drives with timing
void pidtest();

// Pure Pursuit path follower with optional GPS sensor fusion
// Uses motor encoders + IMU for smooth tracking
// useGPS=true: periodically blends GPS to correct encoder drift (recommended)
// useGPS=false: encoders only (faster but drifts over distance)
bool purePursuitFollowPath(const std::vector<std::pair<double,double>>& path, 
                           float baseVelocity = 7.0f,      // base voltage (0-12)
                           float lookaheadDist = 15.0f,    // lookahead distance in cm (smaller = tighter)
                           float endTolerance = 8.0f,      // how close to end point to stop (cm)
                           float endHeading = -1.0f,       // final heading (-1 = don't turn)
                           bool useGPS = true);            // sensor fusion with GPS

// Test function for pure pursuit - plans A* path then follows with pure pursuit
void testPurePursuit();

// OLD Pure Pursuit path follower (simpler steering, more aggressive gains)
// Uses Localizer for position but with old encoder dead reckoning logic
bool purePursuitFollowPathOld(const std::vector<std::pair<double,double>>& path, 
                              float baseVelocity = 4.0f,
                              float lookaheadDist = 25.0f,
                              float endTolerance = 3.0f,
                              float endHeading = -1.0f,
                              bool useGPS = true);

// Test function for OLD pure pursuit
void testPurePursuitOld();

// Multi-point navigation - visits multiple points in optimal order
std::vector<astar::Point> orderPointsNearest(const std::vector<astar::Point>& points, double startX, double startY);
bool navigateMultiplePoints(const std::vector<astar::Point>& targetPoints, bool autoOrder = true);
bool navigateContinuousPath(const std::vector<astar::Point>& targetPoints, bool autoOrder = true);  // One continuous motion
void runMultiPointPath();  // Quick test with hardcoded points
void testMultiPoint();     // Interactive test

// Localization test and display
void testLocalization();      // EKF localization test with scripted motions
void showLocalizerStatus();   // Display current localizer status on controller

// Test function for boomerang drive_to_pose
void testBoomerang();

// Test function for drive commands after front/back flip
void testDriveCommands();

// Calibration system: back up from wall and reset localizer to known position
int findNearestCalibCorner();              // Returns index 0-3 of nearest corner
bool doCalibrationAtCorner(int cornerIndex); // Core calibration at specified corner
bool autoCalibrate();                       // Auto-calibrate at nearest corner
void calibrationBackup();                   // Interactive calibration with manual selection