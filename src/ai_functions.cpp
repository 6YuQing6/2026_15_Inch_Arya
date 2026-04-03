/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.cpp                                            */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Helper movement functions for VEX AI program              */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#include "vex.h"
#include "ai_functions.h"
#include "astar.h"
#include "field_map.h"
#include "Localizer.h"
#include <cmath>
#include <string>
#include <iostream>
#include <vector>
#include <utility>

// External Localizer instance (defined in main.cpp)
extern Localizer localizer;
using namespace vex;
using namespace std;


// Calculates the distance to the coordinates from the current robot position
double distanceTo(double target_x, double target_y){
    Pose p = localizer.getPose();
    double distance = sqrt(pow((target_x - p.x_cm), 2) + pow((target_y - p.y_cm), 2));
    return distance;
}

// Calculates the bearing to drive to the target coordinates in a straight line aligned with global coordinate/heading system.
double calculateBearing(double currX, double currY, double targetX, double targetY) {
    // Calculate the difference in coordinates
    double dx = targetX - currX;
    double dy = targetY - currY;

    // Calculate the bearing in radians
    double bearing_rad = atan2(dy, dx);

    // Convert to degrees
    double bearing_deg = bearing_rad * 180 / M_PI;

    // Normalize to the range 0 to 360
    if (bearing_deg < 0) {
        bearing_deg += 360;
    }

    // Convert from mathematical to navigation coordinates
    bearing_deg = fmod(90 - bearing_deg, 360);
    if (bearing_deg < 0) {
        bearing_deg += 360;
    }

    return bearing_deg;
}

// Turns the robot to face the angle specified, taking into account a tolerance and speed of turn.
void turnTo(double angle, int tolerance, int speed){
    double current_heading = localizer.getPose().heading_deg;
    double angle_to_turn = angle - current_heading;

    // Normalize the angle to the range [-180, 180]
    while (angle_to_turn > 180) angle_to_turn -= 360;
    while (angle_to_turn < -180) angle_to_turn += 360;

    // Determine the direction to turn (left or right)
    turnType direction = angle_to_turn > 0 ? turnType::right : turnType::left;
    Drivetrain.turn(direction, speed, velocityUnits::pct);
    while (1) {
    
        current_heading = localizer.getPose().heading_deg;
        // Check if the current heading is within a tolerance of degrees to the target
        if (current_heading > (angle - tolerance) && current_heading < (angle + tolerance)) {
            break;
        }

    }
    Drivetrain.stop();
}

// Moves the robot toward the target at the specificed heading, for a distance at a given speed.
void driveFor(int heading, double distance, int speed){
    // Determine the smallest degree of turn
    double angle_to_turn = heading - localizer.getPose().heading_deg;
    while (angle_to_turn > 180) angle_to_turn -= 360;
    while (angle_to_turn < -180) angle_to_turn += 360;

    // Decide whether to move forward or backward
    // Allos for a 5 degree margin of error that defaults to forward
    directionType direction = fwd;
    if (std::abs(angle_to_turn) > 105) {
        angle_to_turn += angle_to_turn > 0 ? -180 : 180;
        direction = directionType::rev;
    } else if (std::abs(angle_to_turn) < 75) {
        angle_to_turn += angle_to_turn > 0 ? 180 : -180;
        direction = directionType::fwd;
    }

    Drivetrain.driveFor(direction, distance, vex::distanceUnits::cm, speed, velocityUnits::pct);
}

// Method that moves to a given (x,y) position and a desired target theta to finish movement facing
void moveToPosition(double target_x, double target_y, double target_theta = -1) {
    // Calculate the angle to turn to face the target
    Pose p = localizer.getPose();
    double initialHeading = calculateBearing(p.x_cm, p.y_cm, target_x, target_y);
    // Turn to face the target
    //turnTo(intialHeading, 3, 10);
    Drivetrain.turnToHeading(initialHeading, rotationUnits::deg, 10, velocityUnits::pct);
    double distance = distanceTo(target_x, target_y);
    // Move to the target, only 30% of total distance to account for error
    driveFor(initialHeading, distance*0.3, 30);

    // Recalculate the heading and distance to the target
    p = localizer.getPose();
    double heading = calculateBearing(p.x_cm, p.y_cm, target_x, target_y);
    //turnTo(heading, 3, 10);
    Drivetrain.turnToHeading(heading, rotationUnits::deg, 10, velocityUnits::pct);
    distance = distanceTo(target_x, target_y);
    // Move to the target, completing the remaining distance
    driveFor(heading, distance, 20);

    // Turn to the final target heading if specified, otherwise use current heading
    if (target_theta == -1){
        target_theta = localizer.getPose().heading_deg;
    }
    //turnTo(target_theta, 2, 10);
    Drivetrain.turnToHeading(target_theta, rotationUnits::deg, 10, velocityUnits::pct);
}

// Function to find the target object based on type and return its record
DETECTION_OBJECT findTarget(OBJECT type){
    DETECTION_OBJECT target;
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);
    double lowestDist = 1000000;
    // Iterate through detected objects to find the closest target of the specified type
    for(int i = 0; i < local_map.detectionCount; i++) {
        double distance = distanceTo(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y);
        if (distance < lowestDist && local_map.detections[i].classID == (int) type) {
            target = local_map.detections[i];
            lowestDist = distance;
        }
    }
    return target;
}

// Function to drive to an object based on detection
void goToObject(OBJECT type){
    DETECTION_OBJECT target = findTarget(type);
    // If no target found, turn and try to find again
    if (target.mapLocation.x == 0 && target.mapLocation.y == 0){
        //Drivetrain.turnFor(45, rotationUnits::deg, 50, velocityUnits::pct);
        Drivetrain.turn(turnType::left);
        wait(2, sec);
        Drivetrain.stop();
        target = findTarget(type);
    }
    // Move to the detected target's position
    moveToPosition(target.mapLocation.x*100, target.mapLocation.y*100);
}

void runIntakeForRotations(vex::directionType dir, int rotations, bool driveForward) {
    ZeroStage.spinFor(dir, rotations, vex::rotationUnits::rev, false);
    FirstStage.spinFor(dir, rotations, vex::rotationUnits::rev, false);
    SecondStage.spinFor(dir, rotations, vex::rotationUnits::rev, !driveForward);
    ThirdStage.spinFor(dir, rotations, vex::rotationUnits::rev, !driveForward);
    if (driveForward)
        Drivetrain.driveFor(directionType::fwd, 70, vex::distanceUnits::cm, 40, velocityUnits::pct);
}

void goToGoal() {
    int closestGoalX = 0;
    int closestGoalY = 0;
    int heading = 0;

    if (distanceTo(122, 0) < distanceTo(-122, 0)) {
        closestGoalX = 122;
        heading = 90;
    } else {
        closestGoalX = -122;
        heading = 270;
    }
    if (distanceTo(0, 122) < distanceTo(0, -122)) {
        closestGoalY = 122;
    } else {
        closestGoalY = -122;
    }

    moveToPosition(closestGoalX, closestGoalY, heading);

}

void emergencyStop() {
    chassis.drive_with_voltage(0, 0);
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
}

// Query Jetson obstacle summary encoded as a synthetic detection
// with classID == 100 (OBSTACLE_CLASS_ID in V5Comm.py).
// Returns true if data is present; distance_m is depth in meters;
// stop is true when distance_m <= 0.35m.
bool getObstacleStatus(double &distance_m, bool &stop) {
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);

    const int OBSTACLE_CLASS_ID = 100;

    distance_m = 0.0;
    stop = false;

    for (int i = 0; i < local_map.detectionCount; i++) {
        if (local_map.detections[i].classID == OBSTACLE_CLASS_ID) {
            distance_m = local_map.detections[i].depth;
            stop = (distance_m <= 0.35);
            return true;
        }
    }
    return false;
}

// Median of a small array (used for obstacle distance filtering).
// Sorts a copy so the original is untouched.
static double medianOf(double *buf, int n) {
    double tmp[16];
    for (int i = 0; i < n; i++) tmp[i] = buf[i];
    for (int i = 0; i < n - 1; i++)
        for (int j = i + 1; j < n; j++)
            if (tmp[j] < tmp[i]) { double t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
    return tmp[n / 2];
}

// Obstacle distance threshold (meters) — tune this one constant everywhere.
static const double OBSTACLE_STOP_DIST = 0.35;

// Median filter window size.  At 5ms polling this is 25ms of data.
// A single spike cannot move the median; 3 of 5 must agree.
static const int OBS_FILTER_SIZE = 12;

// Simple forward-drive safety test driven by Jetson obstacle status.
// Uses a median filter on distance so a single spurious reading
// cannot trigger a false stop.
void testObstacleStopForward() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Obstacle test fwd");

    LeftDrive.spin(directionType::rev, 8, voltageUnits::volt);
    RightDrive.spin(directionType::rev, 8, voltageUnits::volt);

    static AI_RECORD obs_map;
    const int OBS_ID = 100;
    int screenTick = 0;

    // Ring buffer for median filtering
    double distBuf[OBS_FILTER_SIZE];
    for (int i = 0; i < OBS_FILTER_SIZE; i++) distBuf[i] = 999.0;
    int bufIdx = 0;

    while (true) {
        jetson_comms.get_data(&obs_map);

        // Find obstacle reading and push into ring buffer
        double rawDist = 999.0;
        bool haveObs = false;
        for (int i = 0; i < obs_map.detectionCount; i++) {
            if (obs_map.detections[i].classID == OBS_ID) {
                rawDist = obs_map.detections[i].depth;
                haveObs = true;
                break;
            }
        }
        distBuf[bufIdx] = rawDist;
        bufIdx = (bufIdx + 1) % OBS_FILTER_SIZE;

        double filteredDist = medianOf(distBuf, OBS_FILTER_SIZE);

        if (haveObs && filteredDist <= OBSTACLE_STOP_DIST) {
            // Immediately reverse at max voltage
            LeftDrive.spin(directionType::fwd, 12, voltageUnits::volt);
            RightDrive.spin(directionType::fwd, 12, voltageUnits::volt);
            Controller.rumble("..");
            Controller.Screen.setCursor(2, 1);
            Controller.Screen.print("STOP %.2fm", filteredDist);

            wait(250, msec);

            LeftDrive.stop(brakeType::brake);
            RightDrive.stop(brakeType::brake);
            wait(100, msec);

            double currentHeading = localizer.getPose().heading_deg;
            double newHeading = fmod(currentHeading + 180.0, 360.0);
            chassis.turn_to_angle(newHeading);

            return;
        }

        // Screen + cancel check every ~150ms
        if (++screenTick >= 30) {
            screenTick = 0;
            if (Controller.ButtonB.pressing()) {
                emergencyStop();
                Controller.Screen.setCursor(2, 1);
                Controller.Screen.print("Cancelled");
                return;
            }
            if (haveObs) {
                Controller.Screen.setCursor(2, 1);
                Controller.Screen.print("d=%.2f m=%.2f", rawDist, filteredDist);
            }
        }

        this_thread::sleep_for(5);
    }
}

// Test A* path planning and waypoint following
void testPathPlanning() {
    Controller.Screen.clearScreen();
    Controller.Screen.print("A* Path Test");
    wait(200, msec);
    
    // Initialize target coordinates (can be adjusted with arrows)
    double target_x = 0.0;  // cm
    double target_y = 0.0;   // cm
    
    // Allow user to adjust target coordinates dynamically
    bool coordinatesLocked = false;
    
    while (!coordinatesLocked) {
        // Display current target coordinates
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("X: %.1f", target_x);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Y: %.1f", target_y);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("A=Lock");
        
        // Up arrow: increase X
        if (Controller.ButtonUp.pressing()) {
            target_x += 1.0;
        }
        
        // Down arrow: decrease X
        if (Controller.ButtonDown.pressing()) {
            target_x -= 1.0;
        }
        
        // Left arrow: decrease Y
        if (Controller.ButtonLeft.pressing()) {
            target_y -= 1.0;
        }
        
        // Right arrow: increase Y
        if (Controller.ButtonRight.pressing()) {
            target_y += 1.0;
        }
        
        // Button A: lock coordinates and start pathfinding
        if (Controller.ButtonA.pressing()) {
            waitUntil(!Controller.ButtonA.pressing());
            coordinatesLocked = true;
            wait(200, msec);
        }
        
        wait(20, msec);
    }
    
    // Coordinates locked - now get current position and start pathfinding
    Controller.Screen.clearScreen();
    Controller.Screen.print("Getting position...");
    wait(300, msec);
    
    Pose currPose = localizer.getPose();
    double curr_x = currPose.x_cm;
    double curr_y = currPose.y_cm;
    double curr_h = currPose.heading_deg;
    
    // Display current position and target
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Current: %.1f,%.1f", curr_x, curr_y);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Target: %.1f,%.1f", target_x, target_y);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Planning...");
    wait(500, msec);
    
    // Create and populate FieldMap with obstacles
    FieldMap fieldMap;
    fieldMap.populateStandardField();
    
    // Call A* to find path using actual robot geometry (13.5in x 13.5in)
    const double robot_width_in = 13.5;
    const double robot_radius_cm = robot_width_in * 2.54 * 0.5; // ~17.145 cm
    const double safety_margin_cm = 0.0;                        // tune 4–10 cm
    const double grid_resolution_cm = 60.96/2;                    // keep coarse grid as-is

    std::vector<astar::Point> path = astar::findPath(
        fieldMap,
        curr_x, curr_y,
        target_x, target_y,
        grid_resolution_cm,
        robot_radius_cm,
        safety_margin_cm
    );
    
    // Check if path was found
    if (path.empty()) {
        Controller.Screen.clearScreen();
        Controller.Screen.print("No path found!");
        wait(2000, msec);
        return;
    }
    
    // Print entire path to console BEFORE robot moves
    cout << "===== A* PATH PLANNED =====\n";
    cout << "Start: (" << curr_x << "," << curr_y << ") H=" << curr_h << "\n";
    cout << "Target: (" << target_x << "," << target_y << ")\n";
    cout << "Total Waypoints: " << path.size() << "\n";
    cout << "Full Path:\n";
    for (size_t i = 0; i < path.size(); i++) {
        cout << "  WP" << i << ": (" << path[i].first << "," << path[i].second << ")\n";
    }
    cout << "===========================\n";
    
    // Display path info on controller in requested order
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Path planned");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("List all waypoints");
    wait(700, msec);
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Waypoints: %d", (int)path.size());
    wait(1000, msec);
    
    // Follow the path waypoint by waypoint
    for (size_t i = 0; i < path.size(); i++) {
        double wp_x = path[i].first;
        double wp_y = path[i].second;
        
        // Get current position from Localizer
        Pose robotPose = localizer.getPose();
        double robot_x = robotPose.x_cm;
        double robot_y = robotPose.y_cm;
        double robot_h = robotPose.heading_deg;
        
        // Calculate bearing to waypoint
        double bearing = calculateBearing(robot_x, robot_y, wp_x, wp_y);
        double dx = wp_x - robot_x;
        double dy = wp_y - robot_y;
        double dist_cm = sqrt(dx*dx + dy*dy);
        
        // Skip if already at waypoint
        if (dist_cm < 5.0) {
            cout << "Waypoint " << (int)(i+1) << " (" << wp_x << "," << wp_y << ") reached\n";
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Waypoint %d/%d (%.1f, %.1f) reached", (int)(i+1), (int)path.size(), wp_x, wp_y);
            continue;
        }
        
        // Display waypoint info
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Moving to WP %d/%d", (int)(i+1), (int)path.size());
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Dist: %.1f cm", dist_cm);
        
        // Turn and drive to waypoint
        double dist_in = dist_cm / 2.54;
        chassis.set_heading(robot_h);
        chassis.turn_to_angle(bearing);
        //wait(20, msec);
        chassis.drive_distance(dist_in);
        //wait(20, msec);
        
        // Print waypoint reached to terminal and controller
        cout << "Waypoint " << (int)(i+1) << " (" << wp_x << "," << wp_y << ") reached\n";
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Waypoint %d/%d (%.1f, %.1f) reached", (int)(i+1), (int)path.size(), wp_x, wp_y);
    }
    
    // Final stop and report
    emergencyStop();
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
    wait(300, msec);
    
    Pose finalPose = localizer.getPose();
    double final_x = finalPose.x_cm;
    double final_y = finalPose.y_cm;
    double final_h = finalPose.heading_deg;
    double err = sqrt(pow(final_x - target_x, 2) + pow(final_y - target_y, 2));
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Arrived!");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Pos: %.1f,%.1f H:%.0f", final_x, final_y, final_h);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Error: %.1f cm", err);
    
    wait(3000, msec);
}

// PID performance test: drives forward 6 times, 20 inches each
void pidtest() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("PID Test Starting...");
    wait(1000, msec);

    const double distance_in = 20.0; // 20 inches per segment
    const int segments = 6;

    Brain.Timer.reset();
    double last_time = 0.0;

    for (int i = 1; i <= segments; i++) {
        // Drive forward 20 inches
        chassis.drive_distance(distance_in);

        // Get elapsed time since last segment
        double current_time = Brain.Timer.time(msec) / 1000.0; // convert to seconds
        double segment_time = current_time - last_time;
        last_time = current_time;

        // Display on controller
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Segment %d/6", i);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Time: %.2f sec", segment_time);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("Total: %.2f sec", current_time);

        wait(1500, msec);
    }

    // Final summary
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Test Complete!");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Total: %.2f sec", last_time);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Avg: %.2f sec", last_time / segments);
    wait(3000, msec);
}

// ============================================================================
// PURE PURSUIT PATH FOLLOWER
// Uses motor encoders for position tracking, IMU for heading
// GPS only used for initial position
// ============================================================================

// Find the lookahead point on the path at approximately lookahead distance ahead
static std::pair<double, double> findLookaheadPoint(
    const std::vector<std::pair<double,double>>& path,
    double robotX, double robotY,
    float lookaheadDist,
    size_t& nearestIdx)
{
    // Find the closest point on path to robot
    double minDist = 1e9;
    size_t closestIdx = 0;
    for (size_t i = 0; i < path.size(); i++) {
        double dx = path[i].first - robotX;
        double dy = path[i].second - robotY;
        double d = sqrt(dx*dx + dy*dy);
        if (d < minDist) {
            minDist = d;
            closestIdx = i;
        }
    }
    nearestIdx = closestIdx;
    
    // Search forward from closest point to find lookahead intersection
    for (size_t i = closestIdx; i < path.size() - 1; i++) {
        double x1 = path[i].first;
        double y1 = path[i].second;
        double x2 = path[i+1].first;
        double y2 = path[i+1].second;
        
        // Vector from robot to segment start
        double dx = x1 - robotX;
        double dy = y1 - robotY;
        
        // Segment direction vector
        double fx = x2 - x1;
        double fy = y2 - y1;
        
        // Quadratic coefficients for circle-line intersection
        double a = fx*fx + fy*fy;
        double b = 2.0 * (dx*fx + dy*fy);
        double c = dx*dx + dy*dy - lookaheadDist*lookaheadDist;
        
        double discriminant = b*b - 4*a*c;
        
        if (discriminant >= 0 && a > 1e-6) {
            double sqrtDisc = sqrt(discriminant);
            double t = (-b + sqrtDisc) / (2*a);
            
            if (t >= 0.0 && t <= 1.0) {
                return {x1 + t*fx, y1 + t*fy};
            }
        }
    }
    
    // If no intersection found, return the last point on path
    return path.back();
}

// Convert navigation heading (0=north, CW positive) to math angle (0=east, CCW positive) in radians
static double navToMathRad(double navDeg) {
    return (90.0 - navDeg) * M_PI / 180.0;
}

// Pure pursuit path follower with sensor fusion (odometry tracking wheels + GPS)
// Uses dedicated tracking wheel encoders (forward + sideways) via JAR-Template odometry
// endHeading: final heading in degrees (0-360), or -1 to skip final turn
// useGPS: true = sensor fusion (GPS corrects odometry drift), false = odometry only
bool purePursuitFollowPath(const std::vector<std::pair<double,double>>& pathIn,
                           float baseVelocity,
                           float lookaheadDist,
                           float endTolerance,
                           float endHeading,
                           bool useGPS)
{
    if (pathIn.empty()) {
        return false;
    }
    
    std::vector<std::pair<double,double>> path = pathIn;
    
    // Robot parameters
    const int maxIterations = 3000;  // ~30 seconds at 10ms loop
    const float minVelocity = 1.5f;  // Reduced from 2.0 for smoother slow approach
    const float steeringDeadband = 2.0f;  // Increased from 1.5 to reduce wobble on straights
    
    // Speed ramping: fast for first 70%, slow for last 30%
    const float fastVelocity = 7.0f;    // Reduced from 8.0 for better control
    const float slowdownPoint = 0.65f;  // Start slowing earlier (65% instead of 70%)
    const float minDistForFast = 60.0f * 2.54f;  // 60 inches in cm (~152cm) - below this, use slow speed only
    
    // Steering smoothing (exponential moving average to reduce oscillations)
    float smoothedTurnOutput = 0.0f;
    const float turnSmoothingFactor = 0.3f;  // 0.3 = 30% new value, 70% old value
    
    // Center structure collision recovery parameters
    // Center structure is 54x54cm at (0,0) - detect when within ~45cm of center
    const float centerStructureRadius = 45.0f;    // cm - trigger when this close to (0,0)
    const float centerStructureClearance = 75.0f; // cm - must escape to this distance
    const float escapeVelocity = 12.0f;  // Voltage for escape maneuver
    const int maxEscapeTime = 3000;     // ms max escape time
    
    // Calculate total path distance for progress tracking
    double totalPathDist = 0.0;
    for (size_t i = 1; i < path.size(); i++) {
        double dx = path[i].first - path[i-1].first;
        double dy = path[i].second - path[i-1].second;
        totalPathDist += sqrt(dx*dx + dy*dy);
    }
    
    // For short paths, skip fast phase entirely
    bool useSlowOnly = (totalPathDist < minDistForFast);
    if (useSlowOnly) {
        cout << "Pure Pursuit: Short path (" << totalPathDist << "cm < " << minDistForFast << "cm), using slow speed only\n";
    }
    
    // Get initial position from Localizer (already fused)
    Pose startPose = localizer.getPose();
    double startX_cm = startPose.x_cm;
    double startY_cm = startPose.y_cm;
    double initHeading = startPose.heading_deg;
    
    // Sync JAR odometry with Localizer position (JAR uses inches)
    float startX_in = startX_cm / 2.54f;
    float startY_in = startY_cm / 2.54f;
    chassis.set_coordinates(startX_in, startY_in, initHeading);
    task::sleep(20);  // Allow odometry task to sync
    
    cout << "Pure Pursuit Start (Localizer): X=" << startX_cm << " Y=" << startY_cm << " H=" << initHeading << "\n";
    
    // End point is the LAST point in path (should be actual target, not cell center)
    double endX = path.back().first;
    double endY = path.back().second;
    
    int iteration = 0;
    size_t nearestIdx = 0;
    bool singlePointPath = (path.size() == 1);
    
    // === ENHANCED STUCK DETECTION ===
    // Two types of stuck detection:
    // 1. Motor divergence: wheels spinning but robot not moving (hit obstacle)
    // 2. No movement: robot position not changing at all (stalled, wedged, etc.)
    
    int stuckCounter = 0;              // For motor divergence detection
    int noMovementCounter = 0;         // For general no-movement detection
    const int stuckCheckInterval = 10;    // Check every 10 iterations (100ms)
    
    // Motor divergence thresholds
    const int stuckThreshold = 80;        // ~0.8 second of motor divergence = trigger
    const double motorOdomDivergence = 15.0; // cm - if motor DR diverges this much from odom
    
    // No-movement thresholds (catches stalled motors, wedged robot, etc.)
    const int noMovementThreshold = 200;  // ~2 seconds of no movement = trigger (was 1s)
    const double stuckDistThreshold = 0.3; // Must move at least 0.3cm per 100ms check (3cm/sec)
    
    double lastGpsX = startX_cm;
    double lastGpsY = startY_cm;
    
    // Motor encoder dead reckoning (separate from odom, for stuck detection)
    double motorDeadReckonX = startX_cm;
    double motorDeadReckonY = startY_cm;
    float lastLeftPos = chassis.get_left_position_in();
    float lastRightPos = chassis.get_right_position_in();
    bool hasRumbledDivergence = false;  // Only rumble once per divergence event
    bool hasRumbledNoMovement = false;  // Only rumble once per no-movement event
    
    // --- Obstacle avoidance (Jetson depth-camera, median-filtered) ---
    static AI_RECORD obs_local;
    const int OBS_CLS = 100;
    double obsDistBuf[OBS_FILTER_SIZE];
    for (int i = 0; i < OBS_FILTER_SIZE; i++) obsDistBuf[i] = 999.0;
    int obsBufIdx = 0;
    
    while (iteration++ < maxIterations) {
        // === READ POSITION FROM EKF LOCALIZER ===
        // Localizer fuses odometry + IMU + dual GPS at 100Hz
        Pose pose = localizer.getPose();
        double robotX = pose.x_cm;
        double robotY = pose.y_cm;
        double robotHeadingNav = pose.heading_deg;
        
        double distToEnd = sqrt(pow(endX - robotX, 2) + pow(endY - robotY, 2));
        
        // === MOTOR ENCODER DEAD RECKONING (for stuck detection) ===
        float leftPos = chassis.get_left_position_in();
        float rightPos = chassis.get_right_position_in();
        float deltaLeft = leftPos - lastLeftPos;
        float deltaRight = rightPos - lastRightPos;
        float motorDist_cm = ((deltaLeft + deltaRight) / 2.0f) * 2.54f;
        
        // Update motor dead reckoning position
        double headingRad = navToMathRad(robotHeadingNav);
        motorDeadReckonX += motorDist_cm * cos(headingRad);
        motorDeadReckonY += motorDist_cm * sin(headingRad);
        
        lastLeftPos = leftPos;
        lastRightPos = rightPos;
        
        // === STUCK DETECTION ===
        // Two types: (1) Motor divergence (wheels spinning, robot stuck)
        //            (2) No movement at all (stalled, wedged, etc.)
        if (iteration % stuckCheckInterval == 0) {
            // Use Localizer position for stuck detection (more reliable than raw GPS)
            double currentPosX = robotX;
            double currentPosY = robotY;
            
            double posDelta = sqrt(pow(currentPosX - lastGpsX, 2) + pow(currentPosY - lastGpsY, 2));
            
            // Calculate divergence between motor dead reckoning and fused position
            double divergence = sqrt(pow(motorDeadReckonX - robotX, 2) + pow(motorDeadReckonY - robotY, 2));
            
            bool posNotMoving = (posDelta < stuckDistThreshold);
            bool motorsDiverged = (divergence > motorOdomDivergence);
            
            // === TYPE 1: Motor Divergence (wheels spinning but robot not moving) ===
            if (posNotMoving && motorsDiverged) {
                stuckCounter++;
                
                // Rumble warning (once)
                if (!hasRumbledDivergence && stuckCounter >= 3) {
                    Controller.rumble(".-");  // Short pattern = divergence warning
                    hasRumbledDivergence = true;
                    cout << "Pure Pursuit: WARNING - Motor divergence! " << divergence << "cm\n";
                }
                
                // Full stuck - abort
                if (stuckCounter >= (stuckThreshold / stuckCheckInterval)) {
                    chassis.drive_with_voltage(0, 0);
                    chassis.drive_stop(brake);
                    Controller.rumble("---");  // Long rumble = stuck
                    cout << "Pure Pursuit: STUCK - Motors spinning but robot not moving\n";
                    cout << "  Pos delta: " << posDelta << "cm, Divergence: " << divergence << "cm\n";
                    Controller.Screen.clearScreen();
                    Controller.Screen.print("STUCK! Wheels spin");
                    chassis.drive_stop(coast);  // Reset to coast for user control
                    return false;
                }
            } else {
                // Reset divergence counter if moving normally
                if (stuckCounter > 0) {
                    stuckCounter--;
                    if (stuckCounter == 0) hasRumbledDivergence = false;
                }
                // Re-sync motor dead reckoning periodically
                if (iteration % 50 == 0) {
                    motorDeadReckonX = robotX;
                    motorDeadReckonY = robotY;
                }
            }
            
            // === TYPE 2: No Movement (robot not moving at all, any reason) ===
            // Skip this check when close to target -- small corrections are expected
            if (posNotMoving && distToEnd > 15.0) {
                noMovementCounter++;
                
                // Rumble warning at 1 second (once)
                if (!hasRumbledNoMovement && noMovementCounter >= 10) {
                    Controller.rumble("..");  // Double short = no movement warning
                    hasRumbledNoMovement = true;
                    cout << "Pure Pursuit: WARNING - No movement detected for 1s\n";
                }
                
                // Full stuck - abort after 1 second of no movement
                if (noMovementCounter >= (noMovementThreshold / stuckCheckInterval)) {
                    chassis.drive_with_voltage(0, 0);
                    chassis.drive_stop(brake);
                    Controller.rumble("...");  // Triple short = no movement stuck
                    cout << "Pure Pursuit: STUCK - No movement for 1 second\n";
                    cout << "  Fused pos: (" << currentPosX << "," << currentPosY << ")\n";
                    Controller.Screen.clearScreen();
                    Controller.Screen.print("STUCK! No movement");
                    chassis.drive_stop(coast);  // Reset to coast for user control
                    return false;
                }
            } else {
                // Reset no-movement counter if robot is moving
                noMovementCounter = 0;
                hasRumbledNoMovement = false;
            }
            
            lastGpsX = currentPosX;
            lastGpsY = currentPosY;
        }
        
        // === OBSTACLE AVOIDANCE (Jetson camera) ===
        {
            jetson_comms.get_data(&obs_local);
            double obsRaw = 999.0;
            bool obsPresent = false;
            for (int i = 0; i < obs_local.detectionCount; i++) {
                if (obs_local.detections[i].classID == OBS_CLS) {
                    obsRaw = obs_local.detections[i].depth;
                    obsPresent = true;
                    break;
                }
            }
            obsDistBuf[obsBufIdx] = obsRaw;
            obsBufIdx = (obsBufIdx + 1) % OBS_FILTER_SIZE;

            double obsFiltered = medianOf(obsDistBuf, OBS_FILTER_SIZE);

            if (obsPresent && obsFiltered <= OBSTACLE_STOP_DIST) {
                chassis.drive_with_voltage(0, 0);
                chassis.drive_stop(brake);

                LeftDrive.spin(directionType::fwd, 12, voltageUnits::volt);
                RightDrive.spin(directionType::fwd, 12, voltageUnits::volt);
                Controller.rumble("..");
                cout << "Pure Pursuit: OBSTACLE STOP  filtered=" << obsFiltered << "m\n";
                wait(250, msec);

                LeftDrive.stop(brakeType::brake);
                RightDrive.stop(brakeType::brake);
                wait(100, msec);

                double curH = localizer.getPose().heading_deg;
                double flipH = fmod(curH + 180.0, 360.0);
                chassis.turn_to_angle(flipH);

                chassis.drive_stop(coast);
                return false;
            }
        }
        
        // NOTE: GPS fusion is now handled by the Localizer EKF at 100Hz
        // No manual fusion needed here
        
        // === CENTER STRUCTURE COLLISION RECOVERY ===
        // Simple check: distance from field origin (0,0) using Localizer fused position
        double distFromCenter = sqrt(robotX * robotX + robotY * robotY);
        
        bool inCenterStructure = (distFromCenter < centerStructureRadius);
        
        if (inCenterStructure) {
            cout << "Pure Pursuit: CENTER STRUCTURE COLLISION! dist=" << distFromCenter << "cm\n";
            Controller.rumble("-");
            Controller.Screen.clearScreen();
            Controller.Screen.print("Collision! Escaping...");
            
            // Determine escape direction based on heading
            // Robot heading: 0 = +Y (north), 90 = +X (east), etc.
            // Calculate which direction (forward or backward) moves us away from center faster
            double headingRad = robotHeadingNav * M_PI / 180.0;
            // Forward direction vector
            double fwdX = sin(headingRad);  // +X component when heading east (90)
            double fwdY = cos(headingRad);  // +Y component when heading north (0)
            
            // Dot product of forward vector with position vector (from center)
            // Positive = forward points away from center (drive forward to escape)
            // Negative = forward points toward center (drive backward to escape)
            double dotProduct = fwdX * robotX + fwdY * robotY;
            
            bool driveForward = (dotProduct > 0);
            float escapeVel = driveForward ? escapeVelocity : -escapeVelocity;
            
            cout << "  Heading: " << robotHeadingNav << ", dot: " << dotProduct 
                 << ", escaping " << (driveForward ? "FORWARD" : "BACKWARD") << "\n";
            
            // Drive until clear of center structure
            int escapeStart = Brain.Timer.system();
            while (Brain.Timer.system() - escapeStart < maxEscapeTime) {
                chassis.drive_with_voltage(escapeVel, escapeVel);
                task::sleep(20);
                
                // Update position from Localizer
                Pose escapePose = localizer.getPose();
                robotX = escapePose.x_cm;
                robotY = escapePose.y_cm;
                
                // Check if we're clear (outside clearance zone)
                double escapeDistFromCenter = sqrt(robotX * robotX + robotY * robotY);
                if (escapeDistFromCenter > centerStructureClearance) {
                    cout << "  Escaped! Now at (" << robotX << "," << robotY << "), dist=" << escapeDistFromCenter << "cm\n";
                    break;
                }
            }
            
            chassis.drive_with_voltage(0, 0);
            Controller.Screen.clearScreen();
            Controller.Screen.print("Escaped - replanning");
            task::sleep(200);
            
            // Regenerate path from current position to original goal
            Pose escapedPose = localizer.getPose();
            robotX = escapedPose.x_cm;
            robotY = escapedPose.y_cm;
            
            FieldMap replanMap;
            replanMap.populateStandardField();
            const double replan_robot_radius_cm = 13.5 * 2.54 * 0.5;
            const double replan_grid_res = 60.96 / 2;
            
            std::vector<astar::Point> newPath = astar::findPath(
                replanMap, robotX, robotY, endX, endY,
                replan_grid_res, replan_robot_radius_cm, 0.0);
            
            if (!newPath.empty()) {
                path = newPath;
                nearestIdx = 0;
                singlePointPath = (path.size() == 1);
                
                chassis.set_coordinates((float)(robotX / 2.54), (float)(robotY / 2.54), escapedPose.heading_deg);
                task::sleep(20);
                
                cout << "Pure Pursuit: Replanned from (" << robotX << "," << robotY 
                     << ") to (" << endX << "," << endY << "), " << path.size() << " waypoints\n";
                Controller.Screen.clearScreen();
                Controller.Screen.print("Replanned - resuming");
            } else {
                cout << "Pure Pursuit: Replan failed! No path from (" << robotX << "," << robotY << ")\n";
                Controller.Screen.clearScreen();
                Controller.Screen.print("Replan failed!");
                chassis.drive_stop(coast);
                return false;
            }
            
            distToEnd = sqrt(pow(endX - robotX, 2) + pow(endY - robotY, 2));
        }
        
        if (distToEnd < endTolerance) {
            chassis.drive_with_voltage(0, 0);
            chassis.drive_stop(brake);  // Brake to stop precisely
            
            // Report final position from Localizer
            task::sleep(100);
            Pose finalPose = localizer.getPose();
            double finalDist = sqrt(pow(endX - finalPose.x_cm, 2) + pow(endY - finalPose.y_cm, 2));
            cout << "Pure Pursuit: Arrived! Fused dist to target: " << finalDist << "cm\n";
            cout << "  Final pose: (" << finalPose.x_cm << "," << finalPose.y_cm << ") H:" << finalPose.heading_deg << "\n";
            
            /* === GPS-BASED FINAL POSITION CORRECTION (DISABLED) ===
            // Uncomment this block if pure pursuit alone isn't precise enough
            // Continuous GPS feedback loop - drive while checking GPS
            const int maxCorrectionTime = 3000;  // 3 second timeout
            const float positionTolerance = 3.0f;  // Stop when within 3cm
            int correctionStart = Brain.Timer.system();
            int lastDebug = 0;
            
            while (Brain.Timer.system() - correctionStart < maxCorrectionTime) {
                double gpsX = GPS.xPosition();
                double gpsY = GPS.yPosition();
                double gpsH = GPS.heading();
                
                // Bad GPS reading - stop and exit
                if (isnan(gpsX) || isnan(gpsY) || isnan(gpsH)) {
                    cout << "  GPS correction: Bad reading, stopping\n";
                    break;
                }
                
                double gpsDist = sqrt(pow(endX - gpsX, 2) + pow(endY - gpsY, 2));
                
                // Debug every 200ms
                int elapsed = Brain.Timer.system() - correctionStart;
                if (elapsed - lastDebug >= 200) {
                    cout << "  GPS correction: dist=" << gpsDist << "cm, elapsed=" << elapsed << "ms\n";
                    lastDebug = elapsed;
                }
                
                // Success - within tolerance
                if (gpsDist < positionTolerance) {
                    cout << "  GPS correction: Target reached! dist=" << gpsDist << "cm\n";
                    break;
                }
                
                // Calculate direction to target
                double dx = endX - gpsX;
                double dy = endY - gpsY;
                double targetAngle = 90.0 - (atan2(dy, dx) * 180.0 / M_PI);
                while (targetAngle < 0) targetAngle += 360;
                while (targetAngle >= 360) targetAngle -= 360;
                
                // Angle error
                double angleErr = targetAngle - gpsH;
                while (angleErr > 180) angleErr -= 360;
                while (angleErr < -180) angleErr += 360;
                
                // Determine if we need to reverse (target is behind us)
                bool shouldReverse = fabs(angleErr) > 90.0;
                if (shouldReverse) {
                    // Flip angle error for reverse driving
                    angleErr = (angleErr > 0) ? angleErr - 180 : angleErr + 180;
                }
                
                // Proportional steering
                float turnCorr = 0.06f * (float)angleErr;
                turnCorr = fmax(-2.0f, fmin(2.0f, turnCorr));
                
                // Speed based on distance - very slow for precision
                float correctionVel;
                if (gpsDist > 10.0f) {
                    correctionVel = 2.0f;
                } else if (gpsDist > 5.0f) {
                    correctionVel = 1.5f;
                } else {
                    correctionVel = 1.2f;  // Minimum to overcome friction
                }
                if (shouldReverse) correctionVel = -correctionVel;
                
                // Apply drive
                float leftVel = correctionVel + turnCorr;
                float rightVel = correctionVel - turnCorr;
                chassis.drive_with_voltage(leftVel, rightVel);
                
                task::sleep(20);  // 50Hz update rate
            }
            chassis.drive_with_voltage(0, 0);
            
            // Final position report
            task::sleep(100);
            finalX = GPS.xPosition();
            finalY = GPS.yPosition();
            finalDist = sqrt(pow(endX - finalX, 2) + pow(endY - finalY, 2));
            cout << "  Final position error: " << finalDist << "cm\n";
            */ // END GPS POSITION CORRECTION
            
            // === GPS-BASED FINAL HEADING CORRECTION ===
            if (endHeading >= 0) {
                cout << "Pure Pursuit: GPS heading correction to " << endHeading << "\n";
                
                // Use GPS to turn precisely
                for (int hCorr = 0; hCorr < 20; hCorr++) {  // Max 20 iterations (~2 sec)
                    task::sleep(50);
                    double gpsH = GPS.heading();
                    if (isnan(gpsH)) break;
                    
                    double headingError = endHeading - gpsH;
                    while (headingError > 180) headingError -= 360;
                    while (headingError < -180) headingError += 360;
                    
                    // If within 2°, heading is good enough
                    if (fabs(headingError) < 2.0) {
                        cout << "  Heading achieved: " << gpsH << " (error: " << headingError << ")\n";
                        break;
                    }
                    
                    // Proportional turn
                    float turnVel = 0.06f * (float)headingError;
                    turnVel = fmax(-4.0f, fmin(4.0f, turnVel));
                    // Minimum voltage to move
                    if (fabs(turnVel) < 1.5f && fabs(turnVel) > 0.1f) {
                        turnVel = (turnVel > 0) ? 1.5f : -1.5f;
                    }
                    
                    chassis.drive_with_voltage(-turnVel, turnVel);  // Match turn_to_angle convention
                }
                chassis.drive_with_voltage(0, 0);
                chassis.drive_stop(brake);  // Brake after heading correction
                
                // Report final heading
                task::sleep(100);
                cout << "  Final GPS heading: " << GPS.heading() << "\n";
            }
            
            chassis.drive_stop(coast);  // Reset to coast for user control
            return true;
        }
        
        // GPS final approach DISABLED - tracking wheel odometry is more consistent
        // Only use for single point paths where we have no waypoints to follow
        if (singlePointPath) {
            // Switch to GPS position for final approach (encoders drift)
            double gpsX = GPS.xPosition();
            double gpsY = GPS.yPosition();
            double gpsH = GPS.heading();
            
            // Use GPS if valid, otherwise fall back to encoder estimate
            double approachX = (!isnan(gpsX)) ? gpsX : robotX;
            double approachY = (!isnan(gpsY)) ? gpsY : robotY;
            double approachH = (!isnan(gpsH)) ? gpsH : robotHeadingNav;
            
            double dx = endX - approachX;
            double dy = endY - approachY;
            double gpsDist = sqrt(dx*dx + dy*dy);
            
            // Calculate target angle in navigation coords
            double targetAngleMath = atan2(dy, dx);
            double targetAngleNav = 90.0 - (targetAngleMath * 180.0 / M_PI);
            while (targetAngleNav < 0) targetAngleNav += 360.0;
            while (targetAngleNav >= 360) targetAngleNav -= 360.0;
            
            double angleError = targetAngleNav - approachH;
            while (angleError > 180) angleError -= 360;
            while (angleError < -180) angleError += 360;
            
            // Debug
            if (iteration % 50 == 0) {
                cout << "PP Final(GPS): dist=" << gpsDist << " err=" << angleError << "\n";
            }
            
            // Check if we're actually at target using GPS (tighter tolerance)
            if (gpsDist < endTolerance * 0.5f) {
                // We're really close per GPS - check encoder estimate agrees
                if (distToEnd < endTolerance * 1.5f) {
                    chassis.drive_with_voltage(0, 0);
                    chassis.drive_stop(brake);  // Brake to stop precisely
                    cout << "Pure Pursuit: Target reached! GPS dist=" << gpsDist << "cm\n";
                    
                    // Do heading correction inline if needed
                    if (endHeading >= 0) {
                        cout << "Pure Pursuit: GPS heading correction to " << endHeading << "\n";
                        for (int hCorr = 0; hCorr < 20; hCorr++) {
                            task::sleep(50);
                            double hGps = GPS.heading();
                            if (isnan(hGps)) break;
                            
                            double hErr = endHeading - hGps;
                            while (hErr > 180) hErr -= 360;
                            while (hErr < -180) hErr += 360;
                            
                            if (fabs(hErr) < 2.0) {
                                cout << "  Heading achieved: " << hGps << "\n";
                                break;
                            }
                            
                            float tVel = 0.06f * (float)hErr;
                            tVel = fmax(-4.0f, fmin(4.0f, tVel));
                            if (fabs(tVel) < 1.5f && fabs(tVel) > 0.1f) {
                                tVel = (tVel > 0) ? 1.5f : -1.5f;
                            }
                            chassis.drive_with_voltage(-tVel, tVel);  // Match turn_to_angle convention
                        }
                        chassis.drive_with_voltage(0, 0);
                        chassis.drive_stop(brake);  // Brake after heading correction
                        cout << "  Final GPS heading: " << GPS.heading() << "\n";
                    }
                    chassis.drive_stop(coast);  // Reset to coast for user control
                    return true;
                }
            }
            
            float absErr = fabs((float)angleError);
            float turnGain = 0.08f;  // Reduced to fix over-turning
            float turnOutput = turnGain * (float)angleError;
            turnOutput = fmax(-baseVelocity, fmin(baseVelocity, turnOutput));
            
            // Progressive slowdown based on GPS distance - very slow when close
            float speedFactor;
            if (gpsDist < 5.0f) {
                speedFactor = 0.2f;  // Crawl for final 5cm
            } else if (gpsDist < 10.0f) {
                speedFactor = 0.3f;  // Very slow for 5-10cm
            } else if (gpsDist < 15.0f) {
                speedFactor = 0.4f;  // Slow for 10-15cm
            } else if (gpsDist < 25.0f) {
                speedFactor = 0.6f;  // Medium for 15-25cm
            } else {
                speedFactor = 0.8f;  // Approaching
            }
            
            // Also slow down for sharp turns
            if (absErr > 30.0f) speedFactor *= 0.4f;
            else if (absErr > 15.0f) speedFactor *= 0.6f;
            
            float driveVel = baseVelocity * speedFactor;
            if (driveVel < minVelocity) driveVel = minVelocity;
            
            // Match drive_distance convention: left = drive - heading, right = drive + heading
            float leftVel = driveVel - turnOutput;
            float rightVel = driveVel + turnOutput;
            
            chassis.drive_with_voltage(leftVel, rightVel);
            task::sleep(10);
            continue;
        }
        
        // Calculate progress through path (0.0 to 1.0)
        float progress = (totalPathDist > 0) ? (float)(1.0 - distToEnd / totalPathDist) : 1.0f;
        progress = fmax(0.0f, fmin(1.0f, progress));  // Clamp to [0, 1]
        
        // Determine if we're in "fast" phase (only if path is long enough and < 70% progress)
        bool inFastPhase = !useSlowOnly && (progress < slowdownPoint);
        
        // Select velocity based on progress: fast for first 70%, slow for last 30%
        float currentVelocity = inFastPhase ? fastVelocity : baseVelocity;
        
        // Dynamic lookahead: larger when fast (see turns earlier), smaller when close to target
        float dynamicLookahead;
        if (distToEnd < 20.0) {
            dynamicLookahead = lookaheadDist * 0.5f;  // Half lookahead for final approach
        } else if (distToEnd < 40.0) {
            dynamicLookahead = lookaheadDist * 0.7f;  // Tighter tracking in approach
        } else if (inFastPhase) {
            dynamicLookahead = lookaheadDist * 1.3f;  // Slightly larger when fast (reduced from 1.5)
        } else {
            dynamicLookahead = lookaheadDist;
        }
        
        // Find lookahead point with dynamic distance
        auto lookahead = findLookaheadPoint(path, robotX, robotY, dynamicLookahead, nearestIdx);
        double lookaheadX = lookahead.first;
        double lookaheadY = lookahead.second;
        
        // Calculate angle to lookahead point (in math coords: 0=east, CCW positive)
        double dx = lookaheadX - robotX;
        double dy = lookaheadY - robotY;
        double targetAngleMath = atan2(dy, dx);  // radians, math convention
        
        // Convert target angle to navigation (0=north, CW positive) for comparison with IMU
        double targetAngleNav = 90.0 - (targetAngleMath * 180.0 / M_PI);
        // Normalize to [0, 360)
        while (targetAngleNav < 0) targetAngleNav += 360.0;
        while (targetAngleNav >= 360) targetAngleNav -= 360.0;
        
        // Calculate angle error in navigation coords
        double angleError = targetAngleNav - robotHeadingNav;
        // Normalize to [-180, 180]
        while (angleError > 180) angleError -= 360;
        while (angleError < -180) angleError += 360;
        
        // Debug output every 50 iterations
        if (iteration % 50 == 0) {
            cout << "PP: pos=(" << robotX << "," << robotY << ") h=" << robotHeadingNav 
                 << " progress=" << (int)(progress*100) << "% vel=" << currentVelocity << "V"
                 << " LA=" << dynamicLookahead << " err=" << angleError << "\n";
        }
        
        // Steering with smoothing to reduce oscillations
        float absError = fabs((float)angleError);
        float rawTurnOutput = 0.0f;
        
        // Only apply small deadband to reduce wobble on straight sections
        if (absError > steeringDeadband) {
            // Scale turn gain with velocity and distance to target
            // Lower gain when close to target to reduce end oscillations
            // Reduced ~40% from original to fix over-steering
            float turnGain;
            if (distToEnd < 15.0) {
                turnGain = 0.04f;  // Very gentle near target
            } else if (distToEnd < 30.0) {
                turnGain = 0.05f;  // Gentle in approach
            } else if (inFastPhase) {
                turnGain = 0.07f;  // Moderate when fast
            } else {
                turnGain = 0.06f;  // Moderate when slow
            }
            rawTurnOutput = turnGain * (float)angleError;
            
            // Limit max turn based on velocity
            float maxTurn = currentVelocity * 0.5f;
            rawTurnOutput = fmax(-maxTurn, fmin(maxTurn, rawTurnOutput));
        }
        
        // Apply exponential smoothing to reduce jerky steering
        smoothedTurnOutput = turnSmoothingFactor * rawTurnOutput + (1.0f - turnSmoothingFactor) * smoothedTurnOutput;
        float turnOutput = smoothedTurnOutput;
        
        // Slow down more when turning sharply AND when close to target
        float speedFactor;
        if (distToEnd < 10.0) {
            speedFactor = 0.15f;  // Crawl speed for final 10cm
        } else if (distToEnd < 20.0) {
            speedFactor = 0.25f;  // Very slow for 10-20cm
        } else if (distToEnd < 40.0) {
            speedFactor = 0.4f;   // Slow for 20-40cm
        } else if (absError > 30.0f) {
            speedFactor = 0.25f;  // Slow for sharp turns
        } else if (absError > 15.0f) {
            speedFactor = 0.5f;   // Medium for moderate turns
        } else {
            speedFactor = 0.85f;  // Normal cruise
        }
        float driveVel = currentVelocity * speedFactor;
        
        // Apply differential steering
        // Match drive_distance convention: left = drive - heading, right = drive + heading
        float leftVel = driveVel - turnOutput;
        float rightVel = driveVel + turnOutput;
        
        // Minimum voltage to overcome friction
        if (fabs(leftVel) < minVelocity && fabs(leftVel) > 0.1f) {
            leftVel = (leftVel > 0) ? minVelocity : -minVelocity;
        }
        if (fabs(rightVel) < minVelocity && fabs(rightVel) > 0.1f) {
            rightVel = (rightVel > 0) ? minVelocity : -minVelocity;
        }
        
        // Clamp to max voltage
        leftVel = fmax(-12.0f, fmin(12.0f, leftVel));
        rightVel = fmax(-12.0f, fmin(12.0f, rightVel));
        
        chassis.drive_with_voltage(leftVel, rightVel);
        task::sleep(10);
    }
    
    // Timeout
    chassis.drive_with_voltage(0, 0);
    chassis.drive_stop(brake);
    cout << "Pure Pursuit: Timeout\n";
    chassis.drive_stop(coast);  // Reset to coast for user control
    return false;
}

// ============================================================================
// OLD PURE PURSUIT PATH FOLLOWER (simpler, uses Localizer for position)
// Ported from WallE - Old Pure Pursuit with Localizer integration
// ============================================================================

bool purePursuitFollowPathOld(const std::vector<std::pair<double,double>>& pathIn,
                              float baseVelocity,
                              float lookaheadDist,
                              float endTolerance,
                              float endHeading,
                              bool useGPS)
{
    if (pathIn.empty()) {
        return false;
    }
    
    std::vector<std::pair<double,double>> path = pathIn;
    
    // Robot parameters (OLD values)
    const int maxIterations = 3000;  // ~30 seconds at 10ms loop
    const float minVelocity = 2.0f;
    const float steeringDeadband = 1.5f;  // Small deadband to reduce wobble
    
    // Speed ramping: fast for first 70%, slow for last 30%
    const float fastVelocity = 8.0f;    // Cruise speed (V)
    const float slowdownPoint = 0.7f;   // Start slowing at 70% progress
    const float minDistForFast = 60.0f * 2.54f;  // 60 inches in cm (~152cm)
    
    // Calculate total path distance for progress tracking
    double totalPathDist = 0.0;
    for (size_t i = 1; i < path.size(); i++) {
        double dx = path[i].first - path[i-1].first;
        double dy = path[i].second - path[i-1].second;
        totalPathDist += sqrt(dx*dx + dy*dy);
    }
    
    // For short paths, skip fast phase entirely
    bool useSlowOnly = (totalPathDist < minDistForFast);
    if (useSlowOnly) {
        cout << "PP Old: Short path (" << totalPathDist << "cm), using slow speed only\n";
    }
    
    // Sensor fusion parameters (OLD)
    const int gpsUpdateInterval = 10;   // Blend GPS every N iterations (100ms)
    const float gpsBlendFactor = 0.3f;  // 30% GPS, 70% dead reckoning
    
    // Center structure collision recovery parameters
    // Center structure is 54x54cm at (0,0) - detect when within ~45cm of center
    const float centerStructureRadius = 45.0f;    // cm - trigger when this close to (0,0)
    const float centerStructureClearance = 75.0f; // cm - must escape to this distance
    const float escapeVelocity = 12.0f;  // Voltage for escape maneuver
    const int maxEscapeTime = 3000;     // ms max escape time
    
    // Get initial position from Localizer (NEW - was raw GPS)
    Pose startPose = localizer.getPose();
    double robotX = startPose.x_cm;
    double robotY = startPose.y_cm;
    double initHeading = startPose.heading_deg;
    
    // Sync chassis IMU heading with Localizer heading at start
    chassis.set_heading(initHeading);
    task::sleep(20);  // Allow IMU to sync
    
    cout << "PP Old Start (Localizer): X=" << robotX << " Y=" << robotY << " H=" << initHeading << "\n";
    
    // Store initial encoder positions (inches)
    float lastLeftPos = chassis.get_left_position_in();
    float lastRightPos = chassis.get_right_position_in();
    
    // End point is the LAST point in path
    double endX = path.back().first;
    double endY = path.back().second;
    
    int iteration = 0;
    size_t nearestIdx = 0;
    bool singlePointPath = (path.size() == 1);
    
    // Stuck detection using Localizer position
    int stuckCounter = 0;
    const int stuckThreshold = 100;      // ~1 second of no movement
    const int stuckCheckInterval = 10;   // Check every 100ms
    double lastPosX = robotX;
    double lastPosY = robotY;
    const double stuckDistThreshold = 0.5;  // Must move at least 0.5cm per check
    
    while (iteration++ < maxIterations) {
        // Get heading from IMU (faster and smoother)
        double robotHeadingNav = chassis.get_absolute_heading();
        
        // Get current encoder positions
        float leftPos = chassis.get_left_position_in();
        float rightPos = chassis.get_right_position_in();
        
        // Calculate distance traveled since last iteration
        float deltaLeft = leftPos - lastLeftPos;
        float deltaRight = rightPos - lastRightPos;
        float distanceTraveled_in = (deltaLeft + deltaRight) / 2.0f;
        float distanceTraveled_cm = distanceTraveled_in * 2.54f;
        
        // Stuck detection using Localizer position (NEW - was raw GPS)
        if (iteration % stuckCheckInterval == 0) {
            Pose currentPose = localizer.getPose();
            double currentPosX = currentPose.x_cm;
            double currentPosY = currentPose.y_cm;
            
            double posDelta = sqrt(pow(currentPosX - lastPosX, 2) + pow(currentPosY - lastPosY, 2));
            
            if (posDelta < stuckDistThreshold) {
                stuckCounter++;
                if (stuckCounter >= (stuckThreshold / stuckCheckInterval)) {
                    chassis.drive_with_voltage(0, 0);
                    cout << "PP Old: STUCK - no movement detected\n";
                    Controller.Screen.clearScreen();
                    Controller.Screen.print("STUCK!");
                    return false;
                }
            } else {
                stuckCounter = 0;  // Reset if moving
            }
            
            lastPosX = currentPosX;
            lastPosY = currentPosY;
        }
        
        // Update position using dead reckoning (encoder + IMU)
        double headingRad = (90.0 - robotHeadingNav) * M_PI / 180.0;
        robotX += distanceTraveled_cm * cos(headingRad);
        robotY += distanceTraveled_cm * sin(headingRad);
        
        // Detect if robot is turning
        float turnRate = fabs(deltaLeft - deltaRight);
        bool isTurning = (turnRate > 0.05f);
        
        // Store for next iteration
        lastLeftPos = leftPos;
        lastRightPos = rightPos;
        
        // Check distance to end for adaptive blending
        double distToEnd = sqrt(pow(endX - robotX, 2) + pow(endY - robotY, 2));
        
        // Sensor fusion: periodically blend in Localizer to correct drift (NEW - was raw GPS)
        // BUT only when going straight - lag hurts turning accuracy
        if (useGPS && (iteration % gpsUpdateInterval == 0) && !isTurning) {
            Pose fusePose = localizer.getPose();
            double locX = fusePose.x_cm;
            double locY = fusePose.y_cm;
            
            // Adaptive blend factor: trust Localizer more when close to target
            float adaptiveBlend = gpsBlendFactor;
            if (distToEnd < 20.0) {
                adaptiveBlend = 0.8f;  // 80% Localizer when within 20cm
            } else if (distToEnd < 40.0) {
                adaptiveBlend = 0.65f; // 65% Localizer when within 40cm
            } else if (distToEnd < 70.0) {
                adaptiveBlend = 0.5f;  // 50% Localizer when within 70cm
            }
            
            // Weighted average: new_pos = blend*localizer + (1-blend)*dead_reckoning
            robotX = adaptiveBlend * locX + (1.0 - adaptiveBlend) * robotX;
            robotY = adaptiveBlend * locY + (1.0 - adaptiveBlend) * robotY;
        }
        
        // === CENTER STRUCTURE COLLISION RECOVERY ===
        // Check distance from field origin (0,0) using current fused position
        double distFromCenter = sqrt(robotX * robotX + robotY * robotY);
        
        if (distFromCenter < centerStructureRadius) {
            cout << "PP Old: CENTER STRUCTURE COLLISION! dist=" << distFromCenter << "cm\n";
            Controller.rumble("-");
            Controller.Screen.clearScreen();
            Controller.Screen.print("Collision! Escaping...");
            
            // Determine escape direction based on heading
            // Robot heading: 0 = +Y (north), 90 = +X (east), etc.
            double headingRadNav = robotHeadingNav * M_PI / 180.0;
            // Forward direction vector
            double fwdX = sin(headingRadNav);  // +X component when heading east (90)
            double fwdY = cos(headingRadNav);  // +Y component when heading north (0)
            
            // Dot product of forward vector with position vector (from center)
            // Positive = forward points away from center (drive forward to escape)
            // Negative = forward points toward center (drive backward to escape)
            double dotProduct = fwdX * robotX + fwdY * robotY;
            
            bool driveForward = (dotProduct > 0);
            float escapeVel = driveForward ? escapeVelocity : -escapeVelocity;
            
            cout << "  Heading: " << robotHeadingNav << ", dot: " << dotProduct 
                 << ", escaping " << (driveForward ? "FORWARD" : "BACKWARD") << "\n";
            
            // Drive until clear of center structure
            int escapeStart = Brain.Timer.system();
            while (Brain.Timer.system() - escapeStart < maxEscapeTime) {
                chassis.drive_with_voltage(escapeVel, escapeVel);
                task::sleep(20);
                
                // Update position from Localizer
                Pose escapePose = localizer.getPose();
                robotX = escapePose.x_cm;
                robotY = escapePose.y_cm;
                
                // Check if we're clear (outside clearance zone)
                double escapeDistFromCenter = sqrt(robotX * robotX + robotY * robotY);
                if (escapeDistFromCenter > centerStructureClearance) {
                    cout << "  Escaped! Now at (" << robotX << "," << robotY << "), dist=" << escapeDistFromCenter << "cm\n";
                    break;
                }
            }
            
            chassis.drive_with_voltage(0, 0);
            Controller.Screen.clearScreen();
            Controller.Screen.print("Escaped - replanning");
            task::sleep(200);
            
            // Regenerate path from current position to original goal
            Pose escapedPose = localizer.getPose();
            robotX = escapedPose.x_cm;
            robotY = escapedPose.y_cm;
            
            FieldMap replanMap;
            replanMap.populateStandardField();
            const double replan_robot_radius_cm = 13.5 * 2.54 * 0.5;
            const double replan_grid_res = 60.96 / 2;
            
            std::vector<astar::Point> newPath = astar::findPath(
                replanMap, robotX, robotY, endX, endY,
                replan_grid_res, replan_robot_radius_cm, 0.0);
            
            if (!newPath.empty()) {
                path.clear();
                for (const auto& pt : newPath) {
                    path.push_back({pt.first, pt.second});
                }
                nearestIdx = 0;
                singlePointPath = (path.size() == 1);
                
                cout << "PP Old: Replanned from (" << robotX << "," << robotY 
                     << ") to (" << endX << "," << endY << "), " << path.size() << " waypoints\n";
                Controller.Screen.clearScreen();
                Controller.Screen.print("Replanned - resuming");
            } else {
                cout << "PP Old: Replan failed! No path from (" << robotX << "," << robotY << ")\n";
                Controller.Screen.clearScreen();
                Controller.Screen.print("Replan failed!");
                chassis.drive_stop(coast);
                return false;
            }
            
            distToEnd = sqrt(pow(endX - robotX, 2) + pow(endY - robotY, 2));
        }
        
        if (distToEnd < endTolerance) {
            chassis.drive_with_voltage(0, 0);
            
            // Report final position from Localizer
            task::sleep(100);
            Pose finalPose = localizer.getPose();
            double finalDist = sqrt(pow(endX - finalPose.x_cm, 2) + pow(endY - finalPose.y_cm, 2));
            cout << "PP Old: Arrived! Localizer dist to target: " << finalDist << "cm\n";
            
            // === HEADING CORRECTION ===
            if (endHeading >= 0) {
                cout << "PP Old: Heading correction to " << endHeading << "\n";
                
                for (int hCorr = 0; hCorr < 20; hCorr++) {
                    task::sleep(50);
                    Pose hPose = localizer.getPose();
                    double currentH = hPose.heading_deg;
                    
                    double headingError = endHeading - currentH;
                    while (headingError > 180) headingError -= 360;
                    while (headingError < -180) headingError += 360;
                    
                    if (fabs(headingError) < 2.0) {
                        cout << "  Heading achieved: " << currentH << "\n";
                        break;
                    }
                    
                    float turnVel = 0.06f * (float)headingError;
                    turnVel = fmax(-4.0f, fmin(4.0f, turnVel));
                    if (fabs(turnVel) < 1.5f && fabs(turnVel) > 0.1f) {
                        turnVel = (turnVel > 0) ? 1.5f : -1.5f;
                    }
                    
                    chassis.drive_with_voltage(-turnVel, turnVel);  // Match turn_to_angle convention
                }
                chassis.drive_with_voltage(0, 0);
                
                task::sleep(100);
                cout << "  Final heading: " << localizer.getPose().heading_deg << "\n";
            }
            
            return true;
        }
        
        // Single point path OR final approach - use Localizer directly
        if (singlePointPath || (nearestIdx >= path.size() - 1 && distToEnd < lookaheadDist * 1.5)) {
            Pose approachPose = localizer.getPose();
            double approachX = approachPose.x_cm;
            double approachY = approachPose.y_cm;
            double approachH = approachPose.heading_deg;
            
            double dx = endX - approachX;
            double dy = endY - approachY;
            double locDist = sqrt(dx*dx + dy*dy);
            
            double targetAngleMath = atan2(dy, dx);
            double targetAngleNav = 90.0 - (targetAngleMath * 180.0 / M_PI);
            while (targetAngleNav < 0) targetAngleNav += 360.0;
            while (targetAngleNav >= 360) targetAngleNav -= 360.0;
            
            double angleError = targetAngleNav - approachH;
            while (angleError > 180) angleError -= 360;
            while (angleError < -180) angleError += 360;
            
            if (iteration % 50 == 0) {
                cout << "PP Old Final: dist=" << locDist << " err=" << angleError << "\n";
            }
            
            // Check if at target
            if (locDist < endTolerance * 0.5f) {
                if (distToEnd < endTolerance * 1.5f) {
                    chassis.drive_with_voltage(0, 0);
                    cout << "PP Old: Target reached! dist=" << locDist << "cm\n";
                    
                    if (endHeading >= 0) {
                        cout << "PP Old: Heading correction to " << endHeading << "\n";
                        for (int hCorr = 0; hCorr < 20; hCorr++) {
                            task::sleep(50);
                            double hNow = localizer.getPose().heading_deg;
                            
                            double hErr = endHeading - hNow;
                            while (hErr > 180) hErr -= 360;
                            while (hErr < -180) hErr += 360;
                            
                            if (fabs(hErr) < 2.0) {
                                cout << "  Heading achieved: " << hNow << "\n";
                                break;
                            }
                            
                            float tVel = 0.06f * (float)hErr;
                            tVel = fmax(-4.0f, fmin(4.0f, tVel));
                            if (fabs(tVel) < 1.5f && fabs(tVel) > 0.1f) {
                                tVel = (tVel > 0) ? 1.5f : -1.5f;
                            }
                            chassis.drive_with_voltage(-tVel, tVel);  // Match turn_to_angle convention
                        }
                        chassis.drive_with_voltage(0, 0);
                        cout << "  Final heading: " << localizer.getPose().heading_deg << "\n";
                    }
                    return true;
                }
            }
            
            float absErr = fabs((float)angleError);
            float turnGain = 0.12f;  // OLD aggressive gain
            float turnOutput = turnGain * (float)angleError;
            turnOutput = fmax(-baseVelocity, fmin(baseVelocity, turnOutput));
            
            // Progressive slowdown based on distance
            float speedFactor;
            if (locDist < 5.0f) {
                speedFactor = 0.2f;
            } else if (locDist < 10.0f) {
                speedFactor = 0.3f;
            } else if (locDist < 15.0f) {
                speedFactor = 0.4f;
            } else if (locDist < 25.0f) {
                speedFactor = 0.6f;
            } else {
                speedFactor = 0.8f;
            }
            
            if (absErr > 30.0f) speedFactor *= 0.4f;
            else if (absErr > 15.0f) speedFactor *= 0.6f;
            
            float driveVel = baseVelocity * speedFactor;
            if (driveVel < minVelocity) driveVel = minVelocity;
            
            // Match drive_distance convention: left = drive - heading, right = drive + heading
            float leftVel = driveVel - turnOutput;
            float rightVel = driveVel + turnOutput;
            
            chassis.drive_with_voltage(leftVel, rightVel);
            task::sleep(10);
            continue;
        }
        
        // Calculate progress through path (0.0 to 1.0)
        float progress = (totalPathDist > 0) ? (float)(1.0 - distToEnd / totalPathDist) : 1.0f;
        progress = fmax(0.0f, fmin(1.0f, progress));
        
        // Fast phase vs slow phase
        bool inFastPhase = !useSlowOnly && (progress < slowdownPoint);
        float currentVelocity = inFastPhase ? fastVelocity : baseVelocity;
        
        // Dynamic lookahead (OLD: 1.5x when fast, 1.0x when slow)
        float dynamicLookahead = inFastPhase ? lookaheadDist * 1.5f : lookaheadDist;
        
        // Find lookahead point
        auto lookahead = findLookaheadPoint(path, robotX, robotY, dynamicLookahead, nearestIdx);
        double lookaheadX = lookahead.first;
        double lookaheadY = lookahead.second;
        
        // Calculate angle to lookahead point
        double dx = lookaheadX - robotX;
        double dy = lookaheadY - robotY;
        double targetAngleMath = atan2(dy, dx);
        
        double targetAngleNav = 90.0 - (targetAngleMath * 180.0 / M_PI);
        while (targetAngleNav < 0) targetAngleNav += 360.0;
        while (targetAngleNav >= 360) targetAngleNav -= 360.0;
        
        double angleError = targetAngleNav - robotHeadingNav;
        while (angleError > 180) angleError -= 360;
        while (angleError < -180) angleError += 360;
        
        // Debug output
        if (iteration % 50 == 0) {
            cout << "PP Old: pos=(" << robotX << "," << robotY << ") h=" << robotHeadingNav 
                 << " progress=" << (int)(progress*100) << "% vel=" << currentVelocity << "V"
                 << " LA=" << dynamicLookahead << " err=" << angleError << "\n";
        }
        
        // Steering - OLD aggressive gains
        float absError = fabs((float)angleError);
        float turnOutput = 0.0f;
        
        if (absError > steeringDeadband) {
            // OLD turn gains: 0.15 when fast, 0.12 when slow
            float turnGain = inFastPhase ? 0.15f : 0.12f;
            turnOutput = turnGain * (float)angleError;
            
            // Allow stronger turn output
            float maxTurn = currentVelocity * 1.0f;
            turnOutput = fmax(-maxTurn, fmin(maxTurn, turnOutput));
        }
        
        // Slow down for sharp turns (OLD: 3 levels)
        float speedFactor = (absError > 30.0f) ? 0.25f : (absError > 15.0f) ? 0.5f : 0.8f;
        float driveVel = currentVelocity * speedFactor;
        
        // Apply differential steering
        // Match drive_distance convention: left = drive - heading, right = drive + heading
        float leftVel = driveVel - turnOutput;
        float rightVel = driveVel + turnOutput;
        
        // Minimum voltage to overcome friction
        if (fabs(leftVel) < minVelocity && fabs(leftVel) > 0.1f) {
            leftVel = (leftVel > 0) ? minVelocity : -minVelocity;
        }
        if (fabs(rightVel) < minVelocity && fabs(rightVel) > 0.1f) {
            rightVel = (rightVel > 0) ? minVelocity : -minVelocity;
        }
        
        // Clamp to max voltage
        leftVel = fmax(-12.0f, fmin(12.0f, leftVel));
        rightVel = fmax(-12.0f, fmin(12.0f, rightVel));
        
        chassis.drive_with_voltage(leftVel, rightVel);
        task::sleep(10);
    }
    
    // Timeout
    chassis.drive_with_voltage(0, 0);
    cout << "PP Old: Timeout\n";
    return false;
}

// Test function for OLD pure pursuit
void testPurePursuitOld() {
    Controller.Screen.clearScreen();
    Controller.Screen.print("PP Old Test");
    wait(200, msec);
    
    // Initialize target coordinates
    double target_x = 0.0;
    double target_y = 0.0;
    
    // Allow user to adjust target coordinates
    bool coordinatesLocked = false;
    
    while (!coordinatesLocked) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("X: %.1f", target_x);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Y: %.1f", target_y);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("A=Lock B=Cancel");
        
        if (Controller.ButtonUp.pressing()) {
            target_x += 10.0;
        }
        if (Controller.ButtonDown.pressing()) {
            target_x -= 10.0;
        }
        if (Controller.ButtonLeft.pressing()) {
            target_y -= 10.0;
        }
        if (Controller.ButtonRight.pressing()) {
            target_y += 10.0;
        }
        if (Controller.ButtonA.pressing()) {
            waitUntil(!Controller.ButtonA.pressing());
            coordinatesLocked = true;
            wait(200, msec);
        }
        if (Controller.ButtonB.pressing()) {
            waitUntil(!Controller.ButtonB.pressing());
            return;
        }
        
        wait(20, msec);
    }
    
    // Get current position from Localizer
    Controller.Screen.clearScreen();
    Controller.Screen.print("Getting position...");
    wait(300, msec);
    
    Pose startPose = localizer.getPose();
    double start_x = startPose.x_cm;
    double start_y = startPose.y_cm;
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("From: %.0f,%.0f", start_x, start_y);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("To: %.0f,%.0f", target_x, target_y);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Planning...");
    wait(500, msec);
    
    cout << "PP OLD: Start (" << start_x << ", " << start_y << ") -> Target (" << target_x << ", " << target_y << ")\n";
    
    // Create field map and plan path
    FieldMap fieldMap;
    fieldMap.populateStandardField();
    
    const double robot_width_in = 13.5;
    const double robot_radius_cm = robot_width_in * 2.54 * 0.5;
    const double safety_margin_cm = 2.0;
    const double grid_resolution_cm = 15.0;
    
    std::vector<astar::Point> path = astar::findPath(
        fieldMap,
        start_x, start_y,
        target_x, target_y,
        grid_resolution_cm,
        robot_radius_cm,
        safety_margin_cm
    );
    
    if (path.empty()) {
        cout << "A* failed - no path found!\n";
        Controller.Screen.clearScreen();
        Controller.Screen.print("No path found!");
        wait(1000, msec);
        return;
    }
    
    cout << "A* found path with " << path.size() << " waypoints\n";
    
    // Convert astar::Point path to std::pair path for purePursuitFollowPathOld
    // Skip first waypoint, keep intermediate, replace last with actual target
    std::vector<std::pair<double,double>> adjustedPath;
    
    if (path.size() <= 2) {
        adjustedPath.push_back({target_x, target_y});
    } else {
        for (size_t i = 1; i < path.size(); i++) {
            adjustedPath.push_back({path[i].first, path[i].second});
        }
        adjustedPath.back() = {target_x, target_y};
    }
    
    cout << "===== PP OLD PATH =====\n";
    for (size_t i = 0; i < adjustedPath.size(); i++) {
        cout << i << ": (" << adjustedPath[i].first << ", " << adjustedPath[i].second << ")\n";
    }
    
    Controller.Screen.clearScreen();
    Controller.Screen.print("Following path...");
    
    // Follow with OLD pure pursuit (lookahead=25cm, tolerance=3cm - OLD values)
    bool success = purePursuitFollowPathOld(adjustedPath, 4.0f, 25.0f, 3.0f, -1.0f, true);
    
    // Stop and report
    chassis.drive_with_voltage(0, 0);
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
    wait(300, msec);
    
    // Final position from Localizer
    Pose finalPose = localizer.getPose();
    double final_x = finalPose.x_cm;
    double final_y = finalPose.y_cm;
    double final_h = finalPose.heading_deg;
    double err = sqrt(pow(final_x - target_x, 2) + pow(final_y - target_y, 2));
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print(success ? "Success!" : "Timeout/Error");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Pos: %.1f,%.1f H:%.0f", final_x, final_y, final_h);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Error: %.1f cm", err);
    
    cout << "PP Old " << (success ? "SUCCESS" : "FAILED") << "\n";
    cout << "Final Pose: (" << final_x << "," << final_y << ") H=" << final_h << " Error: " << err << " cm\n";
    
    wait(3000, msec);
}

// Test function: A* planning + pure pursuit following
void testPurePursuit() {
    Controller.Screen.clearScreen();
    Controller.Screen.print("Pure Pursuit Test");
    wait(200, msec);
    
    // Initialize target coordinates
    double target_x = 0.0;
    double target_y = 0.0;
    
    // Allow user to adjust target coordinates
    bool coordinatesLocked = false;
    
    while (!coordinatesLocked) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("X: %.1f", target_x);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Y: %.1f", target_y);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("A=Lock B=Cancel");
        
        if (Controller.ButtonUp.pressing()) {
            target_x += 10.0;
        }
        if (Controller.ButtonDown.pressing()) {
            target_x -= 10.0;
        }
        if (Controller.ButtonLeft.pressing()) {
            target_y -= 10.0;
        }
        if (Controller.ButtonRight.pressing()) {
            target_y += 10.0;
        }
        if (Controller.ButtonA.pressing()) {
            waitUntil(!Controller.ButtonA.pressing());
            coordinatesLocked = true;
            wait(200, msec);
        }
        if (Controller.ButtonB.pressing()) {
            waitUntil(!Controller.ButtonB.pressing());
            return;
        }
        
        wait(20, msec);
    }
    
    // Get current position from Localizer
    Controller.Screen.clearScreen();
    Controller.Screen.print("Getting position...");
    wait(300, msec);
    
    Pose currPose = localizer.getPose();
    double curr_x = currPose.x_cm;
    double curr_y = currPose.y_cm;
    double curr_h = currPose.heading_deg;
    
    // Create field map and plan path
    FieldMap fieldMap;
    fieldMap.populateStandardField();
    
    const double robot_width_in = 13.5;
    const double robot_radius_cm = robot_width_in * 2.54 * 0.5;
    const double safety_margin_cm = 0.0;
    const double grid_resolution_cm = 10.0;
    
    // If robot is behind a long goal, escape to nearest corner first
    bool behindTopGoal = (curr_x >= -61.0 && curr_x <= 61.0 && curr_y >= 130.0 && curr_y <= 182.88);
    bool behindBottomGoal = (curr_x >= -61.0 && curr_x <= 61.0 && curr_y <= -130.0 && curr_y >= -182.88);
    
    if (behindTopGoal || behindBottomGoal) {
        Controller.rumble("---...---");
        Controller.Screen.clearScreen();
        Controller.Screen.print("BEHIND GOAL ZONE!");
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Pos: %.0f,%.0f", curr_x, curr_y);
        wait(1000, msec);
        double escapePoints[4][2] = {{110, 155}, {-110, 155}, {110, -155}, {-110, -155}};
        
        int closest = 0;
        double closestDist = 1e9;
        for (int i = 0; i < 4; i++) {
            double dx = curr_x - escapePoints[i][0];
            double dy = curr_y - escapePoints[i][1];
            double d = sqrt(dx*dx + dy*dy);
            if (d < closestDist) { closestDist = d; closest = i; }
        }
        
        double esc_x = escapePoints[closest][0];
        double esc_y = escapePoints[closest][1];
        
        Controller.Screen.clearScreen();
        Controller.Screen.print("Escaping to %.0f,%.0f", esc_x, esc_y);
        cout << "Behind long goal - simple escape to (" << esc_x << "," << esc_y << ")\n";
        wait(300, msec);
        
        // Re-read current position and heading fresh
        Pose freshPose = localizer.getPose();
        curr_x = freshPose.x_cm;
        curr_y = freshPose.y_cm;
        curr_h = freshPose.heading_deg;
        
        // Sync chassis heading with localizer so turn_to_angle works correctly
        chassis.set_heading(curr_h);
        task::sleep(100);
        
        // Simple point-to-point: turn then drive straight
        double dx = esc_x - curr_x;
        double dy = esc_y - curr_y;
        double dist_cm = sqrt(dx * dx + dy * dy);
        
        // Target angle in nav convention (0=+Y, 90=+X)
        double targetAngle = 90.0 - (atan2(dy, dx) * 180.0 / M_PI);
        while (targetAngle < 0) targetAngle += 360;
        while (targetAngle >= 360) targetAngle -= 360;
        
        // Check if driving backwards is a smaller turn
        double fwdError = targetAngle - curr_h;
        while (fwdError > 180) fwdError -= 360;
        while (fwdError < -180) fwdError += 360;
        
        bool driveReverse = (fabs(fwdError) > 90.0);
        double turnAngle = targetAngle;
        if (driveReverse) {
            turnAngle = fmod(targetAngle + 180.0, 360.0);
        }
        
        float dist_in = (float)(dist_cm / 2.54);
        if (driveReverse) dist_in = -dist_in;
        
        cout << "  Curr H: " << curr_h << " Turn to " << turnAngle << " then drive " << dist_in << " in\n";
        
        intakeBalls();
        chassis.turn_to_angle(turnAngle, 6);
        task::sleep(200);
        chassis.drive_distance(dist_in, turnAngle, 6, 4);
        chassis.drive_with_voltage(0, 0);
        task::sleep(200);
        
        Pose escPose = localizer.getPose();
        curr_x = escPose.x_cm;
        curr_y = escPose.y_cm;
        curr_h = escPose.heading_deg;
        cout << "Escaped to (" << curr_x << "," << curr_y << ") H:" << curr_h << "\n";
        Controller.Screen.clearScreen();
        Controller.Screen.print("Escaped: %.0f,%.0f", curr_x, curr_y);
    }
    
    // === WALL-ADJACENT BALL APPROACH ===
    // If target is within 20cm of a field wall, use a staging point + straight drive-in
    const double wallThreshold = 20.0;
    const double fieldEdge = 182.88;
    const double stagingDist = 30.0; // cm inward from target for staging point
    
    bool nearRightWall = (target_x > fieldEdge - wallThreshold);
    bool nearLeftWall  = (target_x < -fieldEdge + wallThreshold);
    bool nearTopWall   = (target_y > fieldEdge - wallThreshold);
    bool nearBottomWall= (target_y < -fieldEdge + wallThreshold);
    bool nearWall = nearRightWall || nearLeftWall || nearTopWall || nearBottomWall;
    
    if (nearWall) {
        // Determine which wall is closest and set approach heading + staging point
        double stage_x = target_x;
        double stage_y = target_y;
        float approachHeading = 0;
        
        // Find closest wall
        double distRight  = fieldEdge - target_x;
        double distLeft   = target_x + fieldEdge;
        double distTop    = fieldEdge - target_y;
        double distBottom  = target_y + fieldEdge;
        double minDist = distRight;
        
        // Right wall
        approachHeading = 90;
        stage_x = target_x - stagingDist;
        
        if (distLeft < minDist) {
            minDist = distLeft;
            approachHeading = 270;
            stage_x = target_x + stagingDist;
            stage_y = target_y;
        }
        if (distTop < minDist) {
            minDist = distTop;
            approachHeading = 0;
            stage_x = target_x;
            stage_y = target_y - stagingDist;
        }
        if (distBottom < minDist) {
            minDist = distBottom;
            approachHeading = 180;
            stage_x = target_x;
            stage_y = target_y + stagingDist;
        }
        
        Controller.Screen.clearScreen();
        Controller.Screen.print("Wall pickup H:%.0f", approachHeading);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Stage: %.0f,%.0f", stage_x, stage_y);
        cout << "Wall-adjacent ball: staging at (" << stage_x << "," << stage_y 
             << ") heading " << approachHeading << "\n";
        wait(500, msec);
        
        // Step 1: Path plan + pure pursuit to staging point
        std::vector<astar::Point> stagePath = astar::findPath(
            fieldMap, curr_x, curr_y, stage_x, stage_y,
            grid_resolution_cm, robot_radius_cm, safety_margin_cm);
        
        if (!stagePath.empty()) {
            std::vector<astar::Point> stageAdj;
            if (stagePath.size() <= 2) {
                stageAdj.push_back({stage_x, stage_y});
            } else {
                for (size_t i = 1; i < stagePath.size(); i++)
                    stageAdj.push_back(stagePath[i]);
                stageAdj.back() = {stage_x, stage_y};
            }
            
            intakeBalls();
            bool reachedStage = purePursuitFollowPath(stageAdj, 4.0f, 20.0f, 4.0f, -1.0f, true);
            chassis.drive_with_voltage(0, 0);
            task::sleep(200);
            
            if (!reachedStage) {
                cout << "Wall pickup: failed to reach staging point!\n";
                Controller.Screen.clearScreen();
                Controller.Screen.print("Failed to reach stage!");
                wait(2000, msec);
                stopIntake();
                return;
            }
        } else {
            cout << "Wall pickup: no path to staging point!\n";
            Controller.Screen.clearScreen();
            Controller.Screen.print("No path to stage!");
            wait(2000, msec);
            stopIntake();
            return;
        }
        
        // Step 2: Turn perpendicular to wall
        Controller.Screen.clearScreen();
        Controller.Screen.print("Turning to wall...");
        chassis.turn_to_angle(approachHeading);
        task::sleep(200);
        
        // Step 3: Drive straight into ball
        intakeBalls();
        float driveIn_in = (float)((stagingDist - 14.5) / 2.54);
        chassis.drive_distance(driveIn_in, approachHeading, 6, 4);
        
        // Step 4: Wait for intake to grab ball
        // task::sleep(100);
        
        // Step 5: Reverse 6 inches for clearance
        chassis.drive_distance(-6, approachHeading, 6, 4);
        
        stopIntake();
        chassis.drive_with_voltage(0, 0);
        LeftDrive.stop(hold);
        RightDrive.stop(hold);
        task::sleep(300);
        
        // Report final position
        Pose finalPose = localizer.getPose();
        double final_x = finalPose.x_cm;
        double final_y = finalPose.y_cm;
        double final_h = finalPose.heading_deg;
        double err = sqrt(pow(final_x - target_x, 2) + pow(final_y - target_y, 2));
        
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Wall pickup done!");
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Pos: %.1f,%.1f H:%.0f", final_x, final_y, final_h);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("Error: %.1f cm", err);
        
        cout << "Wall pickup complete at (" << final_x << "," << final_y << ") H=" << final_h << " err=" << err << "cm\n";
        wait(3000, msec);
        return;
    }
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("From: %.0f,%.0f", curr_x, curr_y);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("To: %.0f,%.0f", target_x, target_y);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Planning...");
    wait(500, msec);
    
    std::vector<astar::Point> path = astar::findPath(
        fieldMap,
        curr_x, curr_y,
        target_x, target_y,
        grid_resolution_cm,
        robot_radius_cm,
        safety_margin_cm
    );
    
    // If no path found, robot might be inside obstacle - back up until clear
    if (path.empty()) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("No path - backing up");
        cout << "No path found - robot may be in obstacle, backing up...\n";
        
        const int maxBackupTime = 3000;  // 3 seconds max
        const float backupVel = -2.5f;   // Slow reverse
        int backupStart = Brain.Timer.system();
        
        while (Brain.Timer.system() - backupStart < maxBackupTime) {
            // Drive backwards slowly
            chassis.drive_with_voltage(backupVel, backupVel);
            task::sleep(50);
            
            // Check position from Localizer
            Pose backupPose = localizer.getPose();
            double posX = backupPose.x_cm;
            double posY = backupPose.y_cm;
            
            // Check if we're now outside all obstacles
            if (!fieldMap.isPointInObstacle(posX, posY)) {
                // Also check with robot radius margin
                bool clearOfObstacles = true;
                for (const auto& obs : fieldMap.getObstacles()) {
                    // Check if robot center + radius would still hit obstacle
                    double dx = posX - obs.cx;
                    double dy = posY - obs.cy;
                    double dist = sqrt(dx*dx + dy*dy);
                    if (dist < robot_radius_cm + 10.0) {  // 10cm extra margin
                        clearOfObstacles = false;
                        break;
                    }
                }
                
                if (clearOfObstacles) {
                    chassis.drive_with_voltage(0, 0);
                    cout << "Cleared obstacle at (" << posX << "," << posY << "), retrying path...\n";
                    Controller.Screen.setCursor(2, 1);
                    Controller.Screen.print("Clear! Retrying...");
                    task::sleep(200);
                    
                    // Retry path planning from new position
                    path = astar::findPath(
                        fieldMap,
                        posX, posY,
                        target_x, target_y,
                        grid_resolution_cm,
                        robot_radius_cm,
                        safety_margin_cm
                    );
                    
                    // Update current position for path following
                    curr_x = posX;
                    curr_y = posY;
                    curr_h = backupPose.heading_deg;
                    break;
                }
            }
            
            // Debug every 500ms
            if ((Brain.Timer.system() - backupStart) % 500 < 50) {
                cout << "  Backing up... Pos: (" << posX << "," << posY << ")\n";
            }
        }
        chassis.drive_with_voltage(0, 0);
        
        // If still no path after backup, give up
        if (path.empty()) {
            Controller.Screen.clearScreen();
            Controller.Screen.print("Still no path!");
            cout << "Failed to find path even after backup\n";
            wait(2000, msec);
            return;
        }
    }
    
    // Skip the first waypoint (it's the cell we're starting in)
    // and add the actual target as the final point
    std::vector<astar::Point> adjustedPath;
    
    // If path is very short (start and end in same or adjacent cells), 
    // just go directly to target
    if (path.size() <= 2) {
        adjustedPath.push_back({target_x, target_y});
    } else {
        // Skip first waypoint, keep intermediate ones
        for (size_t i = 1; i < path.size(); i++) {
            adjustedPath.push_back(path[i]);
        }
        // Replace last waypoint (cell center) with actual target
        adjustedPath.back() = {target_x, target_y};
    }
    
    // Log path
    cout << "===== PURE PURSUIT PATH =====\n";
    cout << "Start: (" << curr_x << "," << curr_y << ") H=" << curr_h << "\n";
    cout << "Target: (" << target_x << "," << target_y << ")\n";
    cout << "Original A* waypoints: " << path.size() << "\n";
    cout << "Adjusted waypoints: " << adjustedPath.size() << "\n";
    for (size_t i = 0; i < adjustedPath.size(); i++) {
        cout << "  WP" << i << ": (" << adjustedPath[i].first << "," << adjustedPath[i].second << ")\n";
    }
    cout << "=============================\n";
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Following path...");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("WPs: %d", (int)adjustedPath.size());
    
    // Turn on intake before driving
    intakeBalls();
    
    // Follow path with pure pursuit + GPS sensor fusion
    // Lookahead MUST be >= grid resolution to see upcoming turns!
    // Grid = 15cm, so lookahead = 25cm to see ~2 waypoints ahead
    // Parameters: baseVelocity=4V, lookahead=20cm (reduced), endTolerance=4cm, no final heading, GPS fusion on
    // Lookahead at 20cm for stable path following
    // End tolerance at 4cm for reliable stopping
    bool success = purePursuitFollowPath(adjustedPath, 4.0f, 20.0f, 4.0f, -1.0f, true);
    
    // Stop intake and drive
    stopIntake();
    chassis.drive_with_voltage(0, 0);
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
    wait(300, msec);
    
    // Final position from Localizer
    Pose finalPose = localizer.getPose();
    double final_x = finalPose.x_cm;
    double final_y = finalPose.y_cm;
    double final_h = finalPose.heading_deg;
    double err = sqrt(pow(final_x - target_x, 2) + pow(final_y - target_y, 2));
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print(success ? "Success!" : "Timeout/Error");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Pos: %.1f,%.1f H:%.0f", final_x, final_y, final_h);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Error: %.1f cm", err);
    
    cout << "Pure Pursuit " << (success ? "SUCCESS" : "FAILED") << "\n";
    cout << "Final Pose: (" << final_x << "," << final_y << ") H=" << final_h << " Error: " << err << " cm\n";
    
    wait(3000, msec);
}

// === MULTI-POINT PATH FOLLOWING ===
// Visits multiple target points in optimal order (nearest-neighbor algorithm)
// Returns true if all points were reached successfully

// Helper: Order points using nearest-neighbor (greedy TSP approximation)
std::vector<astar::Point> orderPointsNearest(
    const std::vector<astar::Point>& points,
    double startX, double startY)
{
    if (points.empty()) return {};
    if (points.size() == 1) return points;
    
    std::vector<astar::Point> ordered;
    std::vector<bool> visited(points.size(), false);
    
    double currentX = startX;
    double currentY = startY;
    
    for (size_t i = 0; i < points.size(); i++) {
        // Find nearest unvisited point
        double minDist = 1e9;
        size_t nearestIdx = 0;
        
        for (size_t j = 0; j < points.size(); j++) {
            if (visited[j]) continue;
            
            double dx = points[j].first - currentX;
            double dy = points[j].second - currentY;
            double dist = sqrt(dx*dx + dy*dy);
            
            if (dist < minDist) {
                minDist = dist;
                nearestIdx = j;
            }
        }
        
        visited[nearestIdx] = true;
        ordered.push_back(points[nearestIdx]);
        currentX = points[nearestIdx].first;
        currentY = points[nearestIdx].second;
    }
    
    return ordered;
}

// Navigate through multiple points in sequence
// Each point is visited using A* + pure pursuit
bool navigateMultiplePoints(
    const std::vector<astar::Point>& targetPoints,
    bool autoOrder)  // If true, reorder using nearest-neighbor; if false, use given order
{
    if (targetPoints.empty()) {
        cout << "navigateMultiplePoints: No points provided\n";
        return false;
    }
    
    // Get current position from Localizer
    Pose startPose = localizer.getPose();
    double startX = startPose.x_cm;
    double startY = startPose.y_cm;
    
    // Order points if requested
    std::vector<astar::Point> orderedPoints;
    if (autoOrder && targetPoints.size() > 1) {
        orderedPoints = orderPointsNearest(targetPoints, startX, startY);
        cout << "Multi-point: Reordered " << targetPoints.size() << " points using nearest-neighbor\n";
    } else {
        orderedPoints = targetPoints;
    }
    
    // Log the order
    cout << "===== MULTI-POINT NAVIGATION =====\n";
    cout << "Start: (" << startX << "," << startY << ")\n";
    cout << "Points to visit: " << orderedPoints.size() << "\n";
    for (size_t i = 0; i < orderedPoints.size(); i++) {
        cout << "  " << (i+1) << ": (" << orderedPoints[i].first << "," << orderedPoints[i].second << ")\n";
    }
    cout << "==================================\n";
    
    // Field map and path planning parameters
    FieldMap fieldMap;
    fieldMap.populateStandardField();
    
    const double robot_width_in = 13.5;
    const double robot_radius_cm = robot_width_in * 2.54 * 0.5;
    const double safety_margin_cm = 2.0;
    const double grid_resolution_cm = 15.0;
    
    int successCount = 0;
    
    // Visit each point in order
    for (size_t i = 0; i < orderedPoints.size(); i++) {
        double targetX = orderedPoints[i].first;
        double targetY = orderedPoints[i].second;
        
        // Get current position from Localizer
        Pose currPose = localizer.getPose();
        double currX = currPose.x_cm;
        double currY = currPose.y_cm;
        
        cout << "\nMulti-point: Navigating to point " << (i+1) << "/" << orderedPoints.size() 
             << " (" << targetX << "," << targetY << ")\n";
        
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Point %d/%d", (int)(i+1), (int)orderedPoints.size());
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("To: %.0f,%.0f", targetX, targetY);
        
        // Plan path with A*
        std::vector<astar::Point> path = astar::findPath(
            fieldMap,
            currX, currY,
            targetX, targetY,
            grid_resolution_cm,
            robot_radius_cm,
            safety_margin_cm
        );
        
        if (path.empty()) {
            cout << "Multi-point: No path to point " << (i+1) << ", skipping\n";
            Controller.Screen.setCursor(3, 1);
            Controller.Screen.print("No path!");
            wait(500, msec);
            continue;
        }
        
        // Adjust path (skip first waypoint, use actual target as end)
        std::vector<astar::Point> adjustedPath;
        if (path.size() <= 2) {
            adjustedPath.push_back({targetX, targetY});
        } else {
            for (size_t j = 1; j < path.size(); j++) {
                adjustedPath.push_back(path[j]);
            }
            adjustedPath.back() = {targetX, targetY};
        }
        
        // Follow path with pure pursuit
        // Use slightly larger end tolerance for intermediate points (6cm)
        // Use tighter tolerance (4cm) for the final point
        float endTolerance = (i == orderedPoints.size() - 1) ? 4.0f : 6.0f;
        
        bool success = purePursuitFollowPath(adjustedPath, 4.0f, 20.0f, endTolerance, -1.0f, true);
        
        if (success) {
            successCount++;
            cout << "Multi-point: Reached point " << (i+1) << "\n";
            
            // Brief pause at each point (for ball pickup, etc.)
            Controller.Screen.setCursor(3, 1);
            Controller.Screen.print("Reached!");
            wait(300, msec);
        } else {
            cout << "Multi-point: Failed to reach point " << (i+1) << "\n";
            Controller.Screen.setCursor(3, 1);
            Controller.Screen.print("Failed!");
            wait(500, msec);
        }
    }
    
    // Final report
    cout << "\nMulti-point navigation complete: " << successCount << "/" << orderedPoints.size() << " points reached\n";
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Done: %d/%d pts", successCount, (int)orderedPoints.size());
    
    return (successCount == (int)orderedPoints.size());
}

// Navigate through multiple points as ONE CONTINUOUS PATH
// Plans A* between each segment, concatenates, runs pure pursuit once
bool navigateContinuousPath(
    const std::vector<astar::Point>& targetPoints,
    bool autoOrder)
{
    if (targetPoints.empty()) {
        cout << "navigateContinuousPath: No points provided\n";
        return false;
    }
    
    // Get current position from Localizer
    Pose startPose = localizer.getPose();
    double startX = startPose.x_cm;
    double startY = startPose.y_cm;
    
    // Order points if requested
    std::vector<astar::Point> orderedPoints;
    if (autoOrder && targetPoints.size() > 1) {
        orderedPoints = orderPointsNearest(targetPoints, startX, startY);
        cout << "Continuous path: Reordered " << targetPoints.size() << " points\n";
    } else {
        orderedPoints = targetPoints;
    }
    
    // Log the order
    cout << "===== CONTINUOUS PATH PLANNING =====\n";
    cout << "Start: (" << startX << "," << startY << ")\n";
    cout << "Points: " << orderedPoints.size() << "\n";
    for (size_t i = 0; i < orderedPoints.size(); i++) {
        cout << "  " << (i+1) << ": (" << orderedPoints[i].first << "," << orderedPoints[i].second << ")\n";
    }
    
    // Field map and path planning parameters
    FieldMap fieldMap;
    fieldMap.populateStandardField();
    
    const double robot_width_in = 13.5;
    const double robot_radius_cm = robot_width_in * 2.54 * 0.5;
    const double safety_margin_cm = 2.0;
    const double grid_resolution_cm = 15.0;
    
    // Build combined path through all points
    std::vector<astar::Point> combinedPath;
    
    double currX = startX;
    double currY = startY;
    
    for (size_t i = 0; i < orderedPoints.size(); i++) {
        double targetX = orderedPoints[i].first;
        double targetY = orderedPoints[i].second;
        
        cout << "Planning segment " << (i+1) << ": (" << currX << "," << currY 
             << ") -> (" << targetX << "," << targetY << ")\n";
        
        // Plan A* path for this segment
        std::vector<astar::Point> segmentPath = astar::findPath(
            fieldMap,
            currX, currY,
            targetX, targetY,
            grid_resolution_cm,
            robot_radius_cm,
            safety_margin_cm
        );
        
        if (segmentPath.empty()) {
            cout << "  No path for segment " << (i+1) << "!\n";
            // Skip this point and try to continue
            continue;
        }
        
        // Add segment to combined path
        // Skip first waypoint of segment (it's where we already are, except for first segment)
        size_t startIdx = (combinedPath.empty()) ? 0 : 1;
        
        for (size_t j = startIdx; j < segmentPath.size(); j++) {
            combinedPath.push_back(segmentPath[j]);
        }
        
        // Replace last point of this segment with exact target coordinates
        if (!combinedPath.empty()) {
            combinedPath.back() = {targetX, targetY};
        }
        
        // Update current position for next segment
        currX = targetX;
        currY = targetY;
    }
    
    if (combinedPath.empty()) {
        cout << "Continuous path: No valid path could be planned!\n";
        Controller.Screen.clearScreen();
        Controller.Screen.print("No path!");
        return false;
    }
    
    // Log combined path
    cout << "Combined path: " << combinedPath.size() << " waypoints\n";
    cout << "=====================================\n";
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Path: %d pts", (int)orderedPoints.size());
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("WPs: %d", (int)combinedPath.size());
    
    // Follow the entire combined path with pure pursuit (ONE continuous motion)
    bool success = purePursuitFollowPath(combinedPath, 4.0f, 20.0f, 4.0f, -1.0f, true);
    
    return success;
}

// Quick multi-point test with hardcoded points
// Edit the points vector below to change targets
void runMultiPointPath() {
    // === EDIT THESE POINTS (X, Y in cm) ===
    std::vector<astar::Point> points = {
        {120.0, 60.0},    // Point 1
        {60.0, -60.0},   // Point 2
        {-60.0, -60.0},  // Point 3
        {-120.0, -120.0}    // Point 4
    };
    // =====================================
    
    Controller.Screen.clearScreen();
    Controller.Screen.print("Continuous: %d pts", (int)points.size());
    wait(500, msec);
    
    // Navigate as ONE continuous path
    bool success = navigateContinuousPath(points, true);
    
    // Report result
    Pose finalPose = localizer.getPose();
    double finalX = finalPose.x_cm;
    double finalY = finalPose.y_cm;
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print(success ? "Success!" : "Failed");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Final: %.0f,%.0f", finalX, finalY);
    
    wait(2000, msec);
}

// Test function for multi-point navigation (interactive)
// Add points with A button, start navigation with X
void testMultiPoint() {
    std::vector<astar::Point> points;
    
    // Target being edited
    float target_x = 0.0;
    float target_y = 0.0;
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Multi-Point Test");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Arrows=XY A=add");
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("X=GO B=exit");
    wait(1000, msec);
    
    // Interactive point adding loop
    while (!Controller.ButtonB.pressing()) {
        // Adjust target with arrows (10cm increments)
        if (Controller.ButtonUp.pressing()) {
            target_y += 10.0;
            wait(150, msec);
        }
        if (Controller.ButtonDown.pressing()) {
            target_y -= 10.0;
            wait(150, msec);
        }
        if (Controller.ButtonRight.pressing()) {
            target_x += 10.0;
            wait(150, msec);
        }
        if (Controller.ButtonLeft.pressing()) {
            target_x -= 10.0;
            wait(150, msec);
        }
        
        // Add current point to list
        if (Controller.ButtonA.pressing()) {
            points.push_back({target_x, target_y});
            Controller.rumble(".");
            cout << "Added point " << points.size() << ": (" << target_x << "," << target_y << ")\n";
            wait(300, msec);
        }
        
        // Clear all points
        if (Controller.ButtonY.pressing()) {
            points.clear();
            Controller.rumble("-");
            cout << "Cleared all points\n";
            wait(300, msec);
        }
        
        // Start navigation
        if (Controller.ButtonX.pressing()) {
            if (points.empty()) {
                Controller.Screen.clearScreen();
                Controller.Screen.print("No points added!");
                wait(1000, msec);
            } else {
                Controller.Screen.clearScreen();
                Controller.Screen.print("Starting...");
                wait(500, msec);
                
                // Navigate with auto-ordering
                bool success = navigateMultiplePoints(points, true);
                
                Controller.Screen.clearScreen();
                Controller.Screen.print(success ? "All done!" : "Some failed");
                wait(2000, msec);
                
                // Clear points after navigation
                points.clear();
            }
        }
        
        // Update display
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Pt: %.0f,%.0f", target_x, target_y);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Added: %d pts", (int)points.size());
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("A=add X=GO B=exit");
        
        wait(50, msec);
    }
    
    Controller.Screen.clearScreen();
    Controller.Screen.print("Exited");
}

// Test function for drive_to_pose - interactive target selection
void testBoomerang() {
    // Use controller to select target position and heading
    // Arrows adjust X/Y, L1/R1 adjust heading
    float target_x = 0.0;   // cm
    float target_y = 0.0;   // cm  
    float target_h = 0.0;   // degrees
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Boomerang Test");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Arrows=XY L/R1=H");
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("A=GO B=exit");
    wait(1000, msec);
    
    // Get current position from Localizer as starting reference
    Pose currPose = localizer.getPose();
    float curr_x = currPose.x_cm;
    float curr_y = currPose.y_cm;
    float curr_h = currPose.heading_deg;
    
    // Interactive target selection loop
    while (!Controller.ButtonB.pressing()) {
        // Adjust target with arrows (10cm increments)
        if (Controller.ButtonUp.pressing()) {
            target_y += 10.0;
            wait(200, msec);
        }
        if (Controller.ButtonDown.pressing()) {
            target_y -= 10.0;
            wait(200, msec);
        }
        if (Controller.ButtonRight.pressing()) {
            target_x += 10.0;
            wait(200, msec);
        }
        if (Controller.ButtonLeft.pressing()) {
            target_x -= 10.0;
            wait(200, msec);
        }
        // Adjust heading with L1/R1 (15 degree increments)
        if (Controller.ButtonL1.pressing()) {
            target_h -= 15.0;
            if (target_h < 0) target_h += 360.0;
            wait(200, msec);
        }
        if (Controller.ButtonR1.pressing()) {
            target_h += 15.0;
            if (target_h >= 360) target_h -= 360.0;
            wait(200, msec);
        }
        
        // Display current target
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Tgt: %.0f,%.0f", target_x, target_y);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("H: %.0f deg", target_h);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("A=GO B=exit");
        
        // Execute on A press
        if (Controller.ButtonA.pressing()) {
            waitUntil(!Controller.ButtonA.pressing());
            
            // Get fresh position from Localizer
            currPose = localizer.getPose();
            curr_x = currPose.x_cm;
            curr_y = currPose.y_cm;
            curr_h = currPose.heading_deg;
            
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("From: %.0f,%.0f", curr_x, curr_y);
            Controller.Screen.setCursor(2, 1);
            Controller.Screen.print("To: %.0f,%.0f H=%.0f", target_x, target_y, target_h);
            wait(500, msec);
            
            // Convert cm to inches for JAR-Template
            float curr_x_in = curr_x / 2.54;
            float curr_y_in = curr_y / 2.54;
            float target_x_in = target_x / 2.54;
            float target_y_in = target_y / 2.54;
            
            // Sync odometry with GPS
            chassis.set_coordinates(curr_x_in, curr_y_in, curr_h);
            
            // Debug: print distance and angle to target
            float dx = target_x - curr_x;
            float dy = target_y - curr_y;
            float dist_cm = sqrt(dx*dx + dy*dy);
            float angle_to_target = atan2(dx, dy) * 180.0 / M_PI;  // JAR uses atan2(x,y) convention
            if (angle_to_target < 0) angle_to_target += 360.0;
            
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Dist: %.0fcm", dist_cm);
            Controller.Screen.setCursor(2, 1);
            Controller.Screen.print("Ang: %.0f H: %.0f", angle_to_target, curr_h);
            Controller.Screen.setCursor(3, 1);
            Controller.Screen.print("Driving...");
            wait(1000, msec);  // Pause to see debug info
            
            // Execute drive_to_pose (with final heading)
            // Using explicit parameters: lead=0.3, setback=1.5, min_volt=0, max_volt=6, heading_volt=4
            chassis.drive_to_pose(target_x_in, target_y_in, target_h, 0.3, 1.5, 0, 6, 4);
            
            // Force stop motors
            chassis.drive_with_voltage(0, 0);
            chassis.drive_stop(brake);
            
            // Show result
            Pose finalPose = localizer.getPose();
            float final_x = finalPose.x_cm;
            float final_y = finalPose.y_cm;
            float final_imu_h = finalPose.heading_deg;
            
            // Position error (using Localizer)
            float err_x = fabs(final_x - target_x);
            float err_y = fabs(final_y - target_y);
            float err_dist = sqrt(err_x*err_x + err_y*err_y);
            
            // Heading error using IMU (normalized to -180 to 180)
            float err_h = final_imu_h - target_h;
            while (err_h > 180) err_h -= 360;
            while (err_h < -180) err_h += 360;
            
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Pos err: %.1fcm", err_dist);
            Controller.Screen.setCursor(2, 1);
            Controller.Screen.print("H: %.0f err: %.1f", final_imu_h, err_h);
            Controller.Screen.setCursor(3, 1);
            Controller.Screen.print("A=again B=exit");
            
            // Wait for next action
            waitUntil(Controller.ButtonA.pressing() || Controller.ButtonB.pressing());
            if (Controller.ButtonB.pressing()) break;
            waitUntil(!Controller.ButtonA.pressing());
        }
        
        wait(50, msec);
    }
    
    waitUntil(!Controller.ButtonB.pressing());
    
    // Reset stopping type to coast when exiting
    chassis.drive_stop(coast);
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Boomerang done");
    wait(500, msec);
}

// Test function for drive commands - interactive, press A to advance each test
void testDriveCommands() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Drive Test Menu");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("A=next B=exit");
    
    waitUntil(Controller.ButtonA.pressing());
    waitUntil(!Controller.ButtonA.pressing());
    
    // Test 1: Raw motor spin - LeftDrive group (physically RIGHT side)
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("1: LeftDrive grp");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("(phys RIGHT side)");
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("A=run B=skip");
    waitUntil(Controller.ButtonA.pressing() || Controller.ButtonB.pressing());
    if (Controller.ButtonB.pressing()) { waitUntil(!Controller.ButtonB.pressing()); }
    else {
        waitUntil(!Controller.ButtonA.pressing());
        // Spin reverse because fwd=backwards physically
        LeftDriveA.spin(reverse, 4, volt);
        LeftDriveB.spin(reverse, 4, volt);
        LeftDriveC.spin(reverse, 4, volt);
        wait(1500, msec);
        LeftDriveA.stop();
        LeftDriveB.stop();
        LeftDriveC.stop();
    }
    wait(300, msec);
    
    // Test 2: Raw motor spin - RightDrive group (physically LEFT side)
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("2: RightDrive grp");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("(phys LEFT side)");
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("A=run B=skip");
    waitUntil(Controller.ButtonA.pressing() || Controller.ButtonB.pressing());
    if (Controller.ButtonB.pressing()) { waitUntil(!Controller.ButtonB.pressing()); }
    else {
        waitUntil(!Controller.ButtonA.pressing());
        // Spin reverse because fwd=backwards physically
        RightDriveA.spin(reverse, 4, volt);
        RightDriveB.spin(reverse, 4, volt);
        RightDriveC.spin(reverse, 4, volt);
        wait(1500, msec);
        RightDriveA.stop();
        RightDriveB.stop();
        RightDriveC.stop();
    }
    wait(300, msec);
    
    // Test 3: drive_with_voltage forward
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("3: voltage(4,4)");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("A=run B=skip");
    waitUntil(Controller.ButtonA.pressing() || Controller.ButtonB.pressing());
    if (Controller.ButtonB.pressing()) { waitUntil(!Controller.ButtonB.pressing()); }
    else {
        waitUntil(!Controller.ButtonA.pressing());
        chassis.drive_with_voltage(4, 4);
        wait(1500, msec);
        chassis.drive_with_voltage(0, 0);
    }
    wait(300, msec);
    
    // Test 4: drive_with_voltage backward
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("4: voltage(-4,-4)");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("A=run B=skip");
    waitUntil(Controller.ButtonA.pressing() || Controller.ButtonB.pressing());
    if (Controller.ButtonB.pressing()) { waitUntil(!Controller.ButtonB.pressing()); }
    else {
        waitUntil(!Controller.ButtonA.pressing());
        chassis.drive_with_voltage(-4, -4);
        wait(1500, msec);
        chassis.drive_with_voltage(0, 0);
    }
    wait(300, msec);
    
    // Test 5: Check tracking wheel readings - live update
    // Reset odometry to 0,0,0 first
    chassis.set_coordinates(0, 0, localizer.getPose().heading_deg);
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("5: Odom test");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Push robot FWD");
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("B=exit");
    wait(1000, msec);
    
    // Live update loop - show odometry X,Y position
    while (!Controller.ButtonB.pressing()) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("X:%.1f Y:%.1f", chassis.get_X_position(), chassis.get_Y_position());
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("H:%.1f", chassis.get_absolute_heading());
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("FWD=+X B=exit");
        wait(100, msec);
    }
    waitUntil(!Controller.ButtonB.pressing());
    
    // Test 6: Reset odom and drive_distance 10
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("6: drive_dist(10)");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("A=run B=skip");
    waitUntil(Controller.ButtonA.pressing() || Controller.ButtonB.pressing());
    if (Controller.ButtonB.pressing()) { waitUntil(!Controller.ButtonB.pressing()); }
    else {
        waitUntil(!Controller.ButtonA.pressing());
        chassis.set_coordinates(0, 0, 0);
        chassis.drive_distance(10);
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Odom says:");
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("X:%.1f Y:%.1f", chassis.get_X_position(), chassis.get_Y_position());
        wait(2000, msec);
    }
    wait(300, msec);
    
    // Test 7: turn_to_angle 90
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("7: turn_to(90)");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("A=run B=skip");
    waitUntil(Controller.ButtonA.pressing() || Controller.ButtonB.pressing());
    if (Controller.ButtonB.pressing()) { waitUntil(!Controller.ButtonB.pressing()); }
    else {
        waitUntil(!Controller.ButtonA.pressing());
        chassis.set_coordinates(0, 0, 0);
        chassis.turn_to_angle(90);
    }
    wait(300, msec);
    
    // Test 8: drive_to_pose with GPS global coordinates
    // JAR-Template uses INCHES internally (wheel diameter is 2.75 inches)
    // GPS returns CM, so we must convert
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("8: GPS drive_pose");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("To (-60,-60)cm H=270");
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("A=run B=skip");
    waitUntil(Controller.ButtonA.pressing() || Controller.ButtonB.pressing());
    if (Controller.ButtonB.pressing()) { waitUntil(!Controller.ButtonB.pressing()); }
    else {
        waitUntil(!Controller.ButtonA.pressing());
        
        // Get current position from Localizer (cm) and heading
        Pose startPose = localizer.getPose();
        float startX_cm = startPose.x_cm;
        float startY_cm = startPose.y_cm;
        float startH = startPose.heading_deg;
        
        // Convert cm to inches for JAR (1 inch = 2.54 cm)
        float startX_in = startX_cm / 2.54;
        float startY_in = startY_cm / 2.54;
        
        // Sync JAR odom to current GPS position IN INCHES
        chassis.set_coordinates(startX_in, startY_in, startH);
        
        // Target in cm, convert to inches
        float targetX_cm = -60.0;
        float targetY_cm = -60.0;
        float targetX_in = targetX_cm / 2.54;
        float targetY_in = targetY_cm / 2.54;
        float targetH = 270.0;
        
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("From: %.0f,%.0f cm", startX_cm, startY_cm);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("To: %.0f,%.0f cm", targetX_cm, targetY_cm);
        wait(1000, msec);
        
        // Drive to target (in inches)
        chassis.drive_to_pose(targetX_in, targetY_in, targetH);
        
        // Show result - Localizer vs raw odom for comparison
        Pose endPose = localizer.getPose();
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("LOC: %.0f,%.0f cm", endPose.x_cm, endPose.y_cm);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Odom: %.0f,%.0f cm", chassis.get_X_position() * 2.54, chassis.get_Y_position() * 2.54);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("H: %.1f", endPose.heading_deg);
        wait(3000, msec);
    }
    wait(300, msec);
    
    // Done
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Test Complete!");
    wait(2000, msec);
}

// ============================================================================
// LOCALIZATION TEST MODE (Phase E)
// Tests EKF localization with scripted motions and CSV logging
// ============================================================================

void testLocalization() {
    Controller.Screen.clearScreen();
    Controller.Screen.print("Localization Test");
    wait(500, msec);
    
    // Initialize localizer from current GPS
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Init from GPS...");
    
    if (!localizer.initFromGPS(3000)) {
        Controller.Screen.clearScreen();
        Controller.Screen.print("GPS init failed!");
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Using (0,0,0)");
        localizer.resetPose(0, 0, 0);
        wait(1000, msec);
    }
    
    // Print CSV header
    cout << "\n===== LOCALIZATION TEST LOG =====\n";
    cout << "time_ms,x,y,theta,left_x,left_y,left_q,right_x,right_y,right_q,";
    cout << "dWall,sigmaL,sigmaR,usedL,usedR,gatedL,gatedR,sigma_x,sigma_y\n";
    
    // Logging rate control
    const int LOG_INTERVAL_MS = 50;  // 20Hz logging
    uint32_t lastLogTime = 0;
    
    // Lambda to log current state
    auto logState = [&]() {
        uint32_t now = Brain.Timer.system();
        if (now - lastLogTime < LOG_INTERVAL_MS) return;
        lastLogTime = now;
        
        Pose pose = localizer.getPose();
        LocalizerDebug dbg = localizer.getDebug();
        
        GpsReading left, right;
        GPS.getLeft(left);
        GPS.getRight(right);
        
        printf("%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%.1f,%.1f,%d,",
               now, pose.x_cm, pose.y_cm, pose.heading_deg,
               left.x_cm, left.y_cm, left.quality,
               right.x_cm, right.y_cm, right.quality);
        printf("%.1f,%.1f,%.1f,%d,%d,%d,%d,%.1f,%.1f\n",
               dbg.wall_distance_cm, dbg.sigma_left_cm, dbg.sigma_right_cm,
               dbg.left_gps_used ? 1 : 0, dbg.right_gps_used ? 1 : 0,
               dbg.left_gps_gated ? 1 : 0, dbg.right_gps_gated ? 1 : 0,
               dbg.sigma_x_cm, dbg.sigma_y_cm);
    };
    
    Controller.Screen.clearScreen();
    Controller.Screen.print("Phase 1: Stand still 3s");
    
    // === PHASE 1: Stand still for 3 seconds ===
    uint32_t phaseStart = Brain.Timer.system();
    while (Brain.Timer.system() - phaseStart < 3000) {
        logState();
        
        // Show live pose on controller
        Pose p = localizer.getPose();
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("(%.0f,%.0f) H:%.0f", p.x_cm, p.y_cm, p.heading_deg);
        
        LocalizerDebug d = localizer.getDebug();
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("sig:%.0f,%.0f L:%d R:%d", d.sigma_x_cm, d.sigma_y_cm,
                                d.left_gps_used ? 1 : 0, d.right_gps_used ? 1 : 0);
        
        wait(50, msec);
    }
    
    // === PHASE 2: Drive forward 100cm ===
    Controller.Screen.clearScreen();
    Controller.Screen.print("Phase 2: Forward 100cm");
    
    Pose startPose = localizer.getPose();
    
    // JAR-Template drive_distance is blocking, so log before and after
    logState();
    chassis.drive_distance(100.0f / 2.54f);  // 100cm in inches (blocking)
    logState();
    
    // Log for a moment after
    phaseStart = Brain.Timer.system();
    while (Brain.Timer.system() - phaseStart < 1000) {
        logState();
        Pose p = localizer.getPose();
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("(%.0f,%.0f) H:%.0f", p.x_cm, p.y_cm, p.heading_deg);
        wait(50, msec);
    }
    
    // === PHASE 3: Turn 90 degrees CW ===
    Controller.Screen.clearScreen();
    Controller.Screen.print("Phase 3: Turn 90 CW");
    
    logState();
    chassis.turn_to_angle(startPose.heading_deg + 90.0f);  // Blocking
    logState();
    
    phaseStart = Brain.Timer.system();
    while (Brain.Timer.system() - phaseStart < 1000) {
        logState();
        Pose p = localizer.getPose();
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("(%.0f,%.0f) H:%.0f", p.x_cm, p.y_cm, p.heading_deg);
        wait(50, msec);
    }
    
    // === PHASE 4: Drive square pattern ===
    Controller.Screen.clearScreen();
    Controller.Screen.print("Phase 4: Square 50cm");
    
    for (int side = 0; side < 4; side++) {
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Side %d of 4", side + 1);
        
        logState();
        chassis.drive_distance(50.0f / 2.54f);  // 50cm (blocking)
        logState();
        
        Pose p = localizer.getPose();
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("(%.0f,%.0f)", p.x_cm, p.y_cm);
        
        if (side < 3) {
            p = localizer.getPose();
            chassis.turn_to_angle(p.heading_deg + 90.0f);  // Blocking
            logState();
        }
        wait(200, msec);
    }
    wait(500, msec);
    
    // === PHASE 5: Approach wall ===
    Controller.Screen.clearScreen();
    Controller.Screen.print("Phase 5: Wall approach");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Press B to skip");
    
    // Check if near a wall already
    Pose p = localizer.getPose();
    LocalizerDebug d = localizer.getDebug();
    
    // Wait for user to skip or proceed
    wait(1000, msec);
    
    if (!Controller.ButtonB.pressing() && d.wall_distance_cm > 50) {
        // Drive toward nearest wall
        float distToWall = d.wall_distance_cm - 25.0f;  // Stop 25cm from wall
        if (distToWall > 10.0f) {
            Controller.Screen.setCursor(3, 1);
            Controller.Screen.print("Driving %.0fcm to wall", distToWall);
            
            logState();
            chassis.drive_distance(distToWall / 2.54f);  // Blocking
            logState();
        }
    }
    
    // Log while near wall
    Controller.Screen.clearScreen();
    Controller.Screen.print("Near wall logging...");
    
    phaseStart = Brain.Timer.system();
    while (Brain.Timer.system() - phaseStart < 3000) {
        logState();
        
        p = localizer.getPose();
        d = localizer.getDebug();
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("dWall:%.0f wf:%.2f", d.wall_distance_cm, d.wall_factor);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("L:%d R:%d gL:%d gR:%d", 
                                d.left_gps_used, d.right_gps_used,
                                d.left_gps_gated, d.right_gps_gated);
        
        wait(50, msec);
    }
    
    cout << "===== END LOCALIZATION TEST =====\n\n";
    
    // Final report
    p = localizer.getPose();
    d = localizer.getDebug();
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Test Complete!");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Pos: %.0f,%.0f H:%.0f", p.x_cm, p.y_cm, p.heading_deg);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Sig: %.1f,%.1f cm", d.sigma_x_cm, d.sigma_y_cm);
    
    wait(3000, msec);
}

// Live localization display (for monitoring during operation)
void showLocalizerStatus() {
    Pose p = localizer.getPose();
    LocalizerDebug d = localizer.getDebug();
    
    // Get raw GPS readings for comparison
    GpsReading left, right;
    GPS.getLeft(left);
    GPS.getRight(right);
    
    // Get direct sensor readings for debug
    float imuH = Inertial.heading(degrees);
    float leftH = LGPS.heading();
    float rightH = RGPS.heading();
    
    // Build status indicators for each GPS
    // U=used, G=gated, X=ignored (low quality)
    char lStatus = d.left_gps_used ? 'U' : (d.left_gps_gated ? 'G' : 'X');
    char rStatus = d.right_gps_used ? 'U' : (d.right_gps_gated ? 'G' : 'X');
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    // Line 1: Fused position and heading
    Controller.Screen.print("LOC:%.0f,%.0f H:%.0f", p.x_cm, p.y_cm, p.heading_deg);
    Controller.Screen.setCursor(2, 1);
    // Line 2: GPS quality and status (U=used, G=gated, X=ignored)
    Controller.Screen.print("L:%d%c R:%d%c W:%.0f", left.quality, lStatus, right.quality, rStatus, d.wall_distance_cm);
    Controller.Screen.setCursor(3, 1);
    // Line 3: Sigma values and confidence
    Controller.Screen.print("sL:%.0f sR:%.0f", d.sigma_left_cm, d.sigma_right_cm);
    
    // Detailed console output (include individual GPS headings)
    cout << "LOC:(" << p.x_cm << "," << p.y_cm << ",H:" << p.heading_deg << ") "
         << "LH:" << leftH << " RH:" << rightH << " IMU:" << imuH << " "
         << "L:(" << left.x_cm << "," << left.y_cm << ",q:" << left.quality << "," << lStatus << ",s:" << d.sigma_left_cm << ") "
         << "R:(" << right.x_cm << "," << right.y_cm << ",q:" << right.quality << "," << rStatus << ",s:" << d.sigma_right_cm << ") "
         << "Wall:" << d.wall_distance_cm << "cm IMU:" << imuH << "\n";
}

// ============================================================================
// CALIBRATION SYSTEM
// Navigate to corner, back up into wall aligner, reset localizer to known position
// ============================================================================

// Calibration point structure
struct CalibPoint {
    double start_x, start_y, heading;  // Corner position to navigate to
    double end_x, end_y;               // Position after backing up into aligner
};

// Global calibration corners (accessible to all calibration functions)
// Corner positions:
//   1: (115, 121, heading 90)   -> backs up to (74.29, 121, 90)
//   2: (-115, 121, heading 270) -> backs up to (-74.29, 121, 270)
//   3: (115, -121, heading 90)  -> backs up to (74.29, -121, 90)
//   4: (-115, -121, heading 270)-> backs up to (-74.29, -121, 270)
static CalibPoint calibrationCorners[4] = {
    { 115.0,  121.0,  90.0,  74.29,  121.0},   // Corner 0: +X, +Y
    {-115.0,  121.0, 270.0, -74.29,  121.0},   // Corner 1: -X, +Y
    { 115.0, -121.0,  90.0,  74.29, -121.0},   // Corner 2: +X, -Y
    {-115.0, -121.0, 270.0, -74.29, -121.0}    // Corner 3: -X, -Y
};

// Find the nearest calibration corner to current position
// Returns index 0-3
int findNearestCalibCorner() {
    Pose currentPose = localizer.getPose();
    double minDist = 999999.0;
    int nearest = 0;
    
    for (int i = 0; i < 4; i++) {
        double dx = calibrationCorners[i].start_x - currentPose.x_cm;
        double dy = calibrationCorners[i].start_y - currentPose.y_cm;
        double dist = sqrt(dx*dx + dy*dy);
        
        if (dist < minDist) {
            minDist = dist;
            nearest = i;
        }
    }
    
    cout << "Nearest calibration corner: " << nearest << " at distance " << minDist << "cm\n";
    return nearest;
}

// Core calibration routine - performs calibration at specified corner
// Returns true on success, false on failure
bool doCalibrationAtCorner(int cornerIndex) {
    if (cornerIndex < 0 || cornerIndex > 3) {
        cout << "Calibration: Invalid corner index " << cornerIndex << "\n";
        return false;
    }
    
    CalibPoint& cp = calibrationCorners[cornerIndex];
    
    cout << "Calibration: Starting at corner " << cornerIndex << " (" << cp.start_x << "," << cp.start_y << ") H:" << cp.heading << "\n";
    
    // === STEP 1: Navigate to corner using old pure pursuit ===
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Going to corner %d...", cornerIndex + 1);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("%.0f,%.0f H:%.0f", cp.start_x, cp.start_y, cp.heading);
    
    // Get current position
    Pose startPose = localizer.getPose();
    double start_x = startPose.x_cm;
    double start_y = startPose.y_cm;
    
    // Plan path using A*
    FieldMap fieldMap;
    fieldMap.populateStandardField();
    
    const double robot_width_in = 13.5;
    const double robot_radius_cm = robot_width_in * 2.54 * 0.5;
    const double safety_margin_cm = 2.0;
    const double grid_resolution_cm = 15.0;
    
    std::vector<astar::Point> path = astar::findPath(
        fieldMap,
        start_x, start_y,
        cp.start_x, cp.start_y,
        grid_resolution_cm,
        robot_radius_cm,
        safety_margin_cm
    );
    
    if (path.empty()) {
        cout << "Calibration: A* failed - no path to corner!\n";
        Controller.Screen.clearScreen();
        Controller.Screen.print("No path found!");
        wait(1000, msec);
        return false;
    }
    
    // Convert to pair path for old pure pursuit
    std::vector<std::pair<double,double>> adjustedPath;
    if (path.size() <= 2) {
        adjustedPath.push_back({cp.start_x, cp.start_y});
    } else {
        for (size_t i = 1; i < path.size(); i++) {
            adjustedPath.push_back({path[i].first, path[i].second});
        }
        adjustedPath.back() = {cp.start_x, cp.start_y};
    }
    
    // Follow path with OLD pure pursuit (with final heading)
    bool navSuccess = purePursuitFollowPathOld(adjustedPath, 4.0f, 25.0f, 3.0f, cp.heading, true);
    
    if (!navSuccess) {
        cout << "Calibration: Navigation to corner failed!\n";
        Controller.Screen.clearScreen();
        Controller.Screen.print("Nav failed!");
        wait(1000, msec);
        return false;
    }
    
    // Brief pause at corner
    chassis.drive_with_voltage(0, 0);
    wait(500, msec);
    
    // === STEP 2-4: Align, backup, and validate (with retry) ===
    const int maxRetries = 3;
    bool alignmentSuccess = false;
    
    for (int attempt = 1; attempt <= maxRetries && !alignmentSuccess; attempt++) {
        cout << "Calibration: Alignment attempt " << attempt << "/" << maxRetries << "\n";
        
        // Get current position from localizer
        Pose currentPose = localizer.getPose();
        double curr_x = currentPose.x_cm;
        double curr_y = currentPose.y_cm;
        double curr_h = currentPose.heading_deg;
        
        // Calculate angle from current position to end position (where we want to back up to)
        double dx = cp.end_x - curr_x;
        double dy = cp.end_y - curr_y;
        double angleToEnd_rad = atan2(dy, dx);  // Math convention: 0 = east, CCW positive
        double angleToEnd_deg = angleToEnd_rad * 180.0 / M_PI;
        
        // Convert to nav heading (0 = north, CW positive)
        double navAngleToEnd = 90.0 - angleToEnd_deg;
        while (navAngleToEnd < 0) navAngleToEnd += 360.0;
        while (navAngleToEnd >= 360) navAngleToEnd -= 360.0;
        
        // To back up toward end position, we need to face AWAY from it (add 180°)
        double backupHeading = navAngleToEnd + 180.0;
        while (backupHeading >= 360) backupHeading -= 360.0;
        
        cout << "Calibration: Current pos (" << curr_x << "," << curr_y << ") H:" << curr_h << "\n";
        cout << "Calibration: End pos (" << cp.end_x << "," << cp.end_y << ")\n";
        cout << "Calibration: Angle to end: " << navAngleToEnd << "°, backup heading: " << backupHeading << "°\n";
        
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Attempt %d: Aligning...", attempt);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Turn to H:%.0f", backupHeading);
        
        // Turn to backup heading
        cout << "Calibration: Turning to backup heading " << backupHeading << "°\n";
        
        int turnTimeout = 3000;  // 3 second timeout
        int turnStart = Brain.Timer.system();
        
        while (Brain.Timer.system() - turnStart < turnTimeout) {
            Pose turnPose = localizer.getPose();
            double hErr = backupHeading - turnPose.heading_deg;
            while (hErr > 180) hErr -= 360;
            while (hErr < -180) hErr += 360;
            
            if (fabs(hErr) < 1.0) {
                cout << "Calibration: Aligned at heading " << turnPose.heading_deg << "\n";
                break;
            }
            
            float turnVel = 0.08f * (float)hErr;
            turnVel = fmax(-5.0f, fmin(5.0f, turnVel));
            if (fabs(turnVel) < 1.5f && fabs(turnVel) > 0.1f) {
                turnVel = (turnVel > 0) ? 1.5f : -1.5f;
            }
            
            chassis.drive_with_voltage(-turnVel, turnVel);  // Match turn_to_angle convention
            task::sleep(10);
        }
        
        chassis.drive_with_voltage(0, 0);
        wait(200, msec);
        
        // Update current heading after turn
        Pose alignedPose = localizer.getPose();
        
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Backing up...");
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("H:%.0f", alignedPose.heading_deg);
        
        cout << "Calibration: Backing up for 1.5 seconds...\n";
        
        wait(300, msec);
        
        // Drive backwards at max voltage for 1.5 seconds
        int backupStart = Brain.Timer.system();
        while (Brain.Timer.system() - backupStart < 1500) {
            chassis.drive_with_voltage(-12.0f, -12.0f);
            task::sleep(10);
        }
        
        // Stop
        chassis.drive_with_voltage(0, 0);
        LeftDrive.stop(brake);
        RightDrive.stop(brake);
        
        wait(300, msec);
        
        // === VALIDATION: Check if heading is within ±10° of expected ===
        Pose postBackupPose = localizer.getPose();
        double headingDiff = postBackupPose.heading_deg - cp.heading;
        while (headingDiff > 180) headingDiff -= 360;
        while (headingDiff < -180) headingDiff += 360;
        
        cout << "Calibration: Post-backup heading: " << postBackupPose.heading_deg << "°, expected: " << cp.heading << "°, diff: " << headingDiff << "°\n";
        
        if (fabs(headingDiff) <= 10.0) {
            // Success! Heading is within tolerance
            cout << "Calibration: Alignment SUCCESS - heading within ±10° of expected\n";
            alignmentSuccess = true;
            
            // Reset localizer to calculated end position
            localizer.resetPose(cp.end_x, cp.end_y, cp.heading);
            chassis.set_coordinates(cp.end_x / 2.54f, cp.end_y / 2.54f, cp.heading);
            
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Aligned! Reset to:");
            Controller.Screen.setCursor(2, 1);
            Controller.Screen.print("%.2f,%.0f H:%.0f", cp.end_x, cp.end_y, cp.heading);
            
            cout << "Calibration: Localizer reset to (" << cp.end_x << "," << cp.end_y << ") H:" << cp.heading << "\n";
            
            wait(500, msec);
        } else {
            // Failed - heading too far off, retry
            cout << "Calibration: Alignment FAILED - heading off by " << headingDiff << "°, retrying...\n";
            
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Missed! H diff: %.1f", headingDiff);
            Controller.Screen.setCursor(2, 1);
            Controller.Screen.print("Going back to corner...");
            
            wait(500, msec);
            
            // Navigate back to corner position using pure pursuit
            Pose retryPose = localizer.getPose();
            std::vector<astar::Point> retryPath = astar::findPath(
                fieldMap,
                retryPose.x_cm, retryPose.y_cm,
                cp.start_x, cp.start_y,
                grid_resolution_cm,
                robot_radius_cm,
                safety_margin_cm
            );
            
            if (!retryPath.empty()) {
                std::vector<std::pair<double,double>> retryAdjustedPath;
                if (retryPath.size() <= 2) {
                    retryAdjustedPath.push_back({cp.start_x, cp.start_y});
                } else {
                    for (size_t i = 1; i < retryPath.size(); i++) {
                        retryAdjustedPath.push_back({retryPath[i].first, retryPath[i].second});
                    }
                    retryAdjustedPath.back() = {cp.start_x, cp.start_y};
                }
                
                purePursuitFollowPathOld(retryAdjustedPath, 4.0f, 25.0f, 3.0f, -1.0f, true);
            } else {
                // Fallback: just drive forward a bit if no path found
                chassis.drive_with_voltage(6.0f, 6.0f);
                wait(800, msec);
                chassis.drive_with_voltage(0, 0);
            }
            
            wait(300, msec);
        }
    }
    
    if (!alignmentSuccess) {
        cout << "Calibration: FAILED after " << maxRetries << " attempts!\n";
        Controller.Screen.clearScreen();
        Controller.Screen.print("Calibration FAILED!");
        wait(2000, msec);
        return false;
    }
    
    // === STEP 5: Drive forward 10 inches ===
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Driving fwd 10in...");
    
    cout << "Calibration: Driving forward 10 inches...\n";
    
    chassis.drive_distance(10.0f);  // 10 inches forward (blocking)
    
    // Final stop
    chassis.drive_with_voltage(0, 0);
    LeftDrive.stop(brake);
    RightDrive.stop(brake);
    
    wait(300, msec);
    
    // Report final position
    Pose finalPose = localizer.getPose();
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Calibration Done!");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Loc:%.1f,%.1f", finalPose.x_cm, finalPose.y_cm);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("H:%.1f", finalPose.heading_deg);
    
    cout << "Calibration complete. Final pose: (" << finalPose.x_cm << "," << finalPose.y_cm << ") H:" << finalPose.heading_deg << "\n";
    
    return true;
}

// Auto-calibrate: find nearest corner and calibrate there
// Call this after several pure pursuit moves to reset localization error
bool autoCalibrate() {
    cout << "Auto-calibration: Finding nearest corner...\n";
    
    int nearest = findNearestCalibCorner();
    
    Controller.Screen.clearScreen();
    Controller.Screen.print("Auto-calib corner %d", nearest + 1);
    wait(500, msec);
    
    bool success = doCalibrationAtCorner(nearest);
    
    if (success) {
        wait(1000, msec);  // Brief pause to show success
    }
    
    return success;
}

// Interactive calibration: manual corner selection via controller
void calibrationBackup() {
    Controller.Screen.clearScreen();
    Controller.Screen.print("Calibration Backup");
    wait(200, msec);
    
    // Selection UI
    int selected = 0;
    bool confirmed = false;
    
    while (!confirmed) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Select corner: %d", selected + 1);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("%.0f,%.0f H:%.0f", 
            calibrationCorners[selected].start_x, 
            calibrationCorners[selected].start_y, 
            calibrationCorners[selected].heading);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("L/R=sel A=go B=cancel");
        
        if (Controller.ButtonRight.pressing()) {
            selected = (selected + 1) % 4;
            waitUntil(!Controller.ButtonRight.pressing());
            wait(50, msec);
        }
        if (Controller.ButtonLeft.pressing()) {
            selected = (selected + 3) % 4;  // +3 mod 4 = -1
            waitUntil(!Controller.ButtonLeft.pressing());
            wait(50, msec);
        }
        if (Controller.ButtonA.pressing()) {
            waitUntil(!Controller.ButtonA.pressing());
            confirmed = true;
        }
        if (Controller.ButtonB.pressing()) {
            waitUntil(!Controller.ButtonB.pressing());
            Controller.Screen.clearScreen();
            Controller.Screen.print("Cancelled");
            wait(500, msec);
            return;
        }
        
        wait(20, msec);
    }
    
    // Perform calibration at selected corner
    bool success = doCalibrationAtCorner(selected);
    
    if (success) {
        wait(2000, msec);  // Show success message longer for manual mode
    }
}