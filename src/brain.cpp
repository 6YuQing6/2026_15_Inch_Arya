#include "brain.h"
#include "field_map.h"
#include "astar.h"
#include "Localizer.h"
#include <cmath>
#include <algorithm>
#include <iostream>

extern Localizer localizer;
extern ai::jetson jetson_comms;
extern int activeTeamColor;

using namespace vex;
using namespace std;

// ---------------------------------------------------------------------------
// Construction / basic accessors
// ---------------------------------------------------------------------------

MatchBrain::MatchBrain()
    : m_state(MatchState::IDLE),
      m_teamColor(BallRed),
      m_matchStartMs(0),
      m_moveCount(0),
      m_calibrateEveryN(8),
      m_skipIsolation(false),
      m_running(false),
      m_scanWaypointIndex(0) {}

void MatchBrain::setTeamColor(OBJECT color) {
    m_teamColor = color;
    activeTeamColor = static_cast<int>(color);
}

OBJECT MatchBrain::getTeamColor() const { return m_teamColor; }
void MatchBrain::setSkipIsolation(bool skip) { m_skipIsolation = skip; }
bool MatchBrain::isRunning() const     { return m_running; }

void MatchBrain::stop() {
    m_running = false;
    m_state = MatchState::STOPPED;
}

int MatchBrain::matchElapsedMs() const {
    return Brain.Timer.system() - m_matchStartMs;
}

// ---------------------------------------------------------------------------
// Pre-match team color selection (controller UI)
// ---------------------------------------------------------------------------

OBJECT MatchBrain::selectTeamColor() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Select Team Color");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("L1=Red  R1=Blue");

    while (true) {
        if (Controller.ButtonL1.pressing()) {
            waitUntil(!Controller.ButtonL1.pressing());
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Team: RED");
            wait(500, msec);
            return BallRed;
        }
        if (Controller.ButtonR1.pressing()) {
            waitUntil(!Controller.ButtonR1.pressing());
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Team: BLUE");
            wait(500, msec);
            return BallBlue;
        }
        if (Controller.ButtonA.pressing()) {
            waitUntil(!Controller.ButtonA.pressing());
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Team: RED (default)");
            wait(500, msec);
            return BallRed;
        }
        wait(20, msec);
    }
}

// ---------------------------------------------------------------------------
// State machine: decideNextState (timer-based transitions)
// ---------------------------------------------------------------------------

MatchState MatchBrain::decideNextState() {
    int elapsed = matchElapsedMs();

    if (!m_skipIsolation && elapsed < 15000)
        return MatchState::ISOLATION;
    if (elapsed < 85000)
        return MatchState::COLLECTING;
    return MatchState::ENDGAME;
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------

void MatchBrain::run() {
    m_running = true;
    m_matchStartMs = Brain.Timer.system();
    m_moveCount = 0;
    m_state = decideNextState();

    cout << "[Brain] run() started, team=" << (m_teamColor == BallRed ? "RED" : "BLUE")
         << (m_skipIsolation ? " (skip ISO)" : " (full match)") << "\n";

    intakeBallsSlow();

    while (m_running) {
        MatchState desired = decideNextState();
        if (desired != m_state) {
            cout << "[Brain] state transition -> " << (int)desired << "\n";
            m_state = desired;
        }

        switch (m_state) {
            case MatchState::ISOLATION:
                handleIsolation();
                break;
            case MatchState::COLLECTING:
                handleCollecting();
                break;
            case MatchState::ENDGAME:
                handleEndgame();
                break;
            case MatchState::MANUAL_OVERRIDE:
                handleManualOverride();
                break;
            default:
                break;
        }

        if (!m_running) break;
        task::sleep(20);
    }

    stopIntake();
    chassis.drive_with_voltage(0, 0);
    cout << "[Brain] run() finished. moves=" << m_moveCount << "\n";
}

// ---------------------------------------------------------------------------
// Isolation (first 15s) -- hardcoded paths, no Jetson needed
// Red = left side (ported from Sunny's isolation_Left)
// Blue = right side (mirrored: X flipped, headings 270<->90)
// ---------------------------------------------------------------------------

void MatchBrain::handleIsolation() {
    if (!m_running) return;

    bool isLeft = (m_teamColor == BallRed);
    double xSign = isLeft ? 1.0 : -1.0;
    float wallHeading = isLeft ? 270.0f : 90.0f;

    cout << "[Brain] ISOLATION phase (" << (isLeft ? "LEFT/RED" : "RIGHT/BLUE") << ")\n";

    intakeBalls();

    chassis.drive_distance(-36);
    if (!m_running) return;

    chassis.turn_to_point(-24 * xSign, 24);
    chassis.drive_max_voltage = 6;
    chassis.drive_distance(12);
    chassis.drive_max_voltage = 10;

    MatchLoader.set(true);
    chassis.drive_distance(-14);
    outakeBallsMiddle();
    MatchLoader.set(false);
    wait(1000, msec);

    if (!m_running) return;

    chassis.turn_settle_time = 300;
    intakeBalls();
    chassis.drive_settle_time = 800;
    chassis.drive_max_voltage = 8;
    MatchLoader.set(true);
    chassis.drive_to_point(-48 * xSign, 48);
    chassis.drive_settle_time = 500;
    chassis.turn_to_angle(wallHeading);

    chassis.drive_max_voltage = 5;
    chassis.drive_distance(17);
    chassis.drive_max_voltage = 10;
    wait(2, sec);

    if (!m_running) return;

    chassis.drive_timeout = 200;
    chassis.drive_distance(-10);
    chassis.turn_to_angle(wallHeading);
    chassis.drive_max_voltage = 12;
    chassis.drive_distance(-18);
    outakeBallsTop();

    m_moveCount++;
    checkCalibration();

    // Wait for isolation phase to end before transitioning
    while (m_running && decideNextState() == MatchState::ISOLATION) {
        task::sleep(50);
    }
}

// ---------------------------------------------------------------------------
// Collecting: find ball via Jetson, navigate, pick up, repeat
// ---------------------------------------------------------------------------

BallTarget MatchBrain::findNearestBall(FieldSide side) {
    BallTarget result = {0, 0, false, BallUndefined, false};
    static AI_RECORD local_map;

    // Zero detectionCount before get_data so stale data from a previous
    // call can't linger when the Jetson is off / unresponsive.
    local_map.detectionCount = 0;
    jetson_comms.get_data(&local_map);

    cout << "[Brain] findNearestBall: detectionCount=" << local_map.detectionCount << "\n";
    if (local_map.detectionCount <= 0 || local_map.detectionCount > 20) return result;

    const double fieldEdge = 182.88; // cm
    const double minProbability = 0.85;

    FieldMap fieldMap;
    fieldMap.populateStandardField();

    double lowestDist = 1e9;
    Pose pose = localizer.getPose();

    int count = std::min((int)local_map.detectionCount, 20);
    for (int i = 0; i < count; i++) {
        DETECTION_OBJECT &det = local_map.detections[i];

        if (det.classID != (int)m_teamColor) continue;

        if (det.probability <= 0 || det.probability > 1.0) continue;
        if (det.probability < minProbability) continue;

        double bx = det.mapLocation.x * 100.0; // m -> cm
        double by = det.mapLocation.y * 100.0;

        // Reject detections outside the field
        if (bx < -fieldEdge || bx > fieldEdge || by < -fieldEdge || by > fieldEdge) continue;

        // Reject null-like positions (near 0,0 with no real data)
        if (fabs(det.mapLocation.x) < 0.03 && fabs(det.mapLocation.y) < 0.03) continue;

        // Field side filter
        if (side == FieldSide::POSITIVE_X && bx < 0) continue;
        if (side == FieldSide::NEGATIVE_X && bx > 0) continue;

        // Reject balls whose coordinates land inside an obstacle (goal, wall, etc.)
        if (fieldMap.isPointInObstacle(bx, by)) continue;

        double dx = bx - pose.x_cm;
        double dy = by - pose.y_cm;
        double d  = sqrt(dx * dx + dy * dy);

        if (d < lowestDist) {
            lowestDist = d;
            result.x_cm = bx;
            result.y_cm = by;
            result.color = m_teamColor;
            result.nearWall = isBallNearWall(bx, by);
            result.valid = true;
        }
    }

    // Final sanity gate (mirrors 2024 else-branch): if the "best" target
    // ended up with coordinates suspiciously close to origin, discard it.
    if (result.valid) {
        if (fabs(result.x_cm) < 3.0 && fabs(result.y_cm) < 3.0) {
            result.valid = false;
        }
    }

    return result;
}

bool MatchBrain::isBallNearWall(double x_cm, double y_cm, double threshold_cm) {
    const double edge = 182.88;
    return (x_cm > edge - threshold_cm) || (x_cm < -edge + threshold_cm) ||
           (y_cm > edge - threshold_cm) || (y_cm < -edge + threshold_cm);
}

bool MatchBrain::collectBall(const BallTarget &target) {
    // Caller-side double-validation (2024 pattern): re-check the target
    // before committing motors, in case stale/garbage data slipped through.
    if (!target.valid) return false;
    if (fabs(target.x_cm) < 3.0 && fabs(target.y_cm) < 3.0) {
        cout << "[Brain] collectBall rejected: target near origin\n";
        return false;
    }
    const double fieldEdge = 182.88;
    if (fabs(target.x_cm) > fieldEdge || fabs(target.y_cm) > fieldEdge) {
        cout << "[Brain] collectBall rejected: target outside field\n";
        return false;
    }

    Controller.Screen.clearLine(2);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Ball: %.0f, %.0f", target.x_cm, target.y_cm);
    Controller.rumble("-");

    intakeBalls();
    NavResult res = navigateToPoint(target.x_cm, target.y_cm, target.nearWall);
    if (res != NavResult::SUCCESS) {
        cout << "[Brain] collectBall failed nav: " << (int)res << "\n";
        intakeBallsSlow();
        return false;
    }
    task::sleep(300);
    intakeBallsSlow();
    m_moveCount++;
    checkCalibration();
    return true;
}

// ---------------------------------------------------------------------------
// Scoring variants
// ---------------------------------------------------------------------------

bool MatchBrain::scoreAtGoal() {
    return scoreAtLongGoal();
}

bool MatchBrain::scoreAtLongGoal() {
    Pose pose = localizer.getPose();
    double goalY = (pose.y_cm >= 0) ? 120.0 : -120.0;

    outakeBallsTop();
    NavResult res = navigateToPoint(0, goalY, false);
    if (res != NavResult::SUCCESS) {
        intakeBallsSlow();
        return false;
    }
    wait(1500, msec);
    intakeBallsSlow();
    m_moveCount++;
    checkCalibration();
    return true;
}

bool MatchBrain::scoreAtMiddleGoalTop() {
    outakeBallsMiddle();
    NavResult res = navigateToPoint(0, 30, false);
    if (res != NavResult::SUCCESS) {
        intakeBallsSlow();
        return false;
    }
    wait(1500, msec);
    intakeBallsSlow();
    m_moveCount++;
    checkCalibration();
    return true;
}

bool MatchBrain::scoreAtMiddleGoalBottom() {
    outakeBallsBottom();
    NavResult res = navigateToPoint(0, -30, false);
    if (res != NavResult::SUCCESS) {
        intakeBallsSlow();
        return false;
    }
    wait(1500, msec);
    intakeBallsSlow();
    m_moveCount++;
    checkCalibration();
    return true;
}

// ---------------------------------------------------------------------------
// Scan helpers
// ---------------------------------------------------------------------------

BallTarget MatchBrain::scanForBall(FieldSide side) {
    Pose scanPose = localizer.getPose();
    double turnStep = (scanPose.y_cm > 60.0) ? -30.0 : 30.0;
    cout << "[Brain] scanForBall starting at heading=" << chassis.get_absolute_heading()
         << " y=" << scanPose.y_cm << " step=" << turnStep << "\n";
    for (int i = 0; i < 5; i++) {
        BallTarget target = findNearestBall(side);
        if (target.valid) {
            cout << "[Brain] scanForBall found ball at (" << target.x_cm
                 << "," << target.y_cm << ") after " << i << " rotations\n";
            return target;
        }
        cout << "[Brain] scanForBall rotation " << i << "/4, no ball found, turning " << turnStep << "\n";
        chassis.turn_to_angle(chassis.get_absolute_heading() + turnStep, 6);
        task::sleep(500);
    }
    cout << "[Brain] scanForBall: no ball after 4 rotations\n";
    BallTarget none;
    none.valid = false;
    return none;
}

void MatchBrain::navigateToScanPoint() {
    struct ScanWP { double x; double y; double heading; };

    static const ScanWP redWPs[3]  = { {-110, 121, 215}, {-110, 0, 215}, {-110, -121, 315} };
    static const ScanWP blueWPs[3] = { { 110, 121, 325}, { 110, 0, 325}, { 110, -121,  45} };

    const ScanWP *wps = (m_teamColor == BallRed) ? redWPs : blueWPs;
    const ScanWP &wp  = wps[m_scanWaypointIndex];

    cout << "[Brain] navigating to scan waypoint " << m_scanWaypointIndex
         << " (" << wp.x << "," << wp.y << ") h=" << wp.heading << "\n";

    NavResult res = navigateToPoint(wp.x, wp.y, false);
    if (res == NavResult::SUCCESS) {
        chassis.turn_to_angle(wp.heading, 6);
        task::sleep(300);
    }

    m_scanWaypointIndex = (m_scanWaypointIndex + 1) % 3;
    m_moveCount++;
    checkCalibration();
}

// ---------------------------------------------------------------------------
// Collecting: simplified scan/collect/score loop (debug version)
// ---------------------------------------------------------------------------
// OLD handleCollecting commented out for testing:
// void MatchBrain::handleCollecting() {
//     if (!m_running) return;
//     FieldSide homeSide = (m_teamColor == BallRed) ? FieldSide::NEGATIVE_X : FieldSide::POSITIVE_X;
//     if (isMagazineFull()) {
//         cout << "[Brain] magazine full, scoring\n";
//         scoreAtGoal();
//         return;
//     }
//     BallTarget target = scanForBall(homeSide);
//     if (target.valid) {
//         cout << "[Brain] target ball at (" << target.x_cm << "," << target.y_cm
//              << ") wall=" << target.nearWall << "\n";
//         collectBall(target);
//         return;
//     }
//     cout << "[Brain] no ball found locally, moving to scan waypoint\n";
//     navigateToScanPoint();
// }

void MatchBrain::handleCollecting() {
    if (!m_running) return;

    FieldSide homeSide = (m_teamColor == BallRed)
                         ? FieldSide::NEGATIVE_X : FieldSide::POSITIVE_X;

    // --- SCAN for ball at current position ---
    BallTarget target = scanForBall(homeSide);
    if (target.valid) {
        cout << "[Brain] going for ball at (" << target.x_cm
             << "," << target.y_cm << ")\n";
        bool grabbed = collectBall(target);

        if (grabbed) {
            // After every pickup, immediately score at nearest long goal
            Pose pose = localizer.getPose();
            double goalX    = (m_teamColor == BallRed) ? -90.0 :  90.0;
            double goalY    = (pose.y_cm >= 0) ? 121.0 : -121.0;
            double scoreHdg = (m_teamColor == BallRed) ? 270.0 :  90.0;

            cout << "[Brain] scoring: nav to (" << goalX << "," << goalY << ")\n";
            navigateToPoint(goalX, goalY, false);

            cout << "[Brain] scoring: turning to " << scoreHdg << "\n";
            chassis.turn_to_angle(scoreHdg, 6);
            task::sleep(300);

            cout << "[Brain] scoring: reversing 6 inches\n";
            chassis.drive_distance(-6, scoreHdg, 6, 6);
            task::sleep(200);

            cout << "[Brain] scoring: outtaking\n";
            outakeBallsTop();
            wait(2000, msec);
            intakeBallsSlow();

            cout << "[Brain] scoring: driving forward 5 inches\n";
            chassis.drive_distance(5, scoreHdg, 6, 6);
            task::sleep(200);

            cout << "[Brain] scored! resuming search\n";
            Controller.Screen.clearLine(2);
            Controller.Screen.setCursor(2, 1);
            Controller.Screen.print("Scored! Resuming...");
        }
        return;
    }

    // --- No ball found, move to next scan waypoint ---
    cout << "[Brain] no ball, advancing to scan waypoint\n";
    navigateToScanPoint();
}

// ---------------------------------------------------------------------------
// Endgame: park and signal (debug version)
// ---------------------------------------------------------------------------
// OLD handleEndgame commented out for testing:
// void MatchBrain::handleEndgame() {
//     if (!m_running) return;
//     cout << "[Brain] ENDGAME phase\n";
//     FieldSide homeSide = (m_teamColor == BallRed) ? FieldSide::NEGATIVE_X : FieldSide::POSITIVE_X;
//     BallTarget ball = findNearestBall(homeSide);
//     if (ball.valid) {
//         collectBall(ball);
//         scoreAtGoal();
//     }
//     navigateToPoint(-120.0, 0.0, false);
//     intakeBallsSlow();
//     chassis.drive_with_voltage(0, 0);
//     Controller.rumble("---");
//     cout << "[Brain] PARKED — endgame complete\n";
//     while (m_running) {
//         task::sleep(100);
//     }
// }

void MatchBrain::handleEndgame() {
    if (!m_running) return;
    cout << "[Brain] ENDGAME: parking\n";

    cout << "[Brain] endgame: nav to (-90, 0)\n";
    navigateToPoint(-90.0, 0.0, false);

    cout << "[Brain] endgame: turning to 90\n";
    chassis.turn_to_angle(90, 6);
    task::sleep(300);

    cout << "[Brain] endgame: reversing 10 inches\n";
    chassis.drive_distance(-10, 90, 6, 6);
    task::sleep(200);

    intakeBallsSlow();
    chassis.drive_with_voltage(0, 0);
    Controller.rumble("---");
    cout << "[Brain] PARKED, match over\n";

    while (m_running) {
        task::sleep(100);
    }
}

// ---------------------------------------------------------------------------
// Manual override (placeholder for blocking / custom sequences)
// ---------------------------------------------------------------------------

void MatchBrain::handleManualOverride() {
    // Subclass or future expansion: drive to a specific coordinate,
    // block opponent, etc. For now just idle.
    task::sleep(100);
}

// ---------------------------------------------------------------------------
// Calibration check
// ---------------------------------------------------------------------------

void MatchBrain::checkCalibration() {
    if (m_moveCount > 0 && (m_moveCount % m_calibrateEveryN) == 0) {
        cout << "[Brain] auto-calibrate would fire after " << m_moveCount << " moves (DISABLED)\n";
        // autoCalibrate();  // disabled for testing
    }
}

// ---------------------------------------------------------------------------
// navigateToPoint -- extracted from testPurePursuit (non-interactive)
// ---------------------------------------------------------------------------

NavResult MatchBrain::navigateToPoint(double target_x, double target_y, bool wallApproach) {
    Pose currPose = localizer.getPose();
    double curr_x = currPose.x_cm;
    double curr_y = currPose.y_cm;
    double curr_h = currPose.heading_deg;

    FieldMap fieldMap;
    fieldMap.populateStandardField();

    const double robot_width_in = 13.5;
    const double robot_radius_cm = robot_width_in * 2.54 * 0.5;
    const double safety_margin_cm = 0.0;
    const double grid_resolution_cm = 10.0;

    // ------------------------------------------------------------------
    // Behind-goal escape
    // ------------------------------------------------------------------
    bool behindTopGoal    = (curr_x >= -61.0 && curr_x <= 61.0 && curr_y >= 130.0 && curr_y <= 182.88);
    bool behindBottomGoal = (curr_x >= -61.0 && curr_x <= 61.0 && curr_y <= -130.0 && curr_y >= -182.88);

    if (behindTopGoal || behindBottomGoal) {
        cout << "[Nav] behind-goal escape\n";
        double escapePoints[4][2] = {{110, 155}, {-110, 155}, {110, -155}, {-110, -155}};
        int closest = 0;
        double closestDist = 1e9;
        for (int i = 0; i < 4; i++) {
            double dx = curr_x - escapePoints[i][0];
            double dy = curr_y - escapePoints[i][1];
            double d  = sqrt(dx * dx + dy * dy);
            if (d < closestDist) { closestDist = d; closest = i; }
        }

        double esc_x = escapePoints[closest][0];
        double esc_y = escapePoints[closest][1];

        Pose freshPose = localizer.getPose();
        curr_x = freshPose.x_cm;
        curr_y = freshPose.y_cm;
        curr_h = freshPose.heading_deg;

        chassis.set_heading(curr_h);
        task::sleep(100);

        double dx = esc_x - curr_x;
        double dy = esc_y - curr_y;
        double dist_cm = sqrt(dx * dx + dy * dy);

        double targetAngle = 90.0 - (atan2(dy, dx) * 180.0 / M_PI);
        while (targetAngle < 0) targetAngle += 360;
        while (targetAngle >= 360) targetAngle -= 360;

        double fwdError = targetAngle - curr_h;
        while (fwdError > 180)  fwdError -= 360;
        while (fwdError < -180) fwdError += 360;

        bool driveReverse = (fabs(fwdError) > 90.0);
        double turnAngle = targetAngle;
        if (driveReverse) turnAngle = fmod(targetAngle + 180.0, 360.0);

        float dist_in = (float)(dist_cm / 2.54);
        if (driveReverse) dist_in = -dist_in;

        chassis.turn_to_angle(turnAngle, 6);
        task::sleep(200);
        chassis.drive_distance(dist_in, turnAngle, 6, 4);
        chassis.drive_with_voltage(0, 0);
        task::sleep(200);

        Pose escPose = localizer.getPose();
        curr_x = escPose.x_cm;
        curr_y = escPose.y_cm;
        curr_h = escPose.heading_deg;
        cout << "[Nav] escaped to (" << curr_x << "," << curr_y << ")\n";
    }

    // ------------------------------------------------------------------
    // Wall-perpendicular approach
    // ------------------------------------------------------------------
    if (wallApproach) {
        const double wallThreshold = 20.0;
        const double fieldEdge = 182.88;
        const double stagingDist = 30.0;

        bool nearRightWall  = (target_x > fieldEdge - wallThreshold);
        bool nearLeftWall   = (target_x < -fieldEdge + wallThreshold);
        bool nearTopWall    = (target_y > fieldEdge - wallThreshold);
        bool nearBottomWall = (target_y < -fieldEdge + wallThreshold);
        bool nearWall = nearRightWall || nearLeftWall || nearTopWall || nearBottomWall;

        if (nearWall) {
            double stage_x = target_x;
            double stage_y = target_y;
            float approachHeading = 0;

            double distRight  = fieldEdge - target_x;
            double distLeft   = target_x + fieldEdge;
            double distTop    = fieldEdge - target_y;
            double distBottom = target_y + fieldEdge;
            double minDist = distRight;

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

            cout << "[Nav] wall approach: stage=(" << stage_x << "," << stage_y
                 << ") heading=" << approachHeading << "\n";

            std::vector<astar::Point> stagePath = astar::findPath(
                fieldMap, curr_x, curr_y, stage_x, stage_y,
                grid_resolution_cm, robot_radius_cm, safety_margin_cm);

            if (stagePath.empty()) return NavResult::FAILED_NO_PATH;

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

            if (!reachedStage) return NavResult::FAILED_STUCK;

            chassis.turn_to_angle(approachHeading);
            task::sleep(200);

            intakeBalls();
            float driveIn_in = (float)((stagingDist - 14.5) / 2.54);
            chassis.drive_distance(driveIn_in, approachHeading, 6, 4);

            chassis.drive_distance(-6, approachHeading, 6, 4);
            chassis.drive_with_voltage(0, 0);
            task::sleep(200);

            return NavResult::SUCCESS;
        }
    }

    // ------------------------------------------------------------------
    // Normal A* + pure pursuit path following
    // ------------------------------------------------------------------
    std::vector<astar::Point> path = astar::findPath(
        fieldMap, curr_x, curr_y, target_x, target_y,
        grid_resolution_cm, robot_radius_cm, safety_margin_cm);

    if (path.empty()) {
        // Robot might be inside obstacle -- try backing up
        const int maxBackupTime = 3000;
        const float backupVel = -2.5f;
        int backupStart = Brain.Timer.system();

        while (Brain.Timer.system() - backupStart < maxBackupTime) {
            chassis.drive_with_voltage(backupVel, backupVel);
            task::sleep(50);

            Pose bp = localizer.getPose();
            if (!fieldMap.isPointInObstacle(bp.x_cm, bp.y_cm)) {
                bool clear = true;
                for (const auto &obs : fieldMap.getObstacles()) {
                    double dx = bp.x_cm - obs.cx;
                    double dy = bp.y_cm - obs.cy;
                    if (sqrt(dx * dx + dy * dy) < robot_radius_cm + 10.0) {
                        clear = false;
                        break;
                    }
                }
                if (clear) {
                    chassis.drive_with_voltage(0, 0);
                    task::sleep(200);
                    path = astar::findPath(
                        fieldMap, bp.x_cm, bp.y_cm, target_x, target_y,
                        grid_resolution_cm, robot_radius_cm, safety_margin_cm);
                    curr_x = bp.x_cm;
                    curr_y = bp.y_cm;
                    curr_h = bp.heading_deg;
                    break;
                }
            }
        }
        chassis.drive_with_voltage(0, 0);

        if (path.empty()) return NavResult::FAILED_NO_PATH;
    }

    // Adjust path: skip start cell, replace last cell center with exact target
    std::vector<astar::Point> adjustedPath;
    if (path.size() <= 2) {
        adjustedPath.push_back({target_x, target_y});
    } else {
        for (size_t i = 1; i < path.size(); i++)
            adjustedPath.push_back(path[i]);
        adjustedPath.back() = {target_x, target_y};
    }

    cout << "[Nav] following " << adjustedPath.size() << " waypoints to ("
         << target_x << "," << target_y << ")\n";

    intakeBalls();
    bool success = purePursuitFollowPath(adjustedPath, 4.0f, 20.0f, 4.0f, -1.0f, true);

    stopIntake();
    chassis.drive_with_voltage(0, 0);
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
    task::sleep(300);

    if (!success) return NavResult::FAILED_STUCK;

    Pose finalPose = localizer.getPose();
    double err = sqrt(pow(finalPose.x_cm - target_x, 2) + pow(finalPose.y_cm - target_y, 2));
    cout << "[Nav] arrived err=" << err << "cm\n";

    return NavResult::SUCCESS;
}
