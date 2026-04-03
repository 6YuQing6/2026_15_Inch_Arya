#pragma once
#include "vex.h"
#include "ai_functions.h"
#include "intake.h"
#include "color.h"

enum class MatchState {
    IDLE,
    ISOLATION,
    COLLECTING,
    SCORING,
    CALIBRATING,
    ENDGAME,
    MANUAL_OVERRIDE,
    STOPPED
};

enum class NavResult {
    SUCCESS,
    FAILED_NO_PATH,
    FAILED_STUCK,
    FAILED_TIMEOUT
};

enum class FieldSide {
    WHOLE_FIELD,
    POSITIVE_X,
    NEGATIVE_X
};

struct BallTarget {
    double x_cm;
    double y_cm;
    bool nearWall;
    OBJECT color;
    bool valid;
};

class MatchBrain {
public:
    MatchBrain();

    void setTeamColor(OBJECT color);
    OBJECT getTeamColor() const;
    void setSkipIsolation(bool skip);
    void run();
    void stop();
    bool isRunning() const;

    static OBJECT selectTeamColor();

    // Reusable navigation: plans A* path and follows with pure pursuit.
    // Handles behind-goal escape and wall-perpendicular approach automatically.
    NavResult navigateToPoint(double x_cm, double y_cm, bool wallApproach = false);

private:
    MatchState m_state;
    OBJECT m_teamColor;
    int m_matchStartMs;
    int m_moveCount;
    int m_calibrateEveryN;
    bool m_skipIsolation;
    volatile bool m_running;

    void handleIsolation();
    void handleCollecting();
    void handleEndgame();
    void handleManualOverride();

    BallTarget findNearestBall(FieldSide side = FieldSide::WHOLE_FIELD);
    bool isBallNearWall(double x_cm, double y_cm, double threshold_cm = 20.0);
    bool collectBall(const BallTarget &target);

    bool scoreAtGoal();
    bool scoreAtLongGoal();
    bool scoreAtMiddleGoalTop();
    bool scoreAtMiddleGoalBottom();

    BallTarget scanForBall(FieldSide side);
    void navigateToScanPoint();
    int m_scanWaypointIndex;

    void checkCalibration();

    int matchElapsedMs() const;
    MatchState decideNextState();
};
