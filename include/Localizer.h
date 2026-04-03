/*----------------------------------------------------------------------------*/
/*    Localizer.h - EKF-based localization service for VEX V5                 */
/*    Fuses tracking wheel odometry, IMU, and dual GPS sensors                */
/*----------------------------------------------------------------------------*/

#ifndef LOCALIZER_H
#define LOCALIZER_H

#include "vex.h"
#include "DualGPS.h"
#include "JAR-Template/drive.h"
#include <cmath>

// Enable debug logging
#ifndef LOCALIZER_DEBUG
#define LOCALIZER_DEBUG 0
#endif

// Fused pose output structure
struct Pose {
    float x_cm;           // X position in cm (field coords, +X = East)
    float y_cm;           // Y position in cm (field coords, +Y = North)
    float heading_deg;    // Heading in degrees (0 = North, CW positive)
    uint32_t timestamp_ms;
};

// Debug/tuning information
struct LocalizerDebug {
    // Covariance (uncertainty)
    float sigma_x_cm;       // sqrt(P[0][0])
    float sigma_y_cm;       // sqrt(P[1][1])
    float sigma_theta_deg;  // sqrt(P[2][2]) converted to degrees
    
    // GPS usage flags
    bool left_gps_used;     // Left GPS update was applied this cycle
    bool right_gps_used;    // Right GPS update was applied this cycle
    bool left_gps_gated;    // Left GPS was rejected (outlier)
    bool right_gps_gated;   // Right GPS was rejected (outlier)
    
    // Sensor qualities
    int left_quality;
    int right_quality;
    
    // Computed sigmas for GPS
    float sigma_left_cm;
    float sigma_right_cm;
    
    // Wall proximity
    float wall_distance_cm;
    float wall_factor;      // 0 = far from wall, 1 = at wall
    
    // Residuals (for debugging)
    float residual_left_cm;
    float residual_right_cm;
    
    // Variance tracking (for debugging)
    float variance_left_cm;   // Rolling variance of left GPS
    float variance_right_cm;  // Rolling variance of right GPS
    float variance_mult_left;  // Sigma multiplier from variance
    float variance_mult_right; // Sigma multiplier from variance
};

class Localizer
{
public:
    // Constructor - takes references to sensors and chassis
    Localizer(DualGPS& gps, Drive& chassis, inertial& imu);
    ~Localizer();
    
    // === Primary interface ===
    Pose getPose() const;
    void resetPose(float x_cm, float y_cm, float heading_deg);
    
    // Initialize with known start pose (competition mode)
    void initWithStartPose(float x_cm, float y_cm, float heading_deg);
    
    // Initialize from GPS (fallback mode - waits for good quality)
    bool initFromGPS(int timeout_ms = 3000);
    
    // === Debug/tuning interface ===
    LocalizerDebug getDebug() const;
    
    // === Tuning parameters (can be adjusted at runtime) ===
    
    // Process noise (prediction uncertainty per cycle)
    float Q_xy;       // Position process noise (cm²) - default 0.5
    float Q_theta;    // Heading process noise (rad²) - default 0.001
    
    // GPS measurement noise mapping
    // sigma = sigma_base * qualityFactor * wallFactor
    float sigma_base_cm;        // Base sigma at quality 100 - default 3.0
    float sigma_quality_90_cm;  // Sigma at quality 90 - default 20.0
    float sigma_min_quality;    // Below this quality, ignore GPS - default 85
    
    // Wall proximity parameters
    float wall_threshold_cm;    // Distance at which wall effect starts - default 37.88
    float wall_multiplier;      // Sigma multiplier at wall - default 10.0
    
    // Outlier gating
    float gate_sigma_mult;      // Gate threshold = sigma * this - default 5.0
    float gate_max_cm;          // Hard maximum gate distance - default 80.0
    
    // Field boundaries
    static constexpr float FIELD_HALF_SIZE_CM = 182.88f;  // 12ft / 2 in cm
    
    // Update rate control
    void setUpdateRate(int hz);  // Default 100Hz
    
private:
    // References to hardware
    DualGPS& gps;
    Drive& chassis;
    inertial& imu;
    
    // Background thread
    static int updateThread(void* arg);
    vex::thread* update_thread;
    mutable vex::mutex pose_mutex;
    volatile bool running;
    int update_interval_ms;
    
    // EKF State: [x, y, theta] where theta is in radians internally
    float state[3];           // [x_cm, y_cm, theta_rad]
    float P[3][3];            // 3x3 covariance matrix
    
    // Previous odometry readings for delta calculation
    float prev_odom_x_in;
    float prev_odom_y_in;
    float prev_heading_deg;
    bool odom_initialized;
    
    // Cached output (protected by mutex)
    Pose current_pose;
    LocalizerDebug current_debug;
    
    // Debug logging throttle
    uint32_t last_debug_time;
    
    // === Variance-based GPS trust ===
    // Rolling buffer to track GPS stability
    static constexpr int GPS_HISTORY_SIZE = 10;  // ~100ms at 100Hz (reduced for faster detection)
    float left_gps_history_x[GPS_HISTORY_SIZE];
    float left_gps_history_y[GPS_HISTORY_SIZE];
    float right_gps_history_x[GPS_HISTORY_SIZE];
    float right_gps_history_y[GPS_HISTORY_SIZE];
    int left_history_idx;
    int right_history_idx;
    int left_history_count;
    int right_history_count;
    
    // Variance tracking
    float left_gps_variance;
    float right_gps_variance;
    
    // Variance tuning parameters
    float variance_low_cm;       // Below this variance, GPS is stable (default 2.0)
    float variance_high_cm;      // Above this, GPS is unstable (default 10.0)
    float variance_max_mult;     // Max sigma multiplier from variance (default 5.0)
    
    // Helper function for variance
    void updateGpsVariance(float x, float y, bool is_left);
    float getVarianceSigmaMult(bool is_left) const;
    
    // Core EKF functions
    void predict(float dx_cm, float dy_cm, float dtheta_rad);
    void updateGPS(float z_x, float z_y, float sigma);
    
    // Helper functions
    float qualityToSigma(int quality) const;
    float computeWallFactor(float x_cm, float y_cm) const;
    float computeWallDistance(float x_cm, float y_cm) const;
    float normalizeAngle(float angle_rad) const;  // Wrap to [-pi, pi]
    float degToRad(float deg) const;
    float radToDeg(float rad) const;
    
    // Main update loop
    void update();
};

#endif // LOCALIZER_H
