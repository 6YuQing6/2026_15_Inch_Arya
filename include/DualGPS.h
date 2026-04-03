/*----------------------------------------------------------------------------*/
/*    DualGPS.h - Refactored Dual GPS sensor interface                        */
/*    Provides raw GPS readings for use by Localizer EKF                      */
/*    Thread-safe with proper mutex handling                                  */
/*----------------------------------------------------------------------------*/

#ifndef DUAL_GPS_H
#define DUAL_GPS_H

#include <cmath>
#include "vex.h"

using namespace vex;

// Enable debug logging (1Hz diagnostic prints)
#ifndef DUALGPS_DEBUG
#define DUALGPS_DEBUG 0
#endif

// Raw GPS reading structure - one per sensor
struct GpsReading {
    float x_cm;           // X position in cm (field coords, +X = East)
    float y_cm;           // Y position in cm (field coords, +Y = North)
    float heading_deg;    // Heading in degrees (0 = North, CW positive)
    int quality;          // Quality 0-100
    uint32_t timestamp;   // VEX timestamp in ms
    bool valid;           // True if reading is recent and not NaN
};

// Center position derived from both GPS (for debug only)
struct GpsCenter {
    float x_cm;
    float y_cm;
    float heading_deg;    // Baseline heading from GPS pair (when both valid)
    float baseline_dist;  // Distance between sensors (should be ~25cm)
    int combined_quality; // Min of both qualities
    bool both_valid;      // True if both GPS have quality >= 90
};

class DualGPS
{
public:
    DualGPS(gps &Lgps, gps &Rgps, inertial &imu, vex::distanceUnits units);
    ~DualGPS();

    // Calibration
    void calibrate();
    bool isCalibrating();

    // === PRIMARY INTERFACE: Raw GPS getters (thread-safe) ===
    // Returns true if reading was updated, copies data to output struct
    bool getLeft(GpsReading& out) const;
    bool getRight(GpsReading& out) const;
    bool getCenter(GpsCenter& out) const;  // Debug only - derived from both
    
    // === LEGACY INTERFACE (deprecated, for compatibility) ===
    // These return the center/fused values - prefer getLeft/getRight
    double xPosition();   // Returns center X if both valid, else best single
    double yPosition();   // Returns center Y if both valid, else best single
    double heading();     // Returns IMU heading (not GPS heading)
    double quality();     // Returns min quality of both sensors
    uint32_t timestamp();
    
    // Direct sensor access (use with caution - not thread-safe)
    gps &Left_GPS;
    gps &Right_GPS;
    inertial &IMU;

    // Expected baseline distance between GPS sensors
    static constexpr float EXPECTED_BASELINE_CM = 25.0f;
    static constexpr float BASELINE_TOLERANCE_CM = 5.0f;  // Allow ±5cm variance

private:
    static int updateThread(void* arg);
    vex::thread* update_thread;
    mutable vex::mutex data_mutex;
    
    void updateReadings();
    float normalizeQuality(int quality) const;
    
    vex::distanceUnits Units;
    
    // Cached raw readings (protected by mutex)
    GpsReading left_reading;
    GpsReading right_reading;
    GpsCenter center_cache;
    
    // For debug logging throttle
    uint32_t last_debug_time;
    
    // Thread control
    volatile bool running;
};

#endif // DUAL_GPS_H
