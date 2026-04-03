/*----------------------------------------------------------------------------*/
/*    DualGPS.cpp - Refactored Dual GPS sensor interface                      */
/*    Fixed mutex handling, exposes raw readings for Localizer EKF            */
/*----------------------------------------------------------------------------*/

#include "robot-config.h"
#include "DualGPS.h"
#include <cmath>

// Constructor - starts background update thread
DualGPS::DualGPS(gps &Lgps, gps &Rgps, inertial &imu, vex::distanceUnits units)
    : Left_GPS(Lgps),
      Right_GPS(Rgps), 
      IMU(imu),
      Units(units),
      last_debug_time(0),
      running(true)
{
    // Initialize readings to invalid state
    left_reading = {0, 0, 0, 0, 0, false};
    right_reading = {0, 0, 0, 0, 0, false};
    center_cache = {0, 0, 0, 0, 0, false};
    
    // Start background update thread (~60Hz)
    update_thread = new vex::thread(updateThread, static_cast<void*>(this));
}

DualGPS::~DualGPS()
{
    running = false;
    if (update_thread != nullptr) 
    {
        update_thread->interrupt();
        vex::this_thread::sleep_for(50);  // Allow thread to exit
        delete update_thread;
        update_thread = nullptr;
    }
}

int DualGPS::updateThread(void* arg) 
{
    DualGPS* instance = static_cast<DualGPS*>(arg);
    while (instance->running) 
    {
        instance->updateReadings();
        vex::this_thread::sleep_for(16);  // ~60Hz
    }
    return 0;
}

// Main update function - reads both GPS sensors and caches values
// FIXED: Proper mutex handling with single lock/unlock pair
void DualGPS::updateReadings()
{
    // Read all sensor values BEFORE acquiring mutex (non-blocking reads)
    float leftX = Left_GPS.xPosition(Units);
    float leftY = Left_GPS.yPosition(Units);
    float leftH = Left_GPS.heading();
    int leftQual = Left_GPS.quality();
    uint32_t leftTs = Left_GPS.timestamp();
    
    float rightX = Right_GPS.xPosition(Units);
    float rightY = Right_GPS.yPosition(Units);
    float rightH = Right_GPS.heading();
    int rightQual = Right_GPS.quality();
    uint32_t rightTs = Right_GPS.timestamp();
    
    // Check for valid readings (not NaN)
    bool leftValid = !std::isnan(leftX) && !std::isnan(leftY) && !std::isnan(leftH) && leftQual > 0;
    bool rightValid = !std::isnan(rightX) && !std::isnan(rightY) && !std::isnan(rightH) && rightQual > 0;
    
    // Prepare center calculation (outside mutex)
    float centerX = 0, centerY = 0, baselineHeading = 0, baselineDist = 0;
    int combinedQual = 0;
    bool bothValid = leftValid && rightValid && leftQual >= 90 && rightQual >= 90;
    
    if (bothValid) {
        centerX = (leftX + rightX) / 2.0f;
        centerY = (leftY + rightY) / 2.0f;
        combinedQual = (leftQual < rightQual) ? leftQual : rightQual;
        
        // Calculate baseline vector and heading
        float dx = rightX - leftX;  // RGPS is to the LEFT of LGPS in robot frame
        float dy = rightY - leftY;
        baselineDist = std::sqrt(dx * dx + dy * dy);
        
        // Baseline heading: perpendicular to the line between sensors
        // atan2(dy, dx) gives angle of baseline vector
        // Robot heading is 90° CCW from baseline (robot faces forward, GPS are side-by-side)
        float baselineAngleRad = std::atan2(dy, dx);
        // Convert to VEX heading convention (0=North, CW positive)
        // Math angle: 0=East, CCW positive
        // VEX heading = 90 - math_angle_deg (then normalize)
        float baselineAngleDeg = baselineAngleRad * 180.0f / M_PI;
        baselineHeading = 90.0f - baselineAngleDeg + 90.0f;  // +90 because robot faces perpendicular to baseline
        while (baselineHeading < 0) baselineHeading += 360.0f;
        while (baselineHeading >= 360.0f) baselineHeading -= 360.0f;
    }
    
    // === CRITICAL SECTION: Single lock/unlock pair ===
    data_mutex.lock();
    
    // Update left reading
    left_reading.x_cm = leftX;
    left_reading.y_cm = leftY;
    left_reading.heading_deg = leftH;
    left_reading.quality = leftQual;
    left_reading.timestamp = leftTs;
    left_reading.valid = leftValid;
    
    // Update right reading
    right_reading.x_cm = rightX;
    right_reading.y_cm = rightY;
    right_reading.heading_deg = rightH;
    right_reading.quality = rightQual;
    right_reading.timestamp = rightTs;
    right_reading.valid = rightValid;
    
    // Update center cache
    center_cache.x_cm = centerX;
    center_cache.y_cm = centerY;
    center_cache.heading_deg = baselineHeading;
    center_cache.baseline_dist = baselineDist;
    center_cache.combined_quality = combinedQual;
    center_cache.both_valid = bothValid;
    
    data_mutex.unlock();
    // === END CRITICAL SECTION ===
    
    // Debug logging (outside mutex, throttled to 1Hz)
    #if DUALGPS_DEBUG
    uint32_t now = Brain.Timer.system();
    if (now - last_debug_time >= 1000) {
        last_debug_time = now;
        printf("DualGPS: L(%d)=(%.1f,%.1f) R(%d)=(%.1f,%.1f) base=%.1fcm\n",
               leftQual, leftX, leftY, rightQual, rightX, rightY, baselineDist);
    }
    #endif
}

// Quality normalization (kept for compatibility, but Localizer uses raw quality)
float DualGPS::normalizeQuality(int quality) const
{
    return (quality >= 90) ? (quality - 90.0f) / 10.0f : 0.0f;
}

// === Thread-safe getters for raw readings ===

bool DualGPS::getLeft(GpsReading& out) const
{
    data_mutex.lock();
    out = left_reading;
    data_mutex.unlock();
    return out.valid;
}

bool DualGPS::getRight(GpsReading& out) const
{
    data_mutex.lock();
    out = right_reading;
    data_mutex.unlock();
    return out.valid;
}

bool DualGPS::getCenter(GpsCenter& out) const
{
    data_mutex.lock();
    out = center_cache;
    data_mutex.unlock();
    return out.both_valid;
}

// === Legacy interface (for backward compatibility) ===

double DualGPS::xPosition()
{
    data_mutex.lock();
    double result;
    if (center_cache.both_valid) {
        result = center_cache.x_cm;
    } else if (left_reading.valid && left_reading.quality >= 90) {
        result = left_reading.x_cm;
    } else if (right_reading.valid && right_reading.quality >= 90) {
        result = right_reading.x_cm;
    } else {
        result = 0.0;
    }
    data_mutex.unlock();
    return result;
}

double DualGPS::yPosition()
{
    data_mutex.lock();
    double result;
    if (center_cache.both_valid) {
        result = center_cache.y_cm;
    } else if (left_reading.valid && left_reading.quality >= 90) {
        result = left_reading.y_cm;
    } else if (right_reading.valid && right_reading.quality >= 90) {
        result = right_reading.y_cm;
    } else {
        result = 0.0;
    }
    data_mutex.unlock();
    return result;
}

double DualGPS::heading()
{
    // Return IMU heading, not GPS heading (more reliable)
    return IMU.heading(degrees);
}

double DualGPS::quality()
{
    data_mutex.lock();
    double result = center_cache.combined_quality;
    if (!center_cache.both_valid) {
        result = (left_reading.quality > right_reading.quality) ? left_reading.quality : right_reading.quality;
    }
    data_mutex.unlock();
    return result;
}

uint32_t DualGPS::timestamp()
{
    data_mutex.lock();
    uint32_t result = (left_reading.timestamp > right_reading.timestamp) ? left_reading.timestamp : right_reading.timestamp;
    data_mutex.unlock();
    return result;
}

void DualGPS::calibrate() 
{
    Left_GPS.calibrate();
    Right_GPS.calibrate();
}

bool DualGPS::isCalibrating() 
{
    return Left_GPS.isCalibrating() || Right_GPS.isCalibrating();
}
