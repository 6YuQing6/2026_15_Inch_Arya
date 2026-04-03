# Localizer EKF Tuning Guide

## Overview

The Localizer uses an Extended Kalman Filter (EKF) to fuse:
- **Tracking wheel odometry** (100-200Hz, high frequency, drifts over time)
- **IMU heading** (authoritative for rotation)
- **Dual GPS sensors** (60Hz, absolute position, noisy near walls)

## Key Tuning Parameters

Access these via the global `localizer` object:

```cpp
// Process noise (prediction uncertainty)
localizer.Q_xy = 0.5f;       // Position noise per cycle (cm²) - higher = trust odom less
localizer.Q_theta = 0.001f;  // Heading noise per cycle (rad²)

// GPS measurement noise
localizer.sigma_base_cm = 3.0f;       // Sigma at quality 100
localizer.sigma_quality_90_cm = 20.0f; // Sigma at quality 90
localizer.sigma_min_quality = 85;      // Ignore GPS below this quality

// Wall proximity effect
localizer.wall_threshold_cm = 37.88f;  // Wall effect starts here
localizer.wall_multiplier = 10.0f;     // Sigma multiplier at wall

// Outlier rejection
localizer.gate_sigma_mult = 5.0f;      // Gate at N sigma
localizer.gate_max_cm = 80.0f;         // Hard gate distance
```

## Tuning Process

### 1. Start with Default Values

The defaults should work reasonably well. Run `testLocalization()` (R1 button) and observe behavior.

### 2. Check GPS Quality Near Walls

If GPS is being used too aggressively near walls (causing jumps):
- **Increase `wall_multiplier`** (try 15-20)
- **Decrease `wall_threshold_cm`** to apply effect earlier (try 45-50cm)

### 3. Check GPS Quality in Center Field

If position drifts too much in center (not enough GPS correction):
- **Decrease `sigma_base_cm`** (try 2.0)
- **Decrease `Q_xy`** to trust predictions more between GPS updates

### 4. Check for Sudden Jumps

If pose jumps when GPS quality changes:
- **Increase `sigma_quality_90_cm`** to make low-quality readings weaker
- **Decrease `gate_sigma_mult`** to reject more outliers (try 3.0)
- **Check if one GPS has consistently bad quality** - may need physical adjustment

### 5. Check Heading Drift

The IMU is authoritative for heading. If heading drifts:
- Ensure IMU is calibrated at startup
- Consider adding GPS baseline heading correction (currently not enabled)

## Debug Output

### CSV Logging Format
Run `testLocalization()` to get CSV output:
```
time_ms,x,y,theta,left_x,left_y,left_q,right_x,right_y,right_q,dWall,sigmaL,sigmaR,usedL,usedR,gatedL,gatedR,sigma_x,sigma_y
```

### Key Metrics to Watch

| Metric | Good Range | Indicates |
|--------|------------|-----------|
| `sigma_x, sigma_y` | 2-10 cm | Position uncertainty |
| `dWall` | >40 cm | Safe from wall effects |
| `usedL, usedR` | 1,1 in center | Both GPS contributing |
| `gatedL, gatedR` | 0,0 | No outliers rejected |
| `sigmaL, sigmaR` | 3-10 cm | GPS measurement trust |

### Real-time Status
Press R2 to show live localizer status on controller.

## Common Issues

### Issue: Position jumps when one GPS drops out
**Solution:** The EKF should handle this smoothly. If not:
- Increase `sigma_quality_90_cm` to reduce trust in marginal readings
- The transition should be gradual, not sudden

### Issue: Robot gets "lost" near walls
**Solution:** 
- This is expected - GPS is unreliable near walls
- Odometry takes over - ensure tracking wheels are accurate
- Run quick patterns, return to center for GPS correction

### Issue: Constant oscillation in position
**Solution:**
- Reduce `Q_xy` (trust predictions more)
- Check if both GPS have similar readings (if not, one may be miscalibrated)

### Issue: Slow convergence at startup
**Solution:**
- Use `initWithStartPose()` for known start positions (competition)
- Or increase `sigma_base_cm` temporarily for faster initial convergence

## Recommended Starting Values

```cpp
// Conservative (smooth, may drift slightly)
localizer.Q_xy = 0.3f;
localizer.Q_theta = 0.0005f;
localizer.sigma_base_cm = 4.0f;
localizer.sigma_quality_90_cm = 25.0f;
localizer.wall_multiplier = 12.0f;
localizer.gate_sigma_mult = 4.0f;

// Aggressive (responsive, may be jumpy)
localizer.Q_xy = 1.0f;
localizer.Q_theta = 0.002f;
localizer.sigma_base_cm = 2.0f;
localizer.sigma_quality_90_cm = 15.0f;
localizer.wall_multiplier = 8.0f;
localizer.gate_sigma_mult = 6.0f;
```

## Button Mapping

| Button | Function |
|--------|----------|
| R1 | Run localization test (scripted motions + CSV logging) |
| R2 | Show live localizer status |
| L1 | Multi-point path test |

## API Reference

```cpp
// Get current pose
Pose pose = localizer.getPose();
// pose.x_cm, pose.y_cm, pose.heading_deg, pose.timestamp_ms

// Reset to known position (competition start)
localizer.initWithStartPose(x_cm, y_cm, heading_deg);

// Initialize from GPS (waits for good quality)
bool success = localizer.initFromGPS(timeout_ms);

// Get debug info
LocalizerDebug dbg = localizer.getDebug();
// dbg.sigma_x_cm, dbg.left_gps_used, dbg.wall_distance_cm, etc.
```
