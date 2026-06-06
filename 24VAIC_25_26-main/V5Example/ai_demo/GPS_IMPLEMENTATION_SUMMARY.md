# GPS Implementation Summary

## Overview
This document summarizes the GPS positioning system implemented for the VEX AI robot. The GPS sensor is mounted at the back of the robot (opposite the camera), and the system accounts for this offset in all positioning calculations.

## Key Features Implemented

### 1. GPS Configuration with Offset Compensation
- **Location**: GPS sensor on PORT22 at the back of the robot
- **Offset**: -150mm behind robot center (configurable via `GPS_OFFSET_MM`)
- **Front Position Calculation**: Automatically calculates the robot's front position for accurate targeting

### 2. GPS-Based Goal Positioning
New function `gpsPositionForGoal()` enables precise positioning in front of goals:
- Uses GPS coordinates instead of camera vision
- Positions robot 30cm in front of the goal
- Automatically turns to face the goal before scoring
- Works independently of camera view

**Controller Commands:**
- `Up + Y`: Position for Loader goal using GPS
- `Up + B`: Position for Long goal using GPS

### 3. Field Boundary Avoidance
New function `gpsFieldBoundaryAvoidance()` prevents robot from leaving the field:
- Monitors robot's front position against field boundaries
- Field dimensions: 244cm × 244cm centered at origin
- Safety buffer: 15cm from edges
- Applies progressive steering correction as robot approaches boundaries
- Works in parallel with camera-based obstacle avoidance

### 4. Enhanced Position Display
- Real-time GPS position (X, Y, heading) shown on Brain screen
- Displays during tracking, scanning, and positioning modes
- Shows robot's absolute field position in centimeters

## New Helper Functions

### Position Calculation
```cpp
void getRobotFrontPosition(double &frontX, double &frontY)
```
Calculates the front of the robot's position, accounting for rear-mounted GPS.

### Safety Checks
```cpp
bool isPositionSafe(double x, double y)
```
Checks if a position is within safe field boundaries.

### Field Boundary Avoidance
```cpp
double gpsFieldBoundaryAvoidance()
```
Returns steering correction to avoid field edges. Returns value between -40 and +40.

### Distance and Angle Calculation
```cpp
double gpsDistanceTo(double targetX, double targetY)
double gpsAngleTo(double targetX, double targetY)
```
Calculate distance and angle to target positions using GPS.

### Goal Positioning
```cpp
void gpsPositionForGoal(double goalX, double goalY, bool useLoader)
```
Complete goal approach and scoring sequence using GPS positioning.

## Configuration Parameters

### Field Setup (in main.cpp)
```cpp
// Field dimensions (VEX AI Competition field)
#define FIELD_MIN_X  -122.0  // cm
#define FIELD_MAX_X   122.0  // cm
#define FIELD_MIN_Y  -122.0  // cm
#define FIELD_MAX_Y   122.0  // cm
#define FIELD_BOUNDARY_BUFFER 15.0  // cm - safety margin

// Goal positions - ADJUST THESE FOR YOUR FIELD
#define GOAL_LOADER_X   110.0   // cm
#define GOAL_LOADER_Y   110.0   // cm
#define GOAL_LONG_X    -110.0   // cm
#define GOAL_LONG_Y     110.0   // cm

// GPS positioning
#define GPS_OBSTACLE_AVOID_DIST  30.0  // cm
#define GPS_POSITION_TOLERANCE    5.0  // cm
```

### Robot Setup
```cpp
// GPS offset from robot center
#define GPS_OFFSET_MM -150.0  // GPS 150mm behind center
```

## Integration with Existing Systems

### Combined Obstacle Avoidance
The `applyDrive()` function now combines three steering inputs:
1. **Target Steering**: Drives toward the target ball
2. **Camera-based Obstacle Avoidance**: Avoids detected obstacles
3. **GPS-based Boundary Avoidance**: Keeps robot within field boundaries

```cpp
void applyDrive(double baseSpeed, double targetSteer, double obstacleSteer) {
    double gpsSteer = gpsFieldBoundaryAvoidance();
    double totalSteer = targetSteer + obstacleSteer + gpsSteer;
    // ... apply to motors
}
```

### Display Updates
Screen displays now show GPS information during all operating modes:
- Position: (X, Y) in cm
- Heading: degrees (0-360)
- Distance to targets
- Boundary avoidance status

## Usage Instructions

### 1. Initial Setup
1. Mount GPS sensor on PORT22 at the back of the robot
2. Calibrate GPS using VEX GPS calibration procedure
3. Measure the distance from GPS to robot center and update `GPS_OFFSET_MM`
4. Measure goal positions on your field and update goal coordinate defines

### 2. Testing GPS Positioning
1. Enable the robot
2. Press `Up + Y` on controller to test Loader goal positioning
3. Press `Up + B` on controller to test Long goal positioning
4. Watch Brain screen for GPS position feedback
5. Verify robot positions correctly in front of goals

### 3. Field Boundary Testing
1. Run autonomous mode near field edges
2. Observe robot steering away from boundaries
3. Check Brain screen for GPS position near boundaries
4. Adjust `FIELD_BOUNDARY_BUFFER` if needed

### 4. Tuning Parameters

**If robot goes off field:**
- Increase `FIELD_BOUNDARY_BUFFER` (default: 15.0cm)
- Verify `FIELD_MIN/MAX` values match your field

**If goal positioning is inaccurate:**
- Re-measure goal coordinates on field
- Update `GOAL_LOADER_X/Y` and `GOAL_LONG_X/Y`
- Verify GPS calibration

**If robot front position seems wrong:**
- Measure GPS offset from robot center
- Update `GPS_OFFSET_MM` (negative if GPS is behind center)

## Benefits of GPS Integration

1. **Independent Goal Positioning**: Can score even when goals are not visible to camera
2. **Field Awareness**: Robot always knows its position on the field
3. **Boundary Safety**: Prevents robot from leaving the field
4. **Backup Navigation**: Provides positioning data when camera-based navigation fails
5. **Precise Positioning**: GPS provides absolute position, reducing cumulative error

## Future Enhancements

Potential improvements:
1. GPS-based path planning to avoid known obstacles
2. Return-to-starting-position feature
3. GPS-assisted object searching (spiral patterns, grid search)
4. Multi-robot coordination using shared GPS positions
5. Field zone awareness (safe zones, scoring zones)
6. GPS-based speed limiting near boundaries

## Troubleshooting

### GPS Shows Wrong Position
- Recalibrate GPS sensor
- Check GPS sensor connection to PORT22
- Verify field coordinate system matches GPS coordinate system

### Robot Still Goes Off Field
- Increase `FIELD_BOUNDARY_BUFFER`
- Check that `FIELD_MIN/MAX` values are correct
- Verify GPS readings are accurate near boundaries

### Goal Positioning Misses Goal
- Re-measure goal positions on field
- Check GPS calibration
- Verify `GPS_OFFSET_MM` is correct for your robot
- Adjust approach distance (default 30cm in `gpsPositionForGoal()`)

### Front Position Calculation Wrong
- Measure distance from GPS to robot center
- Update `GPS_OFFSET_MM` (use negative value if GPS is behind center)
- Check robot heading is calibrated correctly

## Testing Checklist

- [ ] GPS sensor connected to PORT22
- [ ] GPS calibrated on field
- [ ] `GPS_OFFSET_MM` measured and configured
- [ ] Field dimensions (`FIELD_MIN/MAX`) verified
- [ ] Goal positions measured and configured
- [ ] Test `Up + Y` (Loader goal positioning)
- [ ] Test `Up + B` (Long goal positioning)
- [ ] Verify boundary avoidance near field edges
- [ ] Check GPS position display on Brain screen
- [ ] Test combined camera + GPS navigation

## Code Files Modified

1. **main.cpp**
   - Added GPS configuration with offset
   - Added GPS-based positioning parameters
   - Added GPS helper functions
   - Added `gpsPositionForGoal()` function
   - Updated `applyDrive()` to include GPS boundary avoidance
   - Added controller commands for GPS goal positioning
   - Enhanced screen displays with GPS information

2. **README.md**
   - Added GPS features documentation
   - Added configuration guide
   - Added usage instructions

## Contact & Support

For questions about this GPS implementation:
- Review the inline comments in `main.cpp`
- Check the configuration parameters section
- Refer to VEX GPS documentation: https://kb.vex.com/hc/en-us/articles/360051461592

---
*Last Updated: June 6, 2026*
