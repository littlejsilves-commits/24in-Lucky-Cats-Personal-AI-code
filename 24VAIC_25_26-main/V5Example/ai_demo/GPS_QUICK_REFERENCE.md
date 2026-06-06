# GPS System Quick Reference

## Controller Commands

| Button Combo | Action | Description |
|--------------|--------|-------------|
| **Down + A** | Skills Autonomous | Run pre-programmed skills routine |
| **Up + Y** | GPS Loader Goal | Position for Loader goal using GPS |
| **Up + B** | GPS Long Goal | Position for Long goal using GPS |

## Key Configuration Parameters

### GPS Hardware
```cpp
// In main.cpp, line ~50
gps GPS = gps(PORT22, 0, GPS_OFFSET_MM, distanceUnits::mm, 0);
```
- **Port:** 22
- **Offset:** -150mm (GPS behind robot center)
- **Units:** millimeters

### Field Boundaries
```cpp
#define FIELD_MIN_X  -122.0  // cm - Left edge
#define FIELD_MAX_X   122.0  // cm - Right edge
#define FIELD_MIN_Y  -122.0  // cm - Bottom edge
#define FIELD_MAX_Y   122.0  // cm - Top edge
#define FIELD_BOUNDARY_BUFFER 15.0  // cm - Safety margin
```

### Goal Positions
```cpp
#define GOAL_LOADER_X   110.0   // cm
#define GOAL_LOADER_Y   110.0   // cm
#define GOAL_LONG_X    -110.0   // cm
#define GOAL_LONG_Y     110.0   // cm
```

### GPS Positioning
```cpp
#define GPS_OFFSET_MM -150.0  // mm - GPS sensor offset from center
#define GPS_OBSTACLE_AVOID_DIST  30.0  // cm
#define GPS_POSITION_TOLERANCE    5.0  // cm
```

## Brain Screen Information

### During Ball Tracking
```
Line 1: TRACKING [blocks]/[total]  [dist]m
Line 2: GPS:(X,Y) H:[heading]°
Line 3: conf:[hits]/[window]  miss:[count]
Line 5: AVOIDING steer:[value] (if active)
```

### During Scanning
```
Line 1: SCANNING... [blocks]/[total]
Line 2: GPS:(X,Y) H:[heading]°
Line 3: conf:[hits]/[window]  need:[threshold]
Line 4: Rotating RIGHT/LEFT
```

### During GPS Goal Positioning
```
Line 1: GPS POSITIONING FOR GOAL
Line 2: Goal: (X,Y)
Line 3: Target: (X,Y)
Line 4: Dist: [distance] cm
Line 5: Pos: (X,Y)
Line 6: Turning to [angle] deg
Line 7: SCORING!
```

## GPS Functions Quick Reference

### Get Front Position
```cpp
double frontX, frontY;
getRobotFrontPosition(frontX, frontY);
// Returns front position accounting for rear GPS
```

### Check Position Safety
```cpp
if (isPositionSafe(x, y)) {
    // Position is within field boundaries
}
```

### Field Boundary Avoidance
```cpp
double steer = gpsFieldBoundaryAvoidance();
// Returns steering correction: -40.0 to +40.0
```

### Distance to Target
```cpp
double dist = gpsDistanceTo(targetX, targetY);
// Returns distance in cm
```

### Angle to Target
```cpp
double angle = gpsAngleTo(targetX, targetY);
// Returns angle in degrees (0-360)
```

### Position for Goal
```cpp
gpsPositionForGoal(GOAL_LOADER_X, GOAL_LOADER_Y, true);
// Positions robot and scores
// Last param: true=Loader, false=Long
```

## Coordinate System

```
          +Y (North)
             ↑
             |
             |
-X ←---------+--------→ +X
(West)    (0,0)     (East)
             |
             |
             ↓
          -Y (South)
```

- **Origin (0,0):** Field center
- **X-axis:** Left (-) to Right (+)
- **Y-axis:** Bottom (-) to Top (+)
- **Heading:** 0° = North, 90° = East, 180° = South, 270° = West

## Common Tasks

### Change Goal Positions
1. Measure goal location from field center
2. Update in main.cpp:
   ```cpp
   #define GOAL_LOADER_X   [your_X]   // cm
   #define GOAL_LOADER_Y   [your_Y]   // cm
   ```
3. Rebuild and upload

### Adjust GPS Offset
1. Measure GPS to robot center (mm)
2. Update in main.cpp:
   ```cpp
   #define GPS_OFFSET_MM [your_offset]
   // Negative if GPS is behind center
   ```
3. Rebuild and upload

### Change Boundary Safety Margin
1. Update in main.cpp:
   ```cpp
   #define FIELD_BOUNDARY_BUFFER [size]  // cm
   ```
2. Larger value = more safety margin
3. Rebuild and upload

### Adjust Scoring Distance
1. Find `gpsPositionForGoal()` function
2. Change line:
   ```cpp
   double approachDist = 30.0;  // cm
   ```
3. Rebuild and upload

## Troubleshooting Quick Fixes

| Problem | Quick Fix |
|---------|-----------|
| GPS shows (0,0) always | Recalibrate GPS sensor |
| Robot goes off field | Increase FIELD_BOUNDARY_BUFFER |
| Wrong goal position | Re-measure goal X,Y coordinates |
| Front position wrong | Check GPS_OFFSET_MM sign and value |
| Jerky boundary avoidance | Decrease GPS_OBSTACLE_AVOID_DIST |
| Not reaching target | Increase GPS_POSITION_TOLERANCE |

## Typical Configuration Values

### Conservative (Safe)
```cpp
#define FIELD_BOUNDARY_BUFFER 20.0  // cm
#define GPS_POSITION_TOLERANCE  8.0  // cm
#define GPS_OBSTACLE_AVOID_DIST 40.0  // cm
```

### Balanced (Default)
```cpp
#define FIELD_BOUNDARY_BUFFER 15.0  // cm
#define GPS_POSITION_TOLERANCE  5.0  // cm
#define GPS_OBSTACLE_AVOID_DIST 30.0  // cm
```

### Aggressive (Precise)
```cpp
#define FIELD_BOUNDARY_BUFFER 10.0  // cm
#define GPS_POSITION_TOLERANCE  3.0  // cm
#define GPS_OBSTACLE_AVOID_DIST 20.0  // cm
```

## Pre-Match Checklist

- [ ] GPS sensor connected to PORT22
- [ ] GPS calibrated at field center
- [ ] Goal positions measured and configured
- [ ] GPS_OFFSET_MM matches robot build
- [ ] Test controller commands (Up+Y, Up+B)
- [ ] Verify GPS readings on Brain screen
- [ ] Test boundary avoidance near edges
- [ ] Confirm robot faces goals correctly

## Match Strategy Tips

1. **Use GPS for Goal Scoring:** More reliable than camera-only
2. **Monitor GPS Position:** Watch Brain screen during autonomous
3. **Trust Boundary Avoidance:** GPS will keep robot on field
4. **Combined Navigation:** GPS + camera = best results
5. **Pre-Position:** Use GPS to position before vision-based scoring

## Constants Location

All GPS-related constants are in `main.cpp` between lines ~50-95:
- GPS sensor configuration
- Field dimensions
- Goal positions
- GPS positioning parameters
- Obstacle avoidance settings

## Function Location

GPS functions are in `main.cpp` after line ~200:
- `getRobotFrontPosition()` - Front position calculation
- `isPositionSafe()` - Boundary check
- `gpsFieldBoundaryAvoidance()` - Steering correction
- `gpsDistanceTo()` - Distance calculation
- `gpsAngleTo()` - Angle calculation
- `gpsPositionForGoal()` - Complete goal positioning

---
**For detailed documentation:** See `GPS_IMPLEMENTATION_SUMMARY.md`
**For calibration guide:** See `GPS_CALIBRATION_GUIDE.md`
