# GPS Calibration and Setup Guide

## Quick Setup Checklist

### 1. Physical Installation
- [x] GPS sensor mounted on PORT22
- [ ] GPS sensor is at the back of the robot (opposite camera)
- [ ] GPS sensor is secure and won't move during operation
- [ ] GPS sensor cables are properly connected

### 2. Measurements Needed

#### GPS Offset Measurement
**What to measure:** Distance from GPS sensor to robot center (rotation point)

1. Locate the GPS sensor on your robot
2. Locate the robot's center of rotation (typically center of drive base)
3. Measure the distance between them in millimeters
4. If GPS is BEHIND center, use NEGATIVE value
5. If GPS is AHEAD of center, use POSITIVE value

**Current Setting:** `-150.0mm` (GPS is 150mm behind robot center)

```cpp
// Update this in main.cpp if your measurement differs:
#define GPS_OFFSET_MM -150.0
```

#### Robot Dimensions (for reference)
- Wheelbase width: _____ mm
- Robot length: _____ mm
- GPS to front: _____ mm
- GPS to back: _____ mm

### 3. VEX GPS Sensor Calibration

**IMPORTANT:** Calibrate GPS before measuring field coordinates!

1. Place robot at field center (0, 0)
2. Run VEX GPS calibration routine:
   - Use VEXcode GPS calibration tool
   - OR use GPS.calibrate() in code
   - Follow on-screen instructions
3. Robot heading should match field orientation:
   - 0° = facing positive Y direction
   - 90° = facing positive X direction
   - 180° = facing negative Y direction
   - 270° = facing negative X direction
4. Verify GPS readings show (0, 0) at field center

### 4. Field Coordinate Mapping

#### Field Dimensions
**VEX AI Competition Field:** 244cm × 244cm

```
Field Layout (Top View):
     
     +Y (122cm)
          ↑
          |
-X ←------+-----→ +X
(-122cm)  |  (122cm)
          |
          ↓
     -Y (-122cm)
```

**Current Settings:**
```cpp
#define FIELD_MIN_X  -122.0  // cm
#define FIELD_MAX_X   122.0  // cm
#define FIELD_MIN_Y  -122.0  // cm
#define FIELD_MAX_Y   122.0  // cm
#define FIELD_BOUNDARY_BUFFER 15.0  // cm
```

### 5. Goal Position Measurement

**CRITICAL:** Measure these positions carefully for accurate scoring!

#### Method 1: Direct Measurement
1. Place robot at field center (GPS shows 0, 0)
2. Drive robot to scoring position in front of Loader goal
3. Note GPS X and Y coordinates from Brain screen
4. Record as `GOAL_LOADER_X` and `GOAL_LOADER_Y`
5. Repeat for Long goal

#### Method 2: Manual Measurement
1. Measure goal locations relative to field center
2. Use positive/negative coordinates based on field orientation
3. Update defines in main.cpp

**Current Goal Positions:**
```cpp
// Loader Goal (typically upper-right area)
#define GOAL_LOADER_X   110.0   // cm
#define GOAL_LOADER_Y   110.0   // cm

// Long Goal (typically upper-left area)
#define GOAL_LONG_X    -110.0   // cm
#define GOAL_LONG_Y     110.0   // cm
```

#### Goal Position Worksheet

**Loader Goal:**
- Located at field position: X = _____ cm, Y = _____ cm
- Optimal scoring distance: _____ cm (default: 30cm)
- Scoring direction (heading): _____ degrees

**Long Goal:**
- Located at field position: X = _____ cm, Y = _____ cm
- Optimal scoring distance: _____ cm (default: 30cm)
- Scoring direction (heading): _____ degrees

### 6. Verification Tests

#### Test 1: GPS Reading at Known Position
1. Place robot at field center
2. GPS should read approximately (0, 0)
3. Acceptable error: ±5cm
4. If error > 5cm, recalibrate GPS

#### Test 2: Front Position Calculation
1. Place robot at field center facing positive Y (heading 0°)
2. GPS reads (0, 0)
3. Front position should read approximately (0, 15) if GPS_OFFSET is -150mm
4. Verify offset is correct

#### Test 3: Goal Positioning
1. Use controller command `Up + Y` for Loader goal
2. Robot should drive to position 30cm in front of Loader goal
3. Robot should turn to face goal
4. Verify position is correct for scoring
5. Repeat with `Up + B` for Long goal

#### Test 4: Boundary Avoidance
1. Drive robot near field edge (e.g., X = 110cm)
2. Robot should start steering away from boundary
3. Robot should not cross FIELD_MAX_X - FIELD_BOUNDARY_BUFFER
4. Verify all four boundaries

## Calibration Data Recording

### GPS Offset Measurement
- Date measured: __________
- GPS to robot center: _______ mm
- Sign (+ or -): _______
- Final value in code: `GPS_OFFSET_MM = _______ mm`

### Field Coordinate System
- GPS calibrated at position: (_____, _____)
- Field center GPS reading: (_____, _____)
- Heading at center: _____ degrees

### Goal Positions (from field center)
| Goal | X (cm) | Y (cm) | Distance from center | Notes |
|------|--------|--------|---------------------|-------|
| Loader | _____ | _____ | _____ | _____ |
| Long | _____ | _____ | _____ | _____ |

### Boundary Test Results
| Boundary | GPS Reading at Buffer | Avoidance Active? | Notes |
|----------|----------------------|-------------------|-------|
| +X (right) | _____ | [ ] Yes [ ] No | _____ |
| -X (left) | _____ | [ ] Yes [ ] No | _____ |
| +Y (top) | _____ | [ ] Yes [ ] No | _____ |
| -Y (bottom) | _____ | [ ] Yes [ ] No | _____ |

## Troubleshooting GPS Issues

### GPS Reads (0, 0) Everywhere
- **Cause:** GPS not calibrated or not connected
- **Solution:** 
  - Check GPS connection to PORT22
  - Run GPS calibration procedure
  - Verify GPS sensor has clear view (no obstructions)

### GPS Position Jumps Around
- **Cause:** Poor GPS signal or interference
- **Solution:**
  - Ensure GPS sensor is mounted securely
  - Check for metal objects near GPS antenna
  - Recalibrate GPS sensor
  - Verify GPS firmware is up to date

### Robot Positions Wrong Distance from Goal
- **Cause:** GPS offset incorrect or goal coordinates wrong
- **Solution:**
  - Re-measure GPS offset from robot center
  - Verify GPS_OFFSET_MM sign (+ or -)
  - Re-measure goal positions on field
  - Test `getRobotFrontPosition()` calculation

### Robot Goes Off Field
- **Cause:** Boundary avoidance not working or field dimensions wrong
- **Solution:**
  - Verify FIELD_MIN/MAX values match physical field
  - Increase FIELD_BOUNDARY_BUFFER
  - Check GPS readings near boundaries
  - Test `gpsFieldBoundaryAvoidance()` function

### Front Position Seems Wrong
- **Cause:** GPS_OFFSET_MM value incorrect
- **Solution:**
  - Re-measure distance from GPS to robot center
  - Verify sign (negative if GPS behind center)
  - Test at known position with known heading
  - Calculate expected front position and compare

## Advanced Configuration

### Adjusting Scoring Distance
The robot positions 30cm in front of goals by default. To adjust:

In `gpsPositionForGoal()` function, change:
```cpp
double approachDist = 30.0;  // Change this value (cm)
```

### Adjusting Boundary Buffer
To be more/less conservative near field edges:
```cpp
#define FIELD_BOUNDARY_BUFFER 15.0  // Increase for more safety margin
```

### Adjusting Position Tolerance
When robot should consider itself "at target":
```cpp
#define GPS_POSITION_TOLERANCE 5.0  // Decrease for more precision
```

## Testing Sequence

### Basic Functionality Test
1. [ ] GPS reads position correctly at field center
2. [ ] GPS heading matches robot orientation
3. [ ] Front position calculation is correct
4. [ ] Controller commands work (Up+Y, Up+B)

### Navigation Test
5. [ ] Robot drives to Loader goal position
6. [ ] Robot turns to face Loader goal correctly
7. [ ] Robot drives to Long goal position
8. [ ] Robot turns to face Long goal correctly

### Safety Test
9. [ ] Robot avoids +X boundary (right edge)
10. [ ] Robot avoids -X boundary (left edge)
11. [ ] Robot avoids +Y boundary (top edge)
12. [ ] Robot avoids -Y boundary (bottom edge)

### Integration Test
13. [ ] GPS boundary avoidance works with camera tracking
14. [ ] GPS position display updates on Brain screen
15. [ ] Robot can collect balls and score using GPS positioning
16. [ ] No conflicts between GPS and camera-based navigation

## Support and Resources

- **VEX GPS Sensor Documentation:** https://kb.vex.com/hc/en-us/articles/360051461592
- **VEX GPS Calibration Guide:** https://kb.vex.com/hc/en-us/articles/360051605211
- **VEX V5 GPS Sensor User Manual:** Check VEX website
- **Code Documentation:** See `GPS_IMPLEMENTATION_SUMMARY.md`

---
*Calibration completed by:* __________
*Date:* __________
*Verified by:* __________
