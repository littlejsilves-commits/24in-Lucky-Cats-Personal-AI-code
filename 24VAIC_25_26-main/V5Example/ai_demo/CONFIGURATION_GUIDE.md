# Robot Configuration Guide

All configurable parameters are now consolidated at the top of `main.cpp` (lines 16-100) for easy access.

## 🎯 Most Common Adjustments

### 1. Robot Speed
```cpp
#define DRIVE_SPEED_PCT  10.0  // Change this one value!
```
- **Testing:** 10-25%
- **Competition:** 75-100%
- All other speeds automatically scale from this value

### 2. Target Selection
```cpp
int TARGET_CLASSID = BALL_BLUE_ID;  // Which ball to chase
```
- `BALL_BLUE_ID` - Chase blue balls
- `BALL_RED_ID` - Chase red balls
- `-1` - Chase any ball

### 3. Goal Positions (GPS)
```cpp
#define GOAL_LOADER_X   110.0   // cm
#define GOAL_LOADER_Y   110.0   // cm
#define GOAL_LONG_X    -110.0   // cm
#define GOAL_LONG_Y     110.0   // cm
```
**⚠️ Measure these on your field!**

### 4. GPS Sensor Offset
```cpp
#define GPS_OFFSET_MM  -150.0  // Negative = GPS behind center
```
Measure distance from GPS to robot center.

---

## 📋 Complete Parameter Reference

### Hardware Configuration
| Parameter | Default | Description |
|-----------|---------|-------------|
| `GPS_OFFSET_MM` | -150.0 | GPS sensor offset from robot center (mm) |
| `MANAGER_ROBOT` | 1 | 1 = Manager robot, 0 = Worker robot |

### Target Selection
| Parameter | Default | Description |
|-----------|---------|-------------|
| `TARGET_CLASSID` | BALL_BLUE_ID | Which ball to chase |
| `BLOCKS_TO_COLLECT` | 3 | Blocks needed before going to goal |

### Speed Settings
| Parameter | Default | Description |
|-----------|---------|-------------|
| `DRIVE_SPEED_PCT` | 10.0 | **Master speed** - all others derive from this |
| `BASE_DRIVE_SPEED` | 10.0 | Normal driving (= DRIVE_SPEED_PCT) |
| `NEAR_DRIVE_SPEED` | 8.0 | Near target (80% of master) |
| `COLLECT_DRIVE_SPEED` | 8.0 | Collecting blocks (80% of master) |
| `SEARCH_SPIN_SPEED` | 8.0 | Rotating to search (80% of master) |
| `GOAL_APPROACH_SPEED` | 10.0 | GPS goal approach (= DRIVE_SPEED_PCT) |

### Camera & Detection
| Parameter | Default | Description |
|-----------|---------|-------------|
| `CAMERA_MIN_DIST` | 0.10 m | Camera loses target below this |
| `BLIND_APPROACH_DIST` | 0.20 m | Distance to drive blind |
| `BLIND_APPROACH_SPEED_MPS` | 0.40 m/s | Robot speed at COLLECT_DRIVE_SPEED |
| `CONFIRM_WINDOW` | 10 frames | Detection confirmation window |
| `LOSE_THRESHOLD` | 5 frames | Consecutive misses to drop target |
| `ACQUIRE_DELAY_MS` | 500 ms | Hold still when first detecting |

### Obstacle Avoidance
| Parameter | Default | Description |
|-----------|---------|-------------|
| `OBSTACLE_AVOID_DIST` | 0.2 m | Start avoiding at this distance |
| `OBSTACLE_AVOID_GAIN` | 80.0 | Steering strength (higher = sharper) |

### Collection Timing
| Parameter | Default | Description |
|-----------|---------|-------------|
| `COLLECTION_TIME` | 3.0 sec | How long to run intake after reaching block |

### Field Dimensions (GPS)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `FIELD_MIN_X` | -122.0 cm | Left boundary |
| `FIELD_MAX_X` | 122.0 cm | Right boundary |
| `FIELD_MIN_Y` | -122.0 cm | Bottom boundary |
| `FIELD_MAX_Y` | 122.0 cm | Top boundary |
| `FIELD_BOUNDARY_BUFFER` | 15.0 cm | Safety margin from edges |

### Goal Positions (GPS)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `GOAL_LOADER_X` | 110.0 cm | Loader goal X position |
| `GOAL_LOADER_Y` | 110.0 cm | Loader goal Y position |
| `GOAL_LONG_X` | -110.0 cm | Long goal X position |
| `GOAL_LONG_Y` | 110.0 cm | Long goal Y position |

### GPS Positioning
| Parameter | Default | Description |
|-----------|---------|-------------|
| `GPS_OBSTACLE_AVOID_DIST` | 30.0 cm | Avoid boundaries at this distance |
| `GPS_POSITION_TOLERANCE` | 5.0 cm | Acceptable positioning error |

### Goal Detection
| Parameter | Default | Description |
|-----------|---------|-------------|
| `GOALS_ENABLED` | 0 | Set to 1 when goal detection ready |

---

## 🔧 Common Tuning Scenarios

### Scenario 1: Robot Too Slow
**Problem:** Robot moves too slowly during competition

**Solution:**
```cpp
#define DRIVE_SPEED_PCT  75.0  // Increase from 10 to 75
```

### Scenario 2: Robot Too Fast / Unstable
**Problem:** Robot overshoots targets or loses tracking

**Solution 1** - Reduce overall speed:
```cpp
#define DRIVE_SPEED_PCT  50.0  // Reduce speed
```

**Solution 2** - Increase near-target slowdown:
```cpp
#define NEAR_DRIVE_SPEED (DRIVE_SPEED_PCT * 0.60)  // Slower near targets (60%)
```

### Scenario 3: Wrong Goal Position
**Problem:** Robot positions incorrectly for scoring

**Solution:**
1. Drive robot to correct scoring position manually
2. Note GPS coordinates from Brain screen
3. Update goal position in code:
```cpp
#define GOAL_LOADER_X   [GPS_X_reading]
#define GOAL_LOADER_Y   [GPS_Y_reading]
```

### Scenario 4: Robot Goes Off Field
**Problem:** Robot drives outside field boundaries

**Solution 1** - Increase safety buffer:
```cpp
#define FIELD_BOUNDARY_BUFFER 25.0  // Increase from 15 to 25
```

**Solution 2** - Check field dimensions:
```cpp
// Verify these match your actual field
#define FIELD_MAX_X   122.0  // Check this is correct
#define FIELD_MAX_Y   122.0  // Check this is correct
```

### Scenario 5: False Target Detection
**Problem:** Robot chases non-existent targets

**Solution:** Increase confirmation threshold:
```cpp
// In confirmThresholdForClass() function
case BALL_BLUE_ID: return 8;  // Increase from 6 to 8
```

### Scenario 6: Slow Target Acquisition
**Problem:** Robot takes too long to start chasing target

**Solution 1** - Reduce acquire delay:
```cpp
#define ACQUIRE_DELAY_MS  200  // Reduce from 500 to 200
```

**Solution 2** - Reduce confirmation threshold:
```cpp
// In confirmThresholdForClass() function
case BALL_BLUE_ID: return 4;  // Reduce from 6 to 4
```

### Scenario 7: Obstacle Avoidance Too Aggressive
**Problem:** Robot avoids obstacles too early or too hard

**Solution:**
```cpp
#define OBSTACLE_AVOID_DIST  0.15  // Reduce from 0.2 to 0.15
#define OBSTACLE_AVOID_GAIN  50.0  // Reduce from 80 to 50
```

---

## 📍 Configuration File Location

**File:** `V5Example/ai_demo/src/main.cpp`

**Lines:** 16-100 (configuration section)

**Sections:**
1. Hardware Configuration (lines 22-24)
2. Target Selection (lines 26-34)
3. Speed Settings (lines 36-44)
4. Camera & Detection (lines 46-67)
5. Obstacle Avoidance (lines 69-71)
6. Collection Timing (line 73-74)
7. Field Dimensions (lines 76-82)
8. Goal Positions (lines 84-89)
9. GPS Positioning (lines 91-93)
10. Goal Detection (lines 95-96)

---

## 🎓 Understanding Speed Calculations

### Master Speed Control
Everything scales from `DRIVE_SPEED_PCT`:

```
DRIVE_SPEED_PCT = 10%
    ↓
BASE_DRIVE_SPEED = 10%
NEAR_DRIVE_SPEED = 8% (80%)
COLLECT_DRIVE_SPEED = 8% (80%)
SEARCH_SPIN_SPEED = 8% (80%)
GOAL_APPROACH_SPEED = 10%
```

### Dynamic Speed Ramping
When distance < 1.0m, speed ramps from NEAR to BASE:

```
Distance 1.0m → Speed 10% (BASE)
Distance 0.5m → Speed 9%  (ramping)
Distance 0.1m → Speed 8.2% (near NEAR)
Distance 0.0m → Speed 8%  (NEAR)
```

Formula: `NEAR_SPEED + (distance × (BASE_SPEED - NEAR_SPEED))`

---

## ✅ Pre-Competition Checklist

- [ ] Set `DRIVE_SPEED_PCT` to competition speed (75-100%)
- [ ] Set `TARGET_CLASSID` to correct alliance color
- [ ] Measure and update `GOAL_LOADER_X/Y` coordinates
- [ ] Measure and update `GOAL_LONG_X/Y` coordinates
- [ ] Verify `GPS_OFFSET_MM` matches robot build
- [ ] Calibrate GPS sensor on field
- [ ] Test GPS goal positioning (Up+Y and Up+B)
- [ ] Verify field boundary avoidance works
- [ ] Test target acquisition and tracking
- [ ] Confirm collection timing is appropriate

---

## 🆘 Quick Troubleshooting

| Problem | Parameter to Check | Typical Fix |
|---------|-------------------|-------------|
| Robot too slow | `DRIVE_SPEED_PCT` | Increase to 75-100 |
| Robot too fast | `DRIVE_SPEED_PCT` | Reduce to 50-60 |
| Wrong goal position | `GOAL_LOADER_X/Y`, `GOAL_LONG_X/Y` | Measure on field |
| Goes off field | `FIELD_BOUNDARY_BUFFER` | Increase to 25 |
| False detections | `confirmThresholdForClass()` | Increase threshold |
| Slow to acquire | `ACQUIRE_DELAY_MS` | Reduce to 200-300 |
| Misses targets | `LOSE_THRESHOLD` | Increase to 8-10 |
| GPS wrong position | `GPS_OFFSET_MM` | Re-measure offset |

---

## 📚 Additional Resources

- **GPS Setup:** See `GPS_CALIBRATION_GUIDE.md`
- **GPS Features:** See `GPS_IMPLEMENTATION_SUMMARY.md`
- **Quick Reference:** See `GPS_QUICK_REFERENCE.md`
- **Main README:** See `README.md`

---

*All parameters can be found in `main.cpp` lines 16-100*
