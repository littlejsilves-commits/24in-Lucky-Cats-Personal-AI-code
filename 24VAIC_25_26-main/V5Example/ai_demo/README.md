# VEX V5 PushBack AI demo

This subdirectory contains a V5 C++ project designed to showcase how the Jetson/Raspberry Pi runs inference on game objects and sends detection data to the V5 Brain, which uses that data to play the game. 

This project contains a basic implementation to get you started and is designed to be ran on the 2025-26 Pushback HeroBot with a ball pre-loaded

When the program is ran, the robot will search for a blue ball and once found, will drive to it and intake it. The robot will then calculate which of the 4 ends of the long goals it is closest to, drive to that end, and score the ball.

## GPS Positioning Features

This robot now uses GPS positioning for precise navigation and obstacle avoidance:

### GPS Configuration
- **GPS Location**: Mounted at the back of the robot (PORT22), opposite the camera
- **GPS Offset**: Configured with -150mm offset to account for rear mounting
- The code automatically calculates the front position of the robot for accurate positioning

### GPS-Based Features

#### 1. Precise Goal Positioning
Use GPS coordinates to position the robot in front of goals for scoring:
- **Controller Commands**:
  - `Up + Y`: Position for Loader goal using GPS
  - `Up + B`: Position for Long goal using GPS
- Positions robot 30cm in front of the goal with correct orientation
- Independent of camera view - works even when goal is not visible

#### 2. Field Boundary Avoidance
- Automatically avoids field boundaries using GPS position
- Field dimensions: 244cm × 244cm (±122cm from center)
- Maintains 15cm safety buffer from edges
- Applies gentle steering correction to keep robot on field

#### 3. Enhanced Position Awareness
- Real-time GPS position displayed on Brain screen during operation
- Shows (X, Y) position in cm and heading in degrees
- Front position calculation accounts for rear-mounted GPS

### Configurable Parameters (in main.cpp)

```cpp
// Field dimensions (adjust if needed)
#define FIELD_MIN_X  -122.0  // cm
#define FIELD_MAX_X   122.0  // cm
#define FIELD_MIN_Y  -122.0  // cm
#define FIELD_MAX_Y   122.0  // cm
#define FIELD_BOUNDARY_BUFFER 15.0  // cm

// Goal positions (adjust to match your field)
#define GOAL_LOADER_X   110.0   // Loader goal X
#define GOAL_LOADER_Y   110.0   // Loader goal Y
#define GOAL_LONG_X    -110.0   // Long goal X
#define GOAL_LONG_Y     110.0   // Long goal Y

// GPS offset (adjust to match your robot)
#define GPS_OFFSET_MM -150.0  // GPS behind center
```

### GPS Functions Added

- `getRobotFrontPosition()` - Calculates front position accounting for rear GPS
- `isPositionSafe()` - Checks if position is within field boundaries
- `gpsFieldBoundaryAvoidance()` - Returns steering correction to avoid boundaries
- `gpsDistanceTo()` - Calculates distance to target position
- `gpsAngleTo()` - Calculates angle to turn to face target
- `gpsPositionForGoal()` - Positions robot for goal scoring using GPS

### Usage Tips

1. **Calibrate GPS**: Ensure GPS is properly calibrated before use
2. **Adjust Goal Positions**: Measure and update goal coordinates for your field
3. **Test Boundary Avoidance**: Verify the robot stays within the field during autonomous
4. **GPS Offset**: If your GPS mounting differs, adjust `GPS_OFFSET_MM`

#### Program Structure
- `main.cpp`- Entry point of the program. Configures devices, sets up the main callback and polls the Jetson/Raspberry Pi for data
- `ai_functions.cpp` - Contains helper functions that allow the robot to navigate the field, and find and interact with objects
- `ai_jetson.cpp` - Code that handles receiving data from the Jetson/Raspberry Pi over serial
- `ai_robot_link` - Code that handles robot-to-robot communications
- `dashboard.cpp` - Presents data and status information for Jetson/Raspberry Pi and VEXLink communications on the Brain screen

***Detailed documentation is listed below:***
        <li> <a href="https://kb.vex.com/hc/en-us/articles/360049619171-Coding-the-VEX-AI-Robot
">Coding the VEX AI Robot</a></li>
