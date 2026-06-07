/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Lucky cATS                                                */
/*    Created:      05/24/2026                                                */
/*    Description:  V5 project with GPS positioning                           */
/*                                                                            */
/*    📋 QUICK START GUIDE:                                                   */
/*       All configurable parameters are in the section below (lines 16-100)  */
/*       Key settings to adjust:                                              */
/*         - DRIVE_SPEED_PCT: Robot speed (10 = testing, 75 = competition)   */
/*         - TARGET_CLASSID: Which ball to chase (BALL_BLUE_ID or BALL_RED_ID)*/
/*         - GOAL_LOADER_X/Y & GOAL_LONG_X/Y: Goal positions on your field   */
/*         - GPS_OFFSET_MM: Distance from GPS sensor to robot center          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "ai_functions.h"
#include "skills_autonomous.h"
#include <cmath>

using namespace vex;

/*============================================================================*/
/*                      CONFIGURABLE PARAMETERS                               */
/*         🔧 Adjust these values to tune robot behavior 🔧                   */
/*============================================================================*/

// ===== DIAGNOSTICS & LOGGING =====
#define ENABLE_SERIAL_LOGGING   1   // 1 = Log to serial console (view in VEXcode terminal), 0 = Disable
#define ENABLE_CONTROLLER_FEEDBACK 1 // 1 = Controller rumble on events, 0 = Disable
#define LOG_INTERVAL_MS     500     // Log every 500ms (don't flood console)

// ===== HARDWARE CONFIGURATION =====
#define GPS_OFFSET_MM -150.0  // GPS sensor offset from robot center (negative = behind center)
#define MANAGER_ROBOT 1       // 1 = Manager robot, 0 = Worker robot

// ===== TARGET SELECTION =====
// Class IDs from Jetson model (must match src/lib/types.ts):
//   0 = BallBlue, 1 = BallBlueInGoal, 2 = BallRed, 3 = BallRedInGoal
#define BALL_BLUE_ID          0
#define BALL_BLUE_IN_GOAL_ID  1
#define BALL_RED_ID           2
#define BALL_RED_IN_GOAL_ID   3

int TARGET_CLASSID         = BALL_BLUE_ID;  // Which ball to chase (or -1 for any)
int BLOCKS_TO_COLLECT      = 3;             // Blocks needed before going to goal

// ===== SPEED SETTINGS =====
// *** TESTING: set to 10. Change to 75 (or higher) for competition. ***
#define DRIVE_SPEED_PCT      50.0  // Master speed control (all other speeds derive from this)

// Derived speeds (automatically calculated - don't edit directly)
#define BASE_DRIVE_SPEED    (DRIVE_SPEED_PCT)        // Normal driving speed
#define NEAR_DRIVE_SPEED    (DRIVE_SPEED_PCT * 0.80) // Within 1m of target (80%)
#define COLLECT_DRIVE_SPEED (DRIVE_SPEED_PCT * 0.80) // Driving through block (80%)
#define SEARCH_SPIN_SPEED   5.0                  // Rotating to search (FIXED at 5% for gentle rotation)
#define GOAL_APPROACH_SPEED (DRIVE_SPEED_PCT)    // Driving to goal

// ===== CAMERA & DETECTION SETTINGS =====
// Detection confirmation (prevents false positives)
#define CONFIRM_WINDOW      8      // Frame window size (smaller = faster)
#define LOSE_THRESHOLD      10     // Consecutive misses before dropping target (higher = more sticky)
#define ACQUIRE_DELAY_MS    0      // No delay - start tracking immediately

// Per-class confirmation thresholds (out of CONFIRM_WINDOW frames)
// Lower = faster reaction, Higher = more stable
int confirmThresholdForClass(int classID) {
    switch (classID) {
        case BALL_BLUE_ID:         return 2;  // BallBlue - need 2/8 frames (25%) - RELAXED
        case BALL_BLUE_IN_GOAL_ID: return 4;  // BallBlueInGoal (rarely chased)
        case BALL_RED_ID:          return 2;  // BallRed - need 2/8 frames (25%) - RELAXED
        case BALL_RED_IN_GOAL_ID:  return 4;  // BallRedInGoal (rarely chased)
        default:                   return 2;  // Unknown/any target - RELAXED
    }
}

// Target persistence - once locked, stick with it
#define TARGET_LOCK_DISTANCE  1.5  // meters - max distance change to consider same target - RELAXED
uint32_t targetLockTime = 0;       // When current target was locked
double   lastTargetX = 0.0;        // Last confirmed target position
double   lastTargetY = 0.0;

// ===== OBSTACLE AVOIDANCE SETTINGS =====
#define OBSTACLE_AVOID_DIST  0.2   // meters - start avoiding obstacles at this distance
#define OBSTACLE_AVOID_GAIN  80.0  // Steering strength (higher = sharper avoidance)

// ===== COLLECTION TIMING =====
double COLLECTION_TIME     = 5.0;  // seconds - how long to run intake after reaching block

// ===== FIELD DIMENSIONS (GPS-based navigation) =====
// VEX AI Competition field is 244cm × 244cm
#define FIELD_MIN_X  -122.0  // cm - Left boundary
#define FIELD_MAX_X   122.0  // cm - Right boundary
#define FIELD_MIN_Y  -122.0  // cm - Bottom boundary
#define FIELD_MAX_Y   122.0  // cm - Top boundary
#define FIELD_BOUNDARY_BUFFER 15.0  // cm - Safety margin from edges

// ===== GOAL POSITIONS (GPS coordinates) =====
// Measure these on your field and update!
#define GOAL_LOADER_X   110.0   // cm - Loader goal X position
#define GOAL_LOADER_Y   110.0   // cm - Loader goal Y position
#define GOAL_LONG_X    -110.0   // cm - Long goal X position
#define GOAL_LONG_Y     110.0   // cm - Long goal Y position

// ===== GPS POSITIONING SETTINGS =====
#define GPS_OBSTACLE_AVOID_DIST  15.0  // cm - avoid boundaries at this distance
#define GPS_POSITION_TOLERANCE    5.0  // cm - acceptable positioning error

// ===== GOAL DETECTION (DISABLED - model not trained yet) =====
#define GOALS_ENABLED       0  // Set to 1 when goal detection model is ready

/*============================================================================*/
/*                    END OF CONFIGURABLE PARAMETERS                          */
/*         Hardware declarations and code logic below this line              */
/*============================================================================*/

brain Brain;
controller Controller;

// Drive motors - three per side (motor_group)
motor leftDrive1  = motor(PORT11, ratio18_1, true);
motor leftDrive2  = motor(PORT12, ratio18_1, true);
motor leftDrive3  = motor(PORT13, ratio18_1, true);

motor rightDrive1 = motor(PORT14, ratio18_1, false);
motor rightDrive2 = motor(PORT15, ratio18_1, false);
motor rightDrive3 = motor(PORT16, ratio18_1, false);

motor_group leftDrive  = motor_group(leftDrive1, leftDrive2, leftDrive3);
motor_group rightDrive = motor_group(rightDrive1, rightDrive2, rightDrive3);

// Intake, Outake, and Loader
motor Intake = motor(PORT1, ratio18_1, true);
motor Outake = motor(PORT2, ratio18_1, false);
motor Loader = motor(PORT3, ratio18_1, false);

// GPS + Drivetrain (GPS at back of robot)
gps GPS = gps(PORT22, 0, GPS_OFFSET_MM, distanceUnits::mm, 0);
smartdrive Drivetrain = smartdrive(leftDrive, rightDrive, GPS, 319.19, 320, 40, mm, 1);

// Competition and comms
competition Competition;
ai::jetson jetson_comms;

// Robot link configuration
#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link link(PORT20, "robot_32456_1", linkType::manager);
#else
#pragma message("building for the worker")
ai::robot_link link(PORT10, "robot_32456_1", linkType::worker);
#endif

// ===== DETECTION CONFIDENCE TRACKER =====
// Circular buffer recording whether the target was seen (1) or not (0) each frame.
struct ConfidenceTracker {
    int  window[CONFIRM_WINDOW]; // circular buffer: 1=seen, 0=not seen
    int  head;                   // next write position
    int  count;                  // how many frames recorded so far (up to CONFIRM_WINDOW)
    int  hits;                   // running sum of 1s in the window
    bool confirmed;              // true once threshold first reached
    int  consecutiveMisses;      // misses since last detection while confirmed

    ConfidenceTracker() : head(0), count(0), hits(0), confirmed(false), consecutiveMisses(0) {
        for (int i = 0; i < CONFIRM_WINDOW; i++) window[i] = 0;
    }

    // Call every loop with whether the target was seen this frame.
    // Returns true if the target should currently be acted upon.
    bool update(bool seen, int classID) {
        // Remove the oldest value from the running sum
        hits -= window[head];
        // Write new value
        window[head] = seen ? 1 : 0;
        hits += window[head];
        head = (head + 1) % CONFIRM_WINDOW;
        if (count < CONFIRM_WINDOW) count++;

        int threshold = confirmThresholdForClass(classID);

        if (!confirmed) {
            // Acquire: need threshold hits in the window to confirm
            if (hits >= threshold) {
                confirmed = true;
                consecutiveMisses = 0;
            }
        } else {
            // Hold: drop confirmation after LOSE_THRESHOLD consecutive misses
            if (seen) {
                consecutiveMisses = 0;
            } else {
                consecutiveMisses++;
                if (consecutiveMisses >= LOSE_THRESHOLD) {
                    confirmed         = false;
                    consecutiveMisses = 0;
                }
            }
        }

        return confirmed;
    }

    // How many hits are in the current window (for display)
    int windowHits() const { return hits; }

    void reset() {
        for (int i = 0; i < CONFIRM_WINDOW; i++) window[i] = 0;
        head = 0; count = 0; hits = 0;
        confirmed = false; consecutiveMisses = 0;
    }
};

// Global tracker for the main tracking loop
ConfidenceTracker targetTracker;

// State
int      blocksCollected     = 0;
bool     collectingBlock     = false;
uint32_t collectionStartTime = 0;

// Logging state
uint32_t lastLogTime = 0;
int      logSequence = 0;

// Helper: Display robot status on brain screen with diagnostics
void displayStatus(const char* mode, double dist = 0, double gpsX = 0, double gpsY = 0, double heading = 0, 
                   int detections = 0, int hits = 0, int threshold = 0) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("%s %d/%d", mode, blocksCollected + 1, BLOCKS_TO_COLLECT);
    if (dist > 0) {
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("D:%.2fm GPS:(%.0f,%.0f) H:%.0f", dist, gpsX, gpsY, heading);
    }
    // Show detection diagnostics
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Det:%d Hits:%d/%d", detections, hits, threshold);
}

/*---------------------------------------------------------------------------*/
/*  Diagnostic Logging Functions                                             */
/*---------------------------------------------------------------------------*/
void logDiagnostic(const char* state, int detections, int hits, int threshold, 
                   double dist, bool confirmed, bool rawFound) {
#if ENABLE_SERIAL_LOGGING
    uint32_t now = Brain.Timer.system();
    if ((now - lastLogTime) < LOG_INTERVAL_MS) return;  // Throttle logging
    lastLogTime = now;
    
    // Print header every 20 logs for readability
    if (logSequence % 20 == 0) {
        printf("\n=== DIAGNOSTIC LOG ===\n");
        printf("Time(ms) | State      | Det | Hits | Dist | Conf | Raw | Blks\n");
        printf("---------|------------|-----|------|------|------|-----|-----\n");
    }
    
    // Log data to serial console
    printf("%-8lu | %-10s | %-3d | %d/%d  | %.2fm | %-4s | %-3s | %d/%d\n",
           now, state, detections, hits, threshold, dist, 
           confirmed ? "YES" : "NO", rawFound ? "YES" : "NO", 
           blocksCollected, BLOCKS_TO_COLLECT);
    
    logSequence++;
#endif
}

void controllerFeedback(const char* event) {
#if ENABLE_CONTROLLER_FEEDBACK
    if (strcmp(event, "LOCKED") == 0) {
        // Short buzz when target locked
        Controller.rumble(".");
    } else if (strcmp(event, "COLLECTING") == 0) {
        // Double buzz when collecting
        Controller.rumble("..");
    } else if (strcmp(event, "COLLECTED") == 0) {
        // Long buzz when block collected
        Controller.rumble("-");
    } else if (strcmp(event, "LOST") == 0) {
        // Short-short buzz when target lost
        Controller.rumble(". .");
    }
#endif
}

/*---------------------------------------------------------------------------*/
/*  GPS Helper Functions                                                     */
/*---------------------------------------------------------------------------*/
// Get robot's front position (accounting for GPS at back)
void getRobotFrontPosition(double &frontX, double &frontY) {
    double robotX = GPS.xPosition(distanceUnits::cm);
    double robotY = GPS.yPosition(distanceUnits::cm);
    double heading = GPS.heading(degrees);
    
    // GPS is at back, calculate front position
    // GPS_OFFSET_MM is negative, so we add it in the heading direction
    double offsetCm = GPS_OFFSET_MM / 10.0;  // Convert mm to cm
    frontX = robotX - offsetCm * sin(heading * M_PI / 180.0);
    frontY = robotY - offsetCm * cos(heading * M_PI / 180.0);
}

// Check if a position is safe (within field boundaries)
bool isPositionSafe(double x, double y) {
    return (x > FIELD_MIN_X + FIELD_BOUNDARY_BUFFER &&
            x < FIELD_MAX_X - FIELD_BOUNDARY_BUFFER &&
            y > FIELD_MIN_Y + FIELD_BOUNDARY_BUFFER &&
            y < FIELD_MAX_Y - FIELD_BOUNDARY_BUFFER);
}

// GPS-based field boundary avoidance - returns steering correction
double gpsFieldBoundaryAvoidance() {
    double frontX, frontY;
    getRobotFrontPosition(frontX, frontY);
    double heading = GPS.heading(degrees);
    double steerCorrection = 0.0;
    
    // Check proximity to each boundary and apply repulsion
    // Left boundary
    if (frontX < FIELD_MIN_X + GPS_OBSTACLE_AVOID_DIST) {
        double penetration = (FIELD_MIN_X + GPS_OBSTACLE_AVOID_DIST - frontX);
        double repulsion = (penetration / GPS_OBSTACLE_AVOID_DIST) * 30.0;
        // If heading left (180-360), push right
        if (heading > 180) steerCorrection += repulsion;
    }
    
    // Right boundary
    if (frontX > FIELD_MAX_X - GPS_OBSTACLE_AVOID_DIST) {
        double penetration = (frontX - (FIELD_MAX_X - GPS_OBSTACLE_AVOID_DIST));
        double repulsion = (penetration / GPS_OBSTACLE_AVOID_DIST) * 30.0;
        // If heading right (0-180), push left
        if (heading < 180) steerCorrection -= repulsion;
    }
    
    // Bottom boundary
    if (frontY < FIELD_MIN_Y + GPS_OBSTACLE_AVOID_DIST) {
        double penetration = (FIELD_MIN_Y + GPS_OBSTACLE_AVOID_DIST - frontY);
        double repulsion = (penetration / GPS_OBSTACLE_AVOID_DIST) * 30.0;
        // If heading down (90-270), push away
        double angleFromDown = fabs(heading - 180.0);
        if (angleFromDown < 90) steerCorrection += repulsion * (heading > 180 ? 1 : -1);
    }
    
    // Top boundary
    if (frontY > FIELD_MAX_Y - GPS_OBSTACLE_AVOID_DIST) {
        double penetration = (frontY - (FIELD_MAX_Y - GPS_OBSTACLE_AVOID_DIST));
        double repulsion = (penetration / GPS_OBSTACLE_AVOID_DIST) * 30.0;
        // If heading up (270-360 or 0-90), push away
        double angleFromUp = fabs(heading);
        if (angleFromUp < 90 || angleFromUp > 270) {
            steerCorrection += repulsion * (heading < 90 || heading > 270 ? -1 : 1);
        }
    }
    
    if (steerCorrection >  40.0) steerCorrection =  40.0;
    if (steerCorrection < -40.0) steerCorrection = -40.0;
    return steerCorrection;
}

// Calculate distance from current position to a target position
double gpsDistanceTo(double targetX, double targetY) {
    double robotX = GPS.xPosition(distanceUnits::cm);
    double robotY = GPS.yPosition(distanceUnits::cm);
    return sqrt((targetX - robotX) * (targetX - robotX) + 
                (targetY - robotY) * (targetY - robotY));
}

// Calculate angle to turn to face a target position
double gpsAngleTo(double targetX, double targetY) {
    double robotX = GPS.xPosition(distanceUnits::cm);
    double robotY = GPS.yPosition(distanceUnits::cm);
    double dx = targetX - robotX;
    double dy = targetY - robotY;
    
    // atan2 gives angle from north (0 degrees = north, 90 = east)
    double angle = atan2(dx, dy) * (180.0 / M_PI);
    if (angle < 0) angle += 360.0;
    return angle;
}

/*---------------------------------------------------------------------------*/
/*  Obstacle avoidance helper                                                */
/*---------------------------------------------------------------------------*/
double obstacleSteeringCorrection(const AI_RECORD &map, int targetClassID) {
    double steerCorrection = 0.0;

    for (int i = 0; i < map.detectionCount; i++) {
        int id = map.detections[i].classID;

        // Skip the target ball
        bool isTarget = (targetClassID == -1) || (id == targetClassID);
        if (isTarget) continue;

        // Skip ALL balls/blocks - robot should only avoid field obstacles
        // Class IDs: 0=BallBlue, 1=BallBlueInGoal, 2=BallRed, 3=BallRedInGoal
        bool isBall = (id == BALL_BLUE_ID || id == BALL_BLUE_IN_GOAL_ID ||
                       id == BALL_RED_ID  || id == BALL_RED_IN_GOAL_ID);
        if (isBall) continue;

        // Skip goals (when GOALS_ENABLED and goal model is trained)
        // This will be uncommented when goals are added to the model
        // bool isGoal = (id == GOAL_LOADER_CLASSID || id == GOAL_LONG_CLASSID);
        // if (isGoal) continue;

        // Only avoid actual obstacles (robots, walls, field elements)
        double ox   = map.detections[i].mapLocation.x;
        double oy   = map.detections[i].mapLocation.y;
        double dist = sqrt(ox * ox + oy * oy);

        if (dist < OBSTACLE_AVOID_DIST && dist > 0.01) {
            double angleToObstacle = atan2(ox, oy) * (180.0 / M_PI);
            double repulsion = OBSTACLE_AVOID_GAIN *
                               (OBSTACLE_AVOID_DIST - dist) / OBSTACLE_AVOID_DIST;
            steerCorrection -= repulsion * (angleToObstacle / 90.0);
        }
    }

    if (steerCorrection >  60.0) steerCorrection =  60.0;
    if (steerCorrection < -60.0) steerCorrection = -60.0;
    return steerCorrection;
}

/*---------------------------------------------------------------------------*/
/*  Apply drive powers - SIMPLIFIED (no steering for testing)                */
/*---------------------------------------------------------------------------*/
void applyDrive(double baseSpeed, double targetSteer, double obstacleSteer) {
    // TESTING: Ignore all steering corrections, just drive straight
    // Once basic movement works, we can add steering back gradually
    
    // Both motors same speed - drive straight
    double speed = baseSpeed;
    
    leftDrive.setVelocity(speed, percent);
    rightDrive.setVelocity(speed, percent);
    leftDrive.spin(forward);
    rightDrive.spin(forward);
}

/*---------------------------------------------------------------------------*/
/*  GPS-based goal positioning - positions robot in front of goal to score  */
/*---------------------------------------------------------------------------*/
void gpsPositionForGoal(double goalX, double goalY, bool useLoader) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("GPS GOAL POSITIONING");
    
    // Calculate target: 30cm in front of goal
    double approachDist = 30.0;
    double robotX = GPS.xPosition(distanceUnits::cm);
    double robotY = GPS.yPosition(distanceUnits::cm);
    double dx = goalX - robotX;
    double dy = goalY - robotY;
    double currentDist = sqrt(dx*dx + dy*dy);
    double norm = currentDist;
    double targetX = robotX + (dx/norm) * (currentDist - approachDist);
    double targetY = robotY + (dy/norm) * (currentDist - approachDist);
    
    // Move to approach position
    uint32_t startTime = Brain.Timer.system();
    uint32_t timeout = 15000;
    
    while ((Brain.Timer.system() - startTime) < timeout) {
        double currentX = GPS.xPosition(distanceUnits::cm);
        double currentY = GPS.yPosition(distanceUnits::cm);
        double distToTarget = sqrt((targetX - currentX)*(targetX - currentX) + 
                                   (targetY - currentY)*(targetY - currentY));
        
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Dist: %.1f cm", distToTarget);
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Pos: (%.1f,%.1f)", currentX, currentY);
        
        if (distToTarget < GPS_POSITION_TOLERANCE) {
            // Reached target position, now turn to face goal
            leftDrive.stop(brake);
            rightDrive.stop(brake);
            
            // Turn to face goal
            double finalAngle = gpsAngleTo(goalX, goalY);
            double currentHeading = GPS.heading(degrees);
            double angleDiff = finalAngle - currentHeading;
            while (angleDiff > 180) angleDiff -= 360;
            while (angleDiff < -180) angleDiff += 360;
            
            Brain.Screen.setCursor(6, 1);
            Brain.Screen.print("Turning to %.1f deg", finalAngle);
            
            // Turn to face goal
            if (fabs(angleDiff) > 5.0) {
                turnType dir = (angleDiff > 0) ? right : left;
                leftDrive.setVelocity(20, percent);
                rightDrive.setVelocity(20, percent);
                Drivetrain.turnToHeading(finalAngle, degrees, 20, velocityUnits::pct);
            }
            
            leftDrive.stop(brake);
            rightDrive.stop(brake);
            
            // Score!
            Brain.Screen.setCursor(7, 1);
            Brain.Screen.print("SCORING!");
            
            if (useLoader) {
                Loader.setVelocity(50, percent);
                Loader.spin(forward);
                Intake.spin(reverse);
                wait(5, seconds);
                Intake.stop();
                Loader.spin(reverse);
                wait(2, seconds);
                Loader.stop();
            } else {
                Outake.setVelocity(50, percent);
                Outake.spin(forward);
                Intake.spin(reverse);
                wait(5, seconds);
                Intake.stop();
                Outake.stop();
            }
            
            blocksCollected = 0;
            return;
        }
        
        // Drive toward target position
        double angleToTarget = gpsAngleTo(targetX, targetY);
        double currentHeading = GPS.heading(degrees);
        double steerAngle = angleToTarget - currentHeading;
        while (steerAngle > 180) steerAngle -= 360;
        while (steerAngle < -180) steerAngle += 360;
        
        double targetSteer = steerAngle * 0.8;
        double baseSpeed = (distToTarget < 50.0) 
            ? NEAR_DRIVE_SPEED + (distToTarget/50.0 * (GOAL_APPROACH_SPEED - NEAR_DRIVE_SPEED))
            : GOAL_APPROACH_SPEED;
        
        // Get field boundary avoidance
        double gpsSteer = gpsFieldBoundaryAvoidance();
        
        applyDrive(baseSpeed, targetSteer, gpsSteer);
        
        this_thread::sleep_for(33);
    }
    
    // Timeout - stop
    leftDrive.stop(brake);
    rightDrive.stop(brake);
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("TIMEOUT");
}

/*---------------------------------------------------------------------------*/
/*  Autonomous                                                               */
/*---------------------------------------------------------------------------*/
void autonomousMain(void) {
    skillsAutonomous();
}

/*---------------------------------------------------------------------------*/
/*  Main                                                                     */
/*---------------------------------------------------------------------------*/
int main() {
    static AI_RECORD local_map;
    int32_t loop_time = 33;

    thread t1(dashboardTask);

    Competition.autonomous(autonomousMain);

    this_thread::sleep_for(loop_time);

    Intake.setVelocity(30, percent);
    Outake.setVelocity(30, percent);
    Loader.setVelocity(30, percent);
    leftDrive.setStopping(coast);
    rightDrive.setStopping(coast);

    while (1) {
        // Trigger skills via controller (Down + A)
        if (Controller.ButtonDown.pressing() && Controller.ButtonA.pressing()) {
            leftDrive.stop(brake);
            rightDrive.stop(brake);
            Intake.stop();
            targetTracker.reset();
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("STARTING SKILLS!");
            wait(1, seconds);
            skillsAutonomous();
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("SKILLS COMPLETE");
            wait(2, seconds);
        }
        
        // GPS-based goal scoring via controller (Up + Y for Loader, Up + B for Long)
        if (Controller.ButtonUp.pressing()) {
            if (Controller.ButtonY.pressing()) {
                leftDrive.stop(brake);
                rightDrive.stop(brake);
                targetTracker.reset();
                Brain.Screen.clearScreen();
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("GPS GOAL: LOADER");
                wait(0.5, seconds);
                gpsPositionForGoal(GOAL_LOADER_X, GOAL_LOADER_Y, true);
                wait(1, seconds);
            } else if (Controller.ButtonB.pressing()) {
                leftDrive.stop(brake);
                rightDrive.stop(brake);
                targetTracker.reset();
                Brain.Screen.clearScreen();
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("GPS GOAL: LONG");
                wait(0.5, seconds);
                gpsPositionForGoal(GOAL_LONG_X, GOAL_LONG_Y, false);
                wait(1, seconds);
            }
        }

        // Go score if enough blocks collected (disabled until goal model is trained)
#if GOALS_ENABLED
        if (blocksCollected >= BLOCKS_TO_COLLECT) {
            targetTracker.reset();
            driveToGoal();
        }
#endif

        jetson_comms.get_data(&local_map);
        link.set_remote_location(local_map.pos.x, local_map.pos.y,
                                 local_map.pos.az, local_map.pos.status);

        // Show detection count for debugging
        int totalDetections = local_map.detectionCount;

        // --- FIND BEST TARGET (prefer locked target, then closest) ---
        bool   rawFound    = false;
        double closestDist = 999999.0;
        double lockedDist  = 999999.0;
        int    targetIndex = -1;
        int    lockedIndex = -1;

        for (int i = 0; i < local_map.detectionCount; i++) {
            bool isTarget = (TARGET_CLASSID == -1) ||
                            (local_map.detections[i].classID == TARGET_CLASSID);
            if (isTarget) {
                double x = local_map.detections[i].mapLocation.x;
                double y = local_map.detections[i].mapLocation.y;
                double d = sqrt(x * x + y * y);
                
                // Check if this is near our last locked target
                if (targetTracker.confirmed) {
                    double dx = x - lastTargetX;
                    double dy = y - lastTargetY;
                    double distFromLocked = sqrt(dx * dx + dy * dy);
                    if (distFromLocked < TARGET_LOCK_DISTANCE && d < lockedDist) {
                        lockedDist = d;
                        lockedIndex = i;
                    }
                }
                
                // Also track closest
                if (d < closestDist) { 
                    closestDist = d; 
                    targetIndex = i; 
                }
            }
        }
        
        // Prefer locked target over closest
        if (lockedIndex != -1) {
            targetIndex = lockedIndex;
            closestDist = lockedDist;
            rawFound = true;
        } else if (targetIndex != -1) {
            rawFound = true;
        }

        // --- UPDATE CONFIDENCE TRACKER ---
        // confirmed = true only once the target appears in enough recent frames
        bool wasConfirmed = targetTracker.confirmed;  // Track previous state
        bool confirmed = targetTracker.update(rawFound, TARGET_CLASSID);
        
        // Start intake immediately when target is first confirmed (decisive lock)
        if (confirmed && !wasConfirmed && !collectingBlock) {
            Intake.spin(forward);
            Brain.Screen.setCursor(4, 1);
            Brain.Screen.print("INTAKE ON - LOCKED!");
            controllerFeedback("LOCKED");  // Rumble when locked
        }
        
        // Update locked target position when confirmed
        if (confirmed && rawFound && targetIndex != -1) {
            lastTargetX = local_map.detections[targetIndex].mapLocation.x;
            lastTargetY = local_map.detections[targetIndex].mapLocation.y;
            if (targetLockTime == 0) {
                targetLockTime = Brain.Timer.system();
            }
        } else if (!confirmed) {
            if (targetLockTime != 0) {
                controllerFeedback("LOST");  // Rumble when target lost
            }
            targetLockTime = 0;  // Reset lock when lost
            if (!collectingBlock) {
                Intake.stop();  // Stop intake if not collecting and lost target
            }
        }

        // Use the last known good position if confirmed but momentarily missing this frame
        double tx = 0.0, ty = 0.0, dist = 0.0;
        if (rawFound && targetIndex != -1) {
            tx   = local_map.detections[targetIndex].mapLocation.x;
            ty   = local_map.detections[targetIndex].mapLocation.y;
            dist = sqrt(tx * tx + ty * ty);
        }

        // Obstacle avoidance (computed every frame regardless of confirmation)
        double avoidSteer = obstacleSteeringCorrection(local_map, TARGET_CLASSID);

        // --- ACT ON CONFIRMED TARGET ---
        int hits = targetTracker.windowHits();
        int threshold = confirmThresholdForClass(TARGET_CLASSID);
        
        // Log current state
        const char* currentState = confirmed ? (collectingBlock ? "COLLECTING" : "TRACK") : 
                                  (rawFound ? "ACQUIRE" : "SCAN");
        logDiagnostic(currentState, totalDetections, hits, threshold, dist, confirmed, rawFound);
        
        if (confirmed && rawFound) {
            // TRACKING: camera has a confirmed lock on the target
            double robotX = GPS.xPosition(distanceUnits::cm);
            double robotY = GPS.yPosition(distanceUnits::cm);
            double heading = GPS.heading(degrees);
            displayStatus("TRACK", dist, robotX, robotY, heading, totalDetections, hits, threshold);
            
            if (fabs(avoidSteer) > 5.0) {
                Brain.Screen.setCursor(4, 1);
                Brain.Screen.print("AVOID:%.1f", avoidSteer);
            }

            if (collectingBlock) {
                // Collecting - wait for collection time to complete
                if ((Brain.Timer.system() - collectionStartTime) >=
                        (uint32_t)(COLLECTION_TIME * 1000)) {
                    collectingBlock = false;
                    blocksCollected++;
                    Intake.stop();
                    targetTracker.reset();  // Reset for next target
                    targetLockTime = 0;
                    controllerFeedback("COLLECTED");  // Rumble when collected
                } else {
                    double angle       = atan2(tx, ty) * (180.0 / M_PI);
                    double targetSteer = angle * 0.85;
                    applyDrive(COLLECT_DRIVE_SPEED, targetSteer, avoidSteer);
                }
            } else if (dist <= 0.15) {
                // Close enough - start collection timer (intake already running)
                collectingBlock     = true;
                collectionStartTime = Brain.Timer.system();
                Brain.Screen.setCursor(5, 1);
                Brain.Screen.print("COLLECTING!");
                controllerFeedback("COLLECTING");  // Rumble when starting collection
            } else {
                // Normal tracking — drive toward target with obstacle avoidance
                double angle       = atan2(tx, ty) * (180.0 / M_PI);
                double targetSteer = angle * 0.85;
                double baseSpeed   = (dist < 1.0)
                    ? NEAR_DRIVE_SPEED + (dist * (BASE_DRIVE_SPEED - NEAR_DRIVE_SPEED))
                    : BASE_DRIVE_SPEED;

                applyDrive(baseSpeed, targetSteer, avoidSteer);
            }

        } else if (confirmed && !rawFound) {
            // HOLDING: confirmed but missed this frame
            displayStatus("HOLD", 0, 0, 0, 0, totalDetections, hits, threshold);

        } else {
            // Not confirmed — acquiring or lost
            if (!collectingBlock) {
                if (!rawFound) {
                    // No detection - stop and wait (don't spin violently)
                    leftDrive.stop(brake);
                    rightDrive.stop(brake);

                    double robotX = GPS.xPosition(distanceUnits::cm);
                    double robotY = GPS.yPosition(distanceUnits::cm);
                    double heading = GPS.heading(degrees);
                    displayStatus("SCAN", 0, robotX, robotY, heading, totalDetections, hits, threshold);
                } else {
                    // Target detected - acquiring, drive straight toward it while building confidence
                    displayStatus("ACQUIRE", dist, 0, 0, 0, totalDetections, hits, threshold);
                    
                    // Drive straight toward target (no steering since applyDrive ignores it)
                    double baseSpeed = (dist < 1.0)
                        ? NEAR_DRIVE_SPEED + (dist * (BASE_DRIVE_SPEED - NEAR_DRIVE_SPEED))
                        : BASE_DRIVE_SPEED;
                    applyDrive(baseSpeed * 0.7, 0, 0);  // 70% speed during acquire, no steering
                }
            } else if ((Brain.Timer.system() - collectionStartTime) >=
                           (uint32_t)(COLLECTION_TIME * 1000)) {
                collectingBlock = false;
                blocksCollected++;
                Intake.stop();
                targetTracker.reset();  // Reset for next target
                targetLockTime = 0;
                controllerFeedback("COLLECTED");  // Rumble when collected
            }
        }

        jetson_comms.request_map();
        this_thread::sleep_for(loop_time);
    }
}
