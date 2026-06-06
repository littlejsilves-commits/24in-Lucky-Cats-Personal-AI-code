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
#define DRIVE_SPEED_PCT      10.0  // Master speed control (all other speeds derive from this)

// Derived speeds (automatically calculated - don't edit directly)
#define BASE_DRIVE_SPEED    (DRIVE_SPEED_PCT)        // Normal driving speed
#define NEAR_DRIVE_SPEED    (DRIVE_SPEED_PCT * 0.80) // Within 1m of target (80%)
#define COLLECT_DRIVE_SPEED (DRIVE_SPEED_PCT * 0.80) // Driving through block (80%)
#define SEARCH_SPIN_SPEED   (DRIVE_SPEED_PCT * 0.80) // Rotating to search (80%)
#define GOAL_APPROACH_SPEED (DRIVE_SPEED_PCT)        // Driving to goal

// ===== CAMERA & DETECTION SETTINGS =====
#define CAMERA_MIN_DIST       0.10  // meters - camera loses target below this distance
#define BLIND_APPROACH_DIST   0.20  // meters - distance to drive blind to reach target
#define BLIND_APPROACH_SPEED_MPS  0.40  // m/s - robot speed at COLLECT_DRIVE_SPEED (measure & adjust)

// Detection confirmation (prevents false positives)
#define CONFIRM_WINDOW      10     // Frame window size for confirmation
#define LOSE_THRESHOLD       5     // Consecutive misses before dropping target
#define ACQUIRE_DELAY_MS    500    // Hold still when first detecting target (ms)

// Per-class confirmation thresholds (out of CONFIRM_WINDOW frames)
// Lower = faster reaction, Higher = more stable
int confirmThresholdForClass(int classID) {
    switch (classID) {
        case BALL_BLUE_ID:         return 6;  // BallBlue
        case BALL_BLUE_IN_GOAL_ID: return 8;  // BallBlueInGoal (rarely chased)
        case BALL_RED_ID:          return 6;  // BallRed
        case BALL_RED_IN_GOAL_ID:  return 8;  // BallRedInGoal (rarely chased)
        default:                   return 4;  // Unknown/any target
    }
}

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
#define GPS_OBSTACLE_AVOID_DIST  30.0  // cm - avoid boundaries at this distance
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
        case BALL_RED_IN_GOAL_ID:  return 8;   // BallRedInGoal — rarely a chase target
        default:                   return 4;   // Any / unknown
    }
}

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

// Blind approach state — active while driving the last BLIND_APPROACH_DIST blind
bool     blindApproachActive    = false;
uint32_t blindApproachStartTime = 0;
uint32_t blindApproachDuration  = 0;   // ms — computed when blind approach starts
double   blindApproachSteer     = 0.0; // locked heading steer value

// Acquire delay state
uint32_t firstDetectTime    = 0;   // timestamp of first detection in current acquire cycle
bool     acquireDelayActive = false;

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
/*  Apply drive powers with obstacle avoidance blended in                    */
/*---------------------------------------------------------------------------*/
void applyDrive(double baseSpeed, double targetSteer, double obstacleSteer) {
    // Add GPS-based field boundary avoidance
    double gpsSteer = gpsFieldBoundaryAvoidance();
    double totalSteer = targetSteer + obstacleSteer + gpsSteer;

    double lp = baseSpeed + totalSteer;
    double rp = baseSpeed - totalSteer;

    lp = (lp >  100) ?  100 : (lp < -100) ? -100 : lp;
    rp = (rp >  100) ?  100 : (rp < -100) ? -100 : rp;

    leftDrive.setVelocity(fabs(lp), percent);
    rightDrive.setVelocity(fabs(rp), percent);
    leftDrive.spin(lp  >= 0 ? forward : reverse);
    rightDrive.spin(rp >= 0 ? forward : reverse);
}

/*---------------------------------------------------------------------------*/
/*  Drive to the goal using Jetson camera detections                         */
/*  NOTE: Disabled until goal model is trained (GOALS_ENABLED = 0)          */
/*---------------------------------------------------------------------------*/
#if GOALS_ENABLED
void driveToGoal() {
    // Goal class IDs — fill in from types.ts when model is trained
    #define GOAL_LOADER_CLASSID 4   // Loader goal (uses Loader motor)
    #define GOAL_LONG_CLASSID   5   // Long goal (uses Outake motor)
    #define TARGET_GOAL         0  // 0=Loader 1=Long 2=Closest
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("GOING TO GOAL!");

    int goalClassID = GOAL_LOADER_CLASSID;
    bool useLoader = true;  // true = Loader goal, false = Long goal
    
    if (TARGET_GOAL == 1) {
        goalClassID = GOAL_LONG_CLASSID;
        useLoader = false;
    } else if (TARGET_GOAL == 0) {
        goalClassID = GOAL_LOADER_CLASSID;
        useLoader = true;
    }

    bool     reachedGoal     = false;
    uint32_t searchStartTime = Brain.Timer.system();
    uint32_t searchTimeout   = 30000;

    while (!reachedGoal && (Brain.Timer.system() - searchStartTime) < searchTimeout) {
        static AI_RECORD goal_map;
        jetson_comms.get_data(&goal_map);

        bool   goalFound       = false;
        int    goalIndex       = -1;
        double closestGoalDist = 999999.0;

        for (int i = 0; i < goal_map.detectionCount; i++) {
            bool isTarget = false;
            if (TARGET_GOAL == 2) {
                // Closest goal (either type)
                isTarget = (goal_map.detections[i].classID == GOAL_LOADER_CLASSID ||
                            goal_map.detections[i].classID == GOAL_LONG_CLASSID);
            } else {
                isTarget = (goal_map.detections[i].classID == goalClassID);
            }
            if (isTarget) {
                double x = goal_map.detections[i].mapLocation.x;
                double y = goal_map.detections[i].mapLocation.y;
                double d = sqrt(x * x + y * y);
                if (d < closestGoalDist) {
                    closestGoalDist = d;
                    goalIndex       = i;
                    goalFound       = true;
                    // If searching for closest, determine which type it is
                    if (TARGET_GOAL == 2) {
                        useLoader = (goal_map.detections[i].classID == GOAL_LOADER_CLASSID);
                    }
                }
            }
        }

        if (goalFound) {
            double gx   = goal_map.detections[goalIndex].mapLocation.x;
            double gy   = goal_map.detections[goalIndex].mapLocation.y;
            double dist = sqrt(gx * gx + gy * gy);

            Brain.Screen.setCursor(2, 1);
            Brain.Screen.print("Goal Dist: %.2f m", dist);

            double avoidSteer = obstacleSteeringCorrection(goal_map, goalClassID);
            if (fabs(avoidSteer) > 5.0) {
                Brain.Screen.setCursor(4, 1);
                Brain.Screen.print("AVOIDING  steer:%.1f", avoidSteer);
            }

            if (dist <= 0.1) {
                reachedGoal = true;
                leftDrive.stop(brake);
                rightDrive.stop(brake);

                if (useLoader) {
                    // Score on Loader goal using Loader motor
                    Brain.Screen.setCursor(3, 1);
                    Brain.Screen.print("SCORING ON LOADER GOAL");
                    Loader.setVelocity(50, percent);
                    Loader.spin(forward);
                    Intake.spin(reverse);
                    wait(5, seconds);
                    Intake.stop();
                    Loader.spin(reverse);
                    wait(2, seconds);
                    Loader.stop();
                } else {
                    // Score on Long goal using Outake motor
                    Brain.Screen.setCursor(3, 1);
                    Brain.Screen.print("SCORING ON LONG GOAL");
                    Outake.setVelocity(50, percent);
                    Outake.spin(forward);
                    Intake.spin(reverse);
                    wait(5, seconds);
                    Intake.stop();
                    Outake.stop();
                }
            } else {
                double angle       = atan2(gx, gy) * (180.0 / M_PI);
                double targetSteer = angle * 0.85;
                double baseSpeed   = (dist < 1.0)
                    ? NEAR_DRIVE_SPEED + (dist * (GOAL_APPROACH_SPEED - NEAR_DRIVE_SPEED))
                    : GOAL_APPROACH_SPEED;

                applyDrive(baseSpeed, targetSteer, avoidSteer);
            }
        } else {
            // Rotate to search for goal
            leftDrive.setVelocity(SEARCH_SPIN_SPEED, percent);
            rightDrive.setVelocity(SEARCH_SPIN_SPEED, percent);
            leftDrive.spin(forward);
            rightDrive.spin(reverse);
        }

        jetson_comms.request_map();
        this_thread::sleep_for(33);
    }

    leftDrive.stop(brake);
    rightDrive.stop(brake);
    blocksCollected = 0;
}
#endif // GOALS_ENABLED

/*---------------------------------------------------------------------------*/
/*  GPS-based goal positioning - positions robot in front of goal to score  */
/*  Uses GPS coordinates for precise positioning regardless of camera view  */
/*---------------------------------------------------------------------------*/
void gpsPositionForGoal(double goalX, double goalY, bool useLoader) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("GPS POSITIONING FOR GOAL");
    
    // Calculate target position: stand 30cm in front of goal
    double approachDist = 30.0;  // cm
    double angleToGoal = gpsAngleTo(goalX, goalY);
    
    // Target position is 30cm away from goal, on the line from robot to goal
    double currentDist = gpsDistanceTo(goalX, goalY);
    double targetDist = currentDist - approachDist;
    
    double robotX = GPS.xPosition(distanceUnits::cm);
    double robotY = GPS.yPosition(distanceUnits::cm);
    
    // Calculate approach position
    double dx = goalX - robotX;
    double dy = goalY - robotY;
    double norm = sqrt(dx*dx + dy*dy);
    double targetX = robotX + (dx/norm) * targetDist;
    double targetY = robotY + (dy/norm) * targetDist;
    
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Goal: (%.1f,%.1f)", goalX, goalY);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Target: (%.1f,%.1f)", targetX, targetY);
    
    // Move to approach position using GPS
    uint32_t startTime = Brain.Timer.system();
    uint32_t timeout = 15000;  // 15 second timeout
    
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

        // --- FIND CLOSEST RAW DETECTION THIS FRAME ---
        bool   rawFound    = false;
        double closestDist = 999999.0;
        int    targetIndex = -1;

        for (int i = 0; i < local_map.detectionCount; i++) {
            bool isTarget = (TARGET_CLASSID == -1) ||
                            (local_map.detections[i].classID == TARGET_CLASSID);
            if (isTarget) {
                double x = local_map.detections[i].mapLocation.x;
                double y = local_map.detections[i].mapLocation.y;
                double d = sqrt(x * x + y * y);
                if (d < closestDist) { closestDist = d; targetIndex = i; rawFound = true; }
            }
        }

        // --- UPDATE CONFIDENCE TRACKER ---
        // confirmed = true only once the target appears in enough recent frames
        bool confirmed = targetTracker.update(rawFound, TARGET_CLASSID);

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
        if (blindApproachActive) {
            // ---------------------------------------------------------------
            // BLIND APPROACH: camera can't see the target at this range.
            // Drive straight on the locked heading until the timed run completes,
            // then immediately start collection.
            // ---------------------------------------------------------------
            uint32_t elapsed = Brain.Timer.system() - blindApproachStartTime;

            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("BLIND APPROACH %d/%d", blocksCollected + 1, BLOCKS_TO_COLLECT);
            Brain.Screen.setCursor(2, 1);
            Brain.Screen.print("time: %dms / %dms", (int)elapsed, (int)blindApproachDuration);

            if (elapsed >= blindApproachDuration) {
                // Blind run complete — start intake
                blindApproachActive = false;
                collectingBlock     = true;
                collectionStartTime = Brain.Timer.system();
                Intake.spin(forward);
                Brain.Screen.setCursor(3, 1);
                Brain.Screen.print("COLLECTING!");
            } else {
                // Drive straight on locked heading (no obstacle avoidance during blind run)
                applyDrive(COLLECT_DRIVE_SPEED, blindApproachSteer, 0.0);
            }

        } else if (confirmed && rawFound) {
            // ---------------------------------------------------------------
            // TRACKING: camera has a confirmed lock on the target.
            // ---------------------------------------------------------------
            double robotX = GPS.xPosition(distanceUnits::cm);
            double robotY = GPS.yPosition(distanceUnits::cm);
            double heading = GPS.heading(degrees);
            double frontX, frontY;
            getRobotFrontPosition(frontX, frontY);
            
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("TRACKING %d/%d  %.2fm",
                blocksCollected + 1, BLOCKS_TO_COLLECT, dist);
            Brain.Screen.setCursor(2, 1);
            Brain.Screen.print("GPS:(%.1f,%.1f) H:%.0f",
                robotX, robotY, heading);
            Brain.Screen.setCursor(3, 1);
            Brain.Screen.print("conf:%d/%d  miss:%d",
                targetTracker.windowHits(), CONFIRM_WINDOW,
                targetTracker.consecutiveMisses);
            if (fabs(avoidSteer) > 5.0) {
                Brain.Screen.setCursor(5, 1);
                Brain.Screen.print("AVOIDING steer:%.1f", avoidSteer);
            }

            if (collectingBlock) {
                // Already collecting from a previous blind approach completion
                if ((Brain.Timer.system() - collectionStartTime) >=
                        (uint32_t)(COLLECTION_TIME * 1000)) {
                    collectingBlock    = false;
                    acquireDelayActive = false;  // reset for next target
                    firstDetectTime    = 0;
                    blocksCollected++;
                    Intake.stop();
                } else {
                    double angle       = atan2(tx, ty) * (180.0 / M_PI);
                    double targetSteer = angle * 0.85;
                    applyDrive(COLLECT_DRIVE_SPEED, targetSteer, avoidSteer);
                }
            } else if (dist <= CAMERA_MIN_DIST) {
                // ---------------------------------------------------------------
                // Target is within the camera blind zone — start blind approach.
                // Lock the current heading and compute how long to drive.
                // ---------------------------------------------------------------
                blindApproachActive    = true;
                blindApproachStartTime = Brain.Timer.system();
                blindApproachSteer     = atan2(tx, ty) * (180.0 / M_PI) * 0.85;
                // Time (ms) = distance (m) / speed (m/s) * 1000
                blindApproachDuration  = (uint32_t)(
                    (BLIND_APPROACH_DIST / BLIND_APPROACH_SPEED_MPS) * 1000.0);

                Brain.Screen.setCursor(3, 1);
                Brain.Screen.print("BLIND START  dur:%dms", (int)blindApproachDuration);
            } else {
                // Normal tracking — drive toward target with obstacle avoidance
                double angle       = atan2(tx, ty) * (180.0 / M_PI);
                double targetSteer = angle * 0.85;
                double baseSpeed   = (dist < 1.0)
                    ? NEAR_DRIVE_SPEED + (dist * (BASE_DRIVE_SPEED - NEAR_DRIVE_SPEED))
                    : BASE_DRIVE_SPEED;

                applyDrive(baseSpeed, targetSteer, avoidSteer);

                Brain.Screen.setCursor(3, 1);
                Brain.Screen.print("spd:%.0f steer:%.1f", baseSpeed, targetSteer);
            }

        } else if (confirmed && !rawFound) {
            // ---------------------------------------------------------------
            // HOLDING: confirmed but missed this frame.
            // If the last known distance was already in the blind zone,
            // start the blind approach now rather than waiting to lose confirmation.
            // ---------------------------------------------------------------
            if (!blindApproachActive && closestDist <= CAMERA_MIN_DIST) {
                blindApproachActive    = true;
                blindApproachStartTime = Brain.Timer.system();
                // No fresh angle this frame — reuse last steer if available,
                // otherwise drive straight (steer = 0)
                blindApproachSteer    = 0.0;
                blindApproachDuration = (uint32_t)(
                    (BLIND_APPROACH_DIST / BLIND_APPROACH_SPEED_MPS) * 1000.0);
            }

            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("HOLDING... miss:%d/%d",
                targetTracker.consecutiveMisses, LOSE_THRESHOLD);

        } else {
            // Not confirmed — either acquiring or lost
            if (!collectingBlock) {
                int threshold = confirmThresholdForClass(TARGET_CLASSID);

                if (!rawFound) {
                    // No detection at all — reset acquire delay and rotate to scan
                    acquireDelayActive = false;
                    firstDetectTime    = 0;

                    uint32_t now = Brain.Timer.system();
                    bool rotateRight = ((now / 3000) % 2) == 0;

                    leftDrive.setVelocity(SEARCH_SPIN_SPEED, percent);
                    rightDrive.setVelocity(SEARCH_SPIN_SPEED, percent);
                    leftDrive.spin(rotateRight  ? forward : reverse);
                    rightDrive.spin(rotateRight ? reverse : forward);

                    double robotX = GPS.xPosition(distanceUnits::cm);
                    double robotY = GPS.yPosition(distanceUnits::cm);
                    double heading = GPS.heading(degrees);
                    
                    Brain.Screen.clearScreen();
                    Brain.Screen.setCursor(1, 1);
                    Brain.Screen.print("SCANNING... %d/%d",
                        blocksCollected, BLOCKS_TO_COLLECT);
                    Brain.Screen.setCursor(2, 1);
                    Brain.Screen.print("GPS:(%.1f,%.1f) H:%.0f",
                        robotX, robotY, heading);
                    Brain.Screen.setCursor(3, 1);
                    Brain.Screen.print("conf:%d/%d  need:%d",
                        targetTracker.windowHits(), CONFIRM_WINDOW, threshold);
                    Brain.Screen.setCursor(4, 1);
                    Brain.Screen.print("Rotating %s", rotateRight ? "RIGHT" : "LEFT");
                } else {
                    // Target detected — start or continue the acquire delay
                    if (!acquireDelayActive) {
                        acquireDelayActive = true;
                        firstDetectTime    = Brain.Timer.system();
                    }

                    uint32_t delayElapsed = Brain.Timer.system() - firstDetectTime;
                    bool     delayDone    = (delayElapsed >= ACQUIRE_DELAY_MS);

                    // Hold still while waiting and building confidence
                    leftDrive.stop(brake);
                    rightDrive.stop(brake);

                    Brain.Screen.clearScreen();
                    Brain.Screen.setCursor(1, 1);
                    Brain.Screen.print("ACQUIRING... %.2fm", dist);
                    Brain.Screen.setCursor(2, 1);
                    Brain.Screen.print("conf:%d/%d  need:%d",
                        targetTracker.windowHits(), CONFIRM_WINDOW, threshold);
                    Brain.Screen.setCursor(3, 1);
                    if (!delayDone) {
                        Brain.Screen.print("wait: %dms / %dms",
                            (int)delayElapsed, ACQUIRE_DELAY_MS);
                    } else {
                        Brain.Screen.print("ready — waiting for conf");
                    }
                }
            } else if ((Brain.Timer.system() - collectionStartTime) >=
                           (uint32_t)(COLLECTION_TIME * 1000)) {
                collectingBlock    = false;
                acquireDelayActive = false;  // reset for next target
                firstDetectTime    = 0;
                blocksCollected++;
                Intake.stop();
            }
        }

        jetson_comms.request_map();
        this_thread::sleep_for(loop_time);
    }
}
