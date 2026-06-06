/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       skills_autonomous.cpp                                    */
/*    Author:       AI Robot Team                                            */
/*    Created:      2024                                                     */
/*    Description:  Autonomous Skills Challenge Routine for VEX AI Robot    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "robot-config.h"
#include "ai_functions.h"

using namespace vex;

// ===== PNEUMATIC CONFIGURATION =====
// Tall goal piston for scoring at tall goals
digital_out TallGoalPiston = digital_out(Brain.ThreeWirePort.A);

// ===== SKILLS CONFIGURATION =====
#define BLOCKS_PER_GOAL 3           // Number of blocks to collect before scoring
#define TOTAL_GOALS_TO_SCORE 4      // How many times to score (12 blocks total)
#define BLOCK_SEARCH_TIMEOUT 15000  // 15 seconds to find a block
#define GOAL_SEARCH_TIMEOUT 20000   // 20 seconds to find goal
#define SKILLS_TIME_LIMIT 60000     // 60 second time limit

// Block and Goal ClassIDs — must match Jetson model (src/lib/types.ts)
#define BLUE_BLOCK_ID  0   // BallBlue
#define RED_BLOCK_ID   2   // BallRed
// Goal IDs — NOT YET TRAINED. Update when goal model is ready.
#define GOALS_ENABLED  0
#define CENTER_GOAL_ID 4
#define BLUE_GOAL_ID   6
#define RED_GOAL_ID    5
#define TALL_GOAL_ID   7

// Distance thresholds
#define BLOCK_COLLECTION_DISTANCE 0.4  // 40cm to start intake
#define GOAL_SCORING_DISTANCE 0.1      // 10cm to start scoring
#define COLLECTION_TIME 3.0            // 3 seconds to collect each block

// Speed configuration
// *** TESTING: set to 10. Change to 75 (or higher) for competition. ***
#define DRIVE_SPEED_PCT      10.0

#define BASE_DRIVE_SPEED    (DRIVE_SPEED_PCT)        // Normal driving speed
#define NEAR_DRIVE_SPEED    (DRIVE_SPEED_PCT * 0.80) // Slower when within 1m of target
#define SEARCH_SPIN_SPEED   (DRIVE_SPEED_PCT * 0.80) // Speed when rotating to search

// Obstacle avoidance
#define OBSTACLE_AVOID_DIST 0.6        // Start avoiding at 60cm
#define OBSTACLE_AVOID_GAIN 80.0       // Steering repulsion strength

// ===== GLOBAL SKILLS STATE =====
int skillsBlocksCollected = 0;
int skillsGoalsScored = 0;
uint32_t skillsStartTime = 0;
bool skillsRunning = false;

// ===== HELPER FUNCTIONS =====

/**
 * Obstacle avoidance steering correction.
 * Scans detections for objects that are NOT targetClassID and NOT goals,
 * within OBSTACLE_AVOID_DIST meters. Returns a steering offset:
 * positive = steer right, negative = steer left.
 */
static double skillsObstacleSteer(const AI_RECORD &map, int targetClassID) {
    double steer = 0.0;
    for (int i = 0; i < map.detectionCount; i++) {
        int id = map.detections[i].classID;
        bool isTarget = (id == targetClassID);
        bool isGoal   = false;
#if GOALS_ENABLED
        isGoal = (id == CENTER_GOAL_ID || id == BLUE_GOAL_ID ||
                  id == RED_GOAL_ID    || id == TALL_GOAL_ID);
#endif
        if (isTarget || isGoal) continue;

        double ox   = map.detections[i].mapLocation.x;
        double oy   = map.detections[i].mapLocation.y;
        double dist = sqrt(ox * ox + oy * oy);

        if (dist < OBSTACLE_AVOID_DIST && dist > 0.01) {
            double angle     = atan2(ox, oy) * (180.0 / 3.14159265);
            double repulsion = OBSTACLE_AVOID_GAIN *
                               (OBSTACLE_AVOID_DIST - dist) / OBSTACLE_AVOID_DIST;
            steer -= repulsion * (angle / 90.0);
        }
    }
    if (steer >  60.0) steer =  60.0;
    if (steer < -60.0) steer = -60.0;
    return steer;
}

/**
 * Apply motor powers with target steering and obstacle avoidance blended.
 */
static void skillsApplyDrive(double baseSpeed, double targetSteer, double avoidSteer) {
    double total = targetSteer + avoidSteer;
    double lp = baseSpeed + total;
    double rp = baseSpeed - total;
    lp = (lp >  100) ?  100 : (lp < -100) ? -100 : lp;
    rp = (rp >  100) ?  100 : (rp < -100) ? -100 : rp;
    leftDrive.setVelocity(fabs(lp), percent);
    rightDrive.setVelocity(fabs(rp), percent);
    leftDrive.spin(lp  >= 0 ? forward : reverse);
    rightDrive.spin(rp >= 0 ? forward : reverse);
}

/**
 * Find and track a specific object type
 * Returns true if object found and within target distance
 * Activates tall goal piston if targeting tall goal
 */
bool findAndDriveToObject(int targetClassID, double targetDistance, uint32_t timeout) {
    uint32_t searchStart = Brain.Timer.system();
    bool objectReached = false;
    bool isTallGoal = (targetClassID == TALL_GOAL_ID);
    
    while (!objectReached && (Brain.Timer.system() - searchStart) < timeout) {
        static AI_RECORD local_map;
        jetson_comms.get_data(&local_map);
        
        // Find closest target object
        bool targetFound = false;
        int targetIndex = -1;
        double closestDist = 999999.0;
        
        for (int i = 0; i < local_map.detectionCount; i++) {
            if (local_map.detections[i].classID == targetClassID) {
                double xVal = local_map.detections[i].mapLocation.x;
                double yVal = local_map.detections[i].mapLocation.y;
                double dist = sqrt((xVal * xVal) + (yVal * yVal));
                
                if (dist < closestDist) {
                    closestDist = dist;
                    targetIndex = i;
                    targetFound = true;
                }
            }
        }
        
        if (targetFound && targetIndex != -1) {
            double targetX = local_map.detections[targetIndex].mapLocation.x;
            double targetY = local_map.detections[targetIndex].mapLocation.y;
            double distanceToTarget = sqrt((targetX * targetX) + (targetY * targetY));
            
            // Display tracking info
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("SKILLS: %d blocks, %d goals", skillsBlocksCollected, skillsGoalsScored);
            Brain.Screen.setCursor(2, 1);
            Brain.Screen.print("Tracking ID: %d", targetClassID);
            Brain.Screen.setCursor(3, 1);
            Brain.Screen.print("Distance: %.2f m", distanceToTarget);
            
            // Activate tall goal piston when within 0.15m of tall goal
            if (isTallGoal && distanceToTarget <= 0.15) {
                TallGoalPiston.set(true);
                Brain.Screen.setCursor(4, 1);
                Brain.Screen.print("TALL GOAL PISTON: ON");
            }
            
            // Check if reached target
            if (distanceToTarget <= targetDistance) {
                objectReached = true;
                leftDrive.stop(brake);
                rightDrive.stop(brake);
            } else {
                // Compute obstacle avoidance
                double avoidSteer = skillsObstacleSteer(local_map, targetClassID);
                if (fabs(avoidSteer) > 5.0) {
                    Brain.Screen.setCursor(4, 1);
                    Brain.Screen.print("AVOIDING steer:%.1f", avoidSteer);
                }

                // Drive toward target at 75%, slow near target
                double angleToTarget = atan2(targetX, targetY) * (180.0 / 3.14159265);
                double targetSteer   = angleToTarget * 0.85;
                double baseSpeed     = (distanceToTarget < 1.0)
                    ? NEAR_DRIVE_SPEED + (distanceToTarget * (BASE_DRIVE_SPEED - NEAR_DRIVE_SPEED))
                    : BASE_DRIVE_SPEED;

                skillsApplyDrive(baseSpeed, targetSteer, avoidSteer);
            }
        } else {
            // No target - rotate to search
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("SEARCHING ID: %d", targetClassID);
            
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
    return objectReached;
}

/**
 * Collect a single block
 */
bool collectBlock(int blockClassID) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("COLLECTING BLOCK %d/%d", skillsBlocksCollected + 1, BLOCKS_PER_GOAL);
    
    // Drive to block
    if (findAndDriveToObject(blockClassID, BLOCK_COLLECTION_DISTANCE, BLOCK_SEARCH_TIMEOUT)) {
        // Start intake
        Intake.spin(forward);
        
        // Collect for specified time
        wait(COLLECTION_TIME, seconds);
        
        // Stop intake
        Intake.stop();
        
        skillsBlocksCollected++;
        
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("COLLECTED! Total: %d", skillsBlocksCollected);
        wait(0.5, seconds);
        
        return true;
    }
    
    return false;
}

/**
 * Score blocks at goal
 * Activates tall goal piston if scoring at tall goal
 */
bool scoreAtGoal(int goalClassID) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("GOING TO GOAL!");
    
    bool isTallGoal = (goalClassID == TALL_GOAL_ID);
    
    // Drive to goal
    if (findAndDriveToObject(goalClassID, GOAL_SCORING_DISTANCE, GOAL_SEARCH_TIMEOUT)) {
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("SCORING %d BLOCKS!", skillsBlocksCollected);
        
        // If tall goal, piston should already be activated by findAndDriveToObject
        if (isTallGoal) {
            Brain.Screen.setCursor(3, 1);
            Brain.Screen.print("TALL GOAL MODE");
        }
        
        // Use Loader motor at 50% power for scoring
        Loader.setVelocity(50, percent);
        Loader.spin(forward);
        
        // Run intake in reverse to score
        Intake.spin(reverse);
        
        // Score for 5 seconds
        wait(5, seconds);
        
        // Stop intake
        Intake.stop();
        
        // Reverse loader to return
        Loader.spin(reverse);
        
        // Wait for loader to return
        wait(2, seconds);
        
        // Stop loader
        Loader.stop();
        
        // Retract tall goal piston after scoring
        if (isTallGoal) {
            TallGoalPiston.set(false);
            Brain.Screen.setCursor(4, 1);
            Brain.Screen.print("TALL GOAL PISTON: OFF");
            wait(0.5, seconds);
        }
        
        skillsGoalsScored++;
        skillsBlocksCollected = 0;  // Reset block counter
        
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("SCORED! Goals: %d/%d", skillsGoalsScored, TOTAL_GOALS_TO_SCORE);
        wait(1, seconds);
        
        // Back away from goal at 10%
        leftDrive.setVelocity(BASE_DRIVE_SPEED, percent);
        rightDrive.setVelocity(BASE_DRIVE_SPEED, percent);
        leftDrive.spin(reverse);
        rightDrive.spin(reverse);
        wait(1, seconds);
        leftDrive.stop();
        rightDrive.stop();
        
        return true;
    }
    
    return false;
}

/**
 * Main Skills Autonomous Routine
 * Collects blocks and scores at center goal repeatedly
 */
void skillsAutonomous() {
    // Initialize
    skillsBlocksCollected = 0;
    skillsGoalsScored = 0;
    skillsStartTime = Brain.Timer.system();
    skillsRunning = true;
    
    // Ensure tall goal piston starts retracted
    TallGoalPiston.set(false);
    
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("SKILLS AUTONOMOUS START!");
    wait(1, seconds);
    
    // Main skills loop
    while (skillsRunning && 
           skillsGoalsScored < TOTAL_GOALS_TO_SCORE &&
           (Brain.Timer.system() - skillsStartTime) < SKILLS_TIME_LIMIT) {
        
        // Phase 1: Collect blocks
        while (skillsBlocksCollected < BLOCKS_PER_GOAL &&
               (Brain.Timer.system() - skillsStartTime) < SKILLS_TIME_LIMIT) {
            
            // Try to collect blue blocks first, then red if needed
            if (!collectBlock(BLUE_BLOCK_ID)) {
                collectBlock(RED_BLOCK_ID);
            }
            
            // Check time remaining
            uint32_t timeElapsed = Brain.Timer.system() - skillsStartTime;
            uint32_t timeRemaining = SKILLS_TIME_LIMIT - timeElapsed;
            
            Brain.Screen.setCursor(4, 1);
            Brain.Screen.print("Time: %d sec", timeRemaining / 1000);
            
            // If running out of time, go score what we have
            if (timeRemaining < 15000 && skillsBlocksCollected > 0) {
                break;
            }
        }
        
        // Phase 2: Score at goal (disabled until goal model is trained)
#if GOALS_ENABLED
        if (skillsBlocksCollected > 0) {
            scoreAtGoal(CENTER_GOAL_ID);
        }
#else
        // Goal model not yet trained — just keep collecting
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("SKILLS: ball collect only");
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Collected: %d", skillsBlocksCollected);
        skillsBlocksCollected = 0;  // reset so loop continues
#endif
    }
    
    // Skills complete - ensure piston is retracted
    skillsRunning = false;
    TallGoalPiston.set(false);
    leftDrive.stop(brake);
    rightDrive.stop(brake);
    Intake.stop();
    Outake.stop();
    Loader.stop();
    
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("SKILLS COMPLETE!");
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Goals Scored: %d", skillsGoalsScored);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Total Blocks: %d", skillsGoalsScored * BLOCKS_PER_GOAL);
    Brain.Screen.setCursor(4, 1);
    uint32_t finalTime = (Brain.Timer.system() - skillsStartTime) / 1000;
    Brain.Screen.print("Time: %d seconds", finalTime);
}

/**
 * Alternative: Quick Skills Test (30 seconds)
 * Collects 3 blocks and scores once
 */
void skillsQuickTest() {
    skillsBlocksCollected = 0;
    skillsGoalsScored = 0;
    skillsStartTime = Brain.Timer.system();
    
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("QUICK SKILLS TEST");
    wait(1, seconds);
    
    // Collect 3 blocks
    for (int i = 0; i < 3; i++) {
        if (!collectBlock(BLUE_BLOCK_ID)) {
            collectBlock(RED_BLOCK_ID);
        }
    }
    
    // Score at center goal
    scoreAtGoal(CENTER_GOAL_ID);
    
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("QUICK TEST COMPLETE!");
}
