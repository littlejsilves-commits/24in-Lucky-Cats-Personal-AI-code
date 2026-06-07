/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp - SIMPLE TEST VERSION                            */
/*    Author:       Lucky cATS                                                */
/*    Created:      06/06/2026                                                */
/*    Description:  Simplified test - detect one blue ball and drive to it   */
/*                                                                            */
/*    TEST SETUP:                                                             */
/*      1. Place ONE blue ball in front of robot                             */
/*      2. Robot will detect, drive toward it, and stop                      */
/*      3. Watch console for diagnostics                                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "ai_functions.h"
#include <cmath>

using namespace vex;

/*============================================================================*/
/*                      SIMPLE TEST PARAMETERS                                */
/*============================================================================*/

#define BALL_BLUE_ID        0       // Blue ball class ID
#define DRIVE_SPEED         30.0    // Slow speed for testing (30%)
#define STOP_DISTANCE       0.3     // Stop 30cm from ball (meters)
#define MANAGER_ROBOT       1       // 1 = Manager, 0 = Worker

/*============================================================================*/
/*                    END OF PARAMETERS                                       */
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

/*---------------------------------------------------------------------------*/
/*  Simple display function                                                  */
/*---------------------------------------------------------------------------*/
void displayInfo(const char* status, int detections, double distance) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("STATUS: %s", status);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Detections: %d", detections);
    if (distance > 0) {
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Distance: %.2fm", distance);
    }
}

/*---------------------------------------------------------------------------*/
/*  Main Loop - Simple Test                                                  */
/*---------------------------------------------------------------------------*/
int main() {
    static AI_RECORD local_map;
    int32_t loop_time = 33;  // 30Hz update rate

    thread t1(dashboardTask);
    
    leftDrive.setStopping(brake);
    rightDrive.setStopping(brake);

    printf("\n=== SIMPLE BALL DETECTION TEST ===\n");
    printf("Looking for BLUE BALL (classID = %d)\n", BALL_BLUE_ID);
    printf("Will drive to %.2fm and stop\n\n", STOP_DISTANCE);
    
    while (1) {
        // Get camera data
        jetson_comms.get_data(&local_map);
        
        // Look for blue ball
        bool foundBall = false;
        double ballX = 0.0;
        double ballY = 0.0;
        double distance = 0.0;
        
        for (int i = 0; i < local_map.detectionCount; i++) {
            if (local_map.detections[i].classID == BALL_BLUE_ID) {
                ballX = local_map.detections[i].mapLocation.x;
                ballY = local_map.detections[i].mapLocation.y;
                distance = sqrt(ballX * ballX + ballY * ballY);
                foundBall = true;
                
                // Log to console
                printf("FOUND BALL: X=%.2f Y=%.2f Dist=%.2fm\n", ballX, ballY, distance);
                break;  // Only track first blue ball found
            }
        }
        
        if (foundBall) {
            // Calculate angle to ball
            double angleToTarget = atan2(ballX, ballY) * (180.0 / M_PI);
            
            if (distance > STOP_DISTANCE) {
                // Drive toward ball
                displayInfo("DRIVING TO BALL", local_map.detectionCount, distance);
                
                // Simple proportional steering
                double steer = angleToTarget * 0.8;  // Steering gain
                
                double leftSpeed = DRIVE_SPEED + steer;
                double rightSpeed = DRIVE_SPEED - steer;
                
                // Clamp speeds
                if (leftSpeed > 100) leftSpeed = 100;
                if (leftSpeed < -100) leftSpeed = -100;
                if (rightSpeed > 100) rightSpeed = 100;
                if (rightSpeed < -100) rightSpeed = -100;
                
                leftDrive.setVelocity(fabs(leftSpeed), percent);
                rightDrive.setVelocity(fabs(rightSpeed), percent);
                leftDrive.spin(leftSpeed >= 0 ? forward : reverse);
                rightDrive.spin(rightSpeed >= 0 ? forward : reverse);
                
                printf("  -> Angle: %.1f deg, Steer: %.1f, L: %.0f R: %.0f\n", 
                       angleToTarget, steer, leftSpeed, rightSpeed);
                
            } else {
                // Close enough - STOP!
                leftDrive.stop(brake);
                rightDrive.stop(brake);
                displayInfo("REACHED BALL!", local_map.detectionCount, distance);
                printf("  -> STOPPED at %.2fm\n", distance);
                Controller.rumble("-");  // Long rumble when reached
            }
            
        } else {
            // No ball detected - stop and wait
            leftDrive.stop(brake);
            rightDrive.stop(brake);
            displayInfo("SEARCHING...", local_map.detectionCount, 0);
            
            if (local_map.detectionCount > 0) {
                printf("NO BLUE BALL (saw %d other objects)\n", local_map.detectionCount);
            }
        }
        
        // Request new camera data
        jetson_comms.request_map();
        this_thread::sleep_for(loop_time);
    }
}
