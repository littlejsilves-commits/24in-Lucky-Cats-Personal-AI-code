/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       skills_autonomous.h                                      */
/*    Author:       AI Robot Team                                            */
/*    Created:      2024                                                     */
/*    Description:  Header for Skills Autonomous Routines                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef SKILLS_AUTONOMOUS_H
#define SKILLS_AUTONOMOUS_H

#include "vex.h"

// Main skills autonomous routine (60 seconds)
void skillsAutonomous();

// Quick test routine (30 seconds, 1 goal)
void skillsQuickTest();

// Helper functions
bool findAndDriveToObject(int targetClassID, double targetDistance, uint32_t timeout);
bool collectBlock(int blockClassID);
bool scoreAtGoal(int goalClassID);

// Global state variables
extern int skillsBlocksCollected;
extern int skillsGoalsScored;
extern uint32_t skillsStartTime;
extern bool skillsRunning;

#endif // SKILLS_AUTONOMOUS_H
