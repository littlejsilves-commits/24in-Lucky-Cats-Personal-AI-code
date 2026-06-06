using namespace vex;

extern brain Brain;

extern motor_group leftDrive;
extern motor_group rightDrive;
extern gps GPS;
extern smartdrive Drivetrain;
extern motor Intake;
extern motor Belt;
extern motor Lever1;
extern motor Lever2;
extern motor_group Lever;
extern controller Controller;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
