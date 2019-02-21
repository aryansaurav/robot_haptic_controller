#ifndef GLOBALS_H
#define GLOBALS_H


#include <iostream>

#define NB_DOF_HAND 16
#define NB_DOF_ARM 7
#define NB_DOF_TOT NB_DOF_ARM + NB_DOF_HAND
#define NB_FINGERS 4
#define NB_DOF_FINGER 4

#define USE_LINKLIST
#define VIRTUAL_PID

#define CONTACT_COUNTER_RESET 20 // Threshold to keep contacts to avoid jitter



enum                        CtrlMode { TORQUE = 0, POSITION_PID,
                                       POSITION_PID_STATIC, GRAV_COMP, CART_IMP };

// extern is important here, otherwise it initilizes/define the value, and may try to do it every time this header is included
// I still need to define the variable later. This just makes the variables accessible (declared) to other files.
extern std::ostringstream ossG;
extern std::ostringstream ossGv; // chain of variable length: to display before the
extern std::ostringstream ossDebug; // chain of variable length: to display before the
// second one
extern std::ostringstream ossT;

struct RobotObstacle{
    string name;
    string link_name;
    KDL::Segment segment;
    double radius;
};
#endif // GLOBALS_H
