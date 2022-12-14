#ifndef _CONTROLLER_HPP_
#define _CONTROLLER_HPP_

#include <Message.hpp>
#include <cstdint>
#include "MessageStructure.hpp"

typedef struct ControllerType
{
    bool emergency_switch;
    //  movement mode
    int8_t movement_mode;
    //  include vector-x,y,theta
    vector3_t movement;
    //  shooter variable
    bool all_reload;
    shooter_t shooter;
    //  face
    int8_t face;
} controller_t;

//  create message
typedef sb::Message<controller_t> Controller;

#endif