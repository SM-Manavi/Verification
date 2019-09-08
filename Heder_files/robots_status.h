#ifndef ROBOTS_STATUS_H
#define ROBOTS_STATUS_H

#include "alpha_robot_mem.h"

struct Status
{
    Status() {}

    void setStatus(State_Data* state, std::vector<Robot_mem> *robots)
    {
        current_state = state;
        status = *robots;
    }

    void setState(State_Data *state)
    {
        current_state = state;
    }
    State_Data* getState() {return current_state;}
    std::vector<Robot_mem>* getRobot_status(){return &status;}

private :
    std::vector<Robot_mem> status;
    State_Data *current_state; 

};


#endif // ROBOTS_STATUS_H

