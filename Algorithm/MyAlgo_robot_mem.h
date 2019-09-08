#ifndef MYALGO_ROBOT_MEM_H
#define MYALGO_ROBOT_MEM_H

#include "Movement_Direction.h"
#include "State_data.h"

struct Robot_mem
{
    Robot_mem(){}

public:

    Point_2 getPosition() const { return Point_2(BigRat(x_cord), BigRat(y_cord)); }
    Robot* getRobot() const { return robot; }
    Point_2 getDestination() const { return Point_2(BigRat(x_destination), BigRat(y_destination)); }

    State_Data* getCurrent_state() const { return current_state; }

    void setRobot(Robot *value) { robot = value; }
    void setPosition(const Point_2 &value) { x_cord = value.x().get_str(); y_cord = value.y().get_str(); }
    void setDestination(const Point_2 &value) { x_destination = value.x().get_str();
                                                y_destination = value.y().get_str(); }

    void setCurrent_state(State_Data* value) { current_state = value; }

private:
    Robot* robot;
    std::string x_cord, y_cord; //Current position
    std::string x_destination, y_destination; //My_Algo destionation

    State_Data* current_state;

};

#endif // MYALGO_ROBOT_MEM_H
