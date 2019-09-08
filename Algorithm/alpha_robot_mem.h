#ifndef ALPHA_ROBOT_MEM_H
#define ALPHA_ROBOT_MEM_H

#include "Movement_Direction.h"
#include "State_data.h"

struct Robot_mem
{
    Robot_mem(){}

public:

    Movement_Direction * getCurrent_dir() const { return toDirecrion_type(current_dir_name); }
    //Movement_Direction * getCurrent_dir() const { return current_dir; }
    int getCurrent_neighbor() const { return current_neighbor; }
    Point_2 getPosition() const { return Point_2(BigRat(x_cord), BigRat(y_cord)); }
    Robot* getRobot() const { return robot; }
	std::string getMode() const { return mode; }
	bool getCollision() const { return collision; }
    //State_Data* getCurrent_state() const { return current_state; }

    void setRobot(Robot *value) { robot = value; }
    void setCurrent_dir(Movement_Direction* value) { current_dir_name = value->get_direction_name(); delete value; }
    //void setCurrent_dir(Movement_Direction* value) { current_dir = value; }
    void setCurrent_neighbor(const int &value) { current_neighbor = value; }
    void setPosition( Point_2 &value) { x_cord = value.x().get_str(); y_cord = value.y().get_str(); }
	void setMode(const std::string &value) { mode = value; }
	void setCollision(const bool &collide) { collision = collide; }
    //void setCurrent_state(State_Data* value) { current_state = value; }

private:
    Robot* robot;
    std::string current_dir_name;
    //Movement_Direction* current_dir;
    int current_neighbor;
    std::string x_cord, y_cord; //Current position
    std::string mode="forward";
	bool collision = false;

	friend bool operator<(const Robot_mem& lhs, const Robot_mem& rhs) 
			{ return lhs.getRobot()->get_Name() < rhs.getRobot()->get_Name(); }

    //State_Data* current_state;
};

#endif // ALPHA_ROBOT_MEM_H
