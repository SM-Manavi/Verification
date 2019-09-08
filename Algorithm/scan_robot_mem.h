#ifndef SCAN_ROBOT_MEM_H
#define SCAN_ROBOT_MEM_H

#include "Movement_Direction.h"
#include "State_data.h"

struct Robot_mem
{
	Robot_mem() {}

public:

	Point_2 getPosition() const { return Point_2(position); }
	Robot* getRobot() const { return robot; }
	Movement_Direction * getCurrent_dir() const { return toDirecrion_type(current_dir_name); }
//	State_Data* getCurrent_state() const { return current_state; }

	void setRobot(Robot *value) { robot = value; }
	//void setPosition(const Point_2 &value) { x_cord = value.x().get_str(); y_cord = value.y().get_str(); 
	void setPosition(const Point_2 &value) { position = value; }
	void setCurrent_dir(Movement_Direction* value) { current_dir_name = value->get_direction_name(); delete value; }
//	void setCurrent_state(State_Data* value) { current_state = value; }

private:
	Robot * robot;
	//std::string x_cord, y_cord; //Current position
	Point_2 position;
	std::string current_dir_name;

//	State_Data* current_state;

	friend bool operator<(const Robot_mem& lhs, const Robot_mem& rhs)
	{
		return lhs.getRobot()->get_Name() < rhs.getRobot()->get_Name();
	}
	
};

#endif // SCAN_ROBOT_MEM_H
