#include "Robot.h"

Robot::Robot(Point_2 position, std::string Name_, double radius, const Rect_2 &environment_)
    : disk(position, radius, Name_){

    robot = position;
    name = Name_;

    creat_engulfing_segments(environment_, radius);
}

Robot::~Robot()
{

}

Robot::Robot(const Robot &robot):disk(robot.disk)
{
    this->name = robot.name;
    this->robot = robot.robot;
    this->engulfing_segments = robot.engulfing_segments;
}

void Robot::GoTo(Point_2 &position, Rect_2 &environment)
{
    robot = position;
    double radius = disk.get_radius().doubleValue();
    disk.reset(position, radius, name);
    engulfing_segments.clear();
    creat_engulfing_segments(environment, radius);

}

void Robot::creat_engulfing_segments(const Rect_2 &env, double radius)
{
    BigRat c = radius * 1.5;
    Point_2 p1, p2;

    p1 = Point_2(robot.x()+radius, env[1].y() - c);
    p2 = Point_2(robot.x()+radius, env[2].y() + c);
    engulfing_segments.push_back(
                Segment_Object(Segment_2(p1, p2 ), name + "_right_segment"));

    p1 = Point_2(env[2].x() + c, robot.y()+radius);
    p2 = Point_2(env[3].x() - c, robot.y()+radius);
    engulfing_segments.push_back(
                Segment_Object(Segment_2(p1, p2), name + "_top_segment"));

    p1 = Point_2(robot.x()-radius, env[3].y() + c);
    p2 = Point_2(robot.x()-radius, env[4].y() - c);
    engulfing_segments.push_back(
                Segment_Object(Segment_2(p1, p2), name + "_left_segment"));

    p1 = Point_2(env[4].x() - c, robot.y()-radius);
    p2 = Point_2(env[1].x() + c, robot.y()-radius);
    engulfing_segments.push_back(
                Segment_Object(Segment_2(p1, p2), name + "_bottom_segment"));

}

bool Robot::communication_check(Robot &robot_) const
{
    BigRat dist = CGAL::squared_distance(this->robot,robot_.get_position());
    if(dist<disk.get_Disk().squared_radius())
        return true;
    else
        return false;
}

std::string Robot::get_Name() const { return name; }

bool Robot::operator==(const Robot &robot_)
{
    if(this->name == robot_.name && this->robot == robot_.robot)
        return true;

    return false;
}

bool Robot::operator!=(const Robot &robot)
{
    return !(*this == robot);
}

TPoint_2 Robot::getRobot()
{
    BigRat x = robot.x() , y = robot.y();
    return TPoint_2(x,y);
}

std::vector<Segment_Object>* Robot::getEngulfing_segments()
{
    return &engulfing_segments;
}

Disk_Object& Robot::getDisk() { return disk; }

Segment_Object Robot::get_right_segment()
{
    return engulfing_segments[0];
}

Segment_Object Robot::get_left_segment()
{
    return engulfing_segments[2];
}

Segment_Object Robot::get_top_segment()
{
    return engulfing_segments[1];
}

Segment_Object Robot::get_bottom_segment()
{
    return engulfing_segments[3];
}













