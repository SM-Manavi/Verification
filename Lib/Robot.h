#ifndef ROBOT_H
#define ROBOT_H

#include "Segment_obj.h"
#include "Disk_obj.h"

class Robot
{
public:
    Robot(Point_2 position, std::string Name_, double radius, const Rect_2 &environment_);
    ~Robot();

    Robot(const Robot &robot);

    Point_2 get_position() const { return robot; }
    void GoTo(Point_2 &position, Rect_2 &environment);

    Disk_Object& getDisk();

    std::vector<Segment_Object>* getEngulfing_segments();

    void setRobot_handle(const Vertex_handle &value);

    TPoint_2 getRobot();

    Vertex_handle getRobot_handle() const;

    void set_Disk_Handle(const Curve_handle disk_handle);
    Curve_handle get_Disk_Handle() const;

    bool communication_check(Robot &robot_) const; //communication check

    std::string get_Name() const;

    bool operator==(const Robot& robot_);
    bool operator!=(const Robot& robot);

    Segment_Object get_right_segment();
    Segment_Object get_left_segment();
    Segment_Object get_top_segment();
    Segment_Object get_bottom_segment();

private:

    void creat_engulfing_segments(const Rect_2 &env, double radius);

private:

    std::string name;
    Point_2 robot;
    Disk_Object disk;
    std::vector<Segment_Object> engulfing_segments;
};

#endif // ROBOT_H
