#ifndef ARRANGEMENT_H
#define ARRANGEMENT_H

#include<Robot.h>
#include<Rect_Obstacle.h>
#include<M_Utility.h>

class Arrangement
{
public:
    Arrangement(std::vector<Rect_Obstacle> *obstacles_, std::vector<Robot> *robots_, std::vector<Segment_Object> *env_boundary_)
    {
        obstacles = obstacles_;
        robots = robots_;
        env_boundary = env_boundary_;
    }

    Arr_with_hist_2 creat_arr_without_(Robot *robot);
    Arr_with_hist_2 creat_arr_of_all_object();
    std::vector<std::string> *get_sorted_faces_names(Arr_with_hist_2 &arr);
    std::vector<std::string> *get_arrangement_faces_names(Arr_with_hist_2 &arr);

private:
    std::vector<Rect_Obstacle> *obstacles;
    std::vector<Robot> *robots;
    std::vector<Segment_Object> *env_boundary;
    Arr_with_hist_2 arr;

    void add_obstacles_to_arrangement();

    void add_obstacle_boundaries_to_arrangement(Rect_Obstacle &obstacle);
    void add_obstacle_engulfing_segments_to_arrangement(Rect_Obstacle &obstacle);
    void add_all_robots_to_arrangement();
    void add_robot_engulfing_segments_to_arrangement(Robot &robot);
    void add_robot_to_arrangement(Robot &robot);
    void add_vector_of_segment_to_arrangement(std::vector<Segment_Object> *segments);
    void add_environment_boundary_to_arr();
    void add_all_robots_to_arrangement_exept(Robot *ex_robot);

    std::vector<std::string> Ccb_curves_name(Ccb_halfedge_const_circulator circ);
};
#endif // ARRANGEMENT_H
