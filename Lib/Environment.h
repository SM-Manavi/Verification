#ifndef ENVIROMENT_H
#define ENVIROMENT_H

//#include <stdio.h>      /* printf, fopen */
//#include <stdlib.h>     /* exit, EXIT_FAILURE */

#include "Robot.h"
#include "Rect_Obstacle.h"
#include "state_space.h"
#include "Arrangement.h"
#include "robots_status.h"
#include "Event.h"

struct Environment
{

public:

    Environment(Rect_2 boundary, double radius_);

	bool check_move_condition(Robot_mem &mem, int &k);
	std::vector<std::string> scan_intersection(std::vector<std::string> &v1, std::vector<std::string> &v2);

    void add_obstacle(Rect_2 obstacle);

    int add_robot(Point_2 &robot_position);

    bool is_R1_Rect_Completely_inside_R2_Rect(Rect_2 &R1, Rect_2 &R2); //R2 contains R1

    Point_2 get_valid_destination(Point_2 source, Point_2 destination); // valid destination and 
    // bejaye segment begam arrow
    Point_2 get_first_segment_intersection_with_Rect(const Segment_2 seg,
                                                   const Rect_2 rect, std::string obj_type); //segment first collission point with rect

    std::vector<Segment_Object> get_segments_boundary() const;//Delete

    Point_2 get_upper_right_corner() const;
    Point_2 get_lower_left_corner() const;

    void creat_virtual_rect();


    std::vector<Segment_Object> getImaginary_segment_boundary() const;

    Rect_2 getEnvironment_Rect() const;

    std::vector<Rect_Obstacle> getObstacles() const;

    std::vector<Robot>* getRobots() ;

    Rect_2 get_virtual_rect () const;

    void move_robot(Robot *robot, Point_2 &new_position);

    bool check_collision(Robot *robot, Point_2 destination);
    std::pair<bool, Point_2> check_collision(Robot* robot , Point_2 destination, Movement_Direction *dir);

    Connectivity_hash_map get_connectivity_matrix();
    std::vector<std::string> check_connections(Robot &robot_);
    State_Data *current_state();
    int getNum_of_neighbor(Robot *robot);
	int getNum_of_State();
    void jump(Status &new_situation);
	// int move(Robot *robot, double distance, Movement_Direction *direction);
	int move(Robot_mem *robot_mem, int alpha, double distance, Movement_Direction *direction);
	
    void initialization();
    Segment_Object get_segmet_path(double distance, Movement_Direction *direction, Point_2 robot_center);
    void createLTS();
	double getRadius();
private:

    Rect_2 env_boundary;
    std::vector<Segment_Object> env_segment_boundary;

//   Imaginary rectangle makes state computation easy
    Rect_2 virtual_rect;
    std::vector<Segment_Object> virtual_rect_segment_boundary;

    std::vector<Rect_Obstacle> obstacles;
    std::vector<Robot> robots;

    Arrangement *arrangment;
    Arr_with_hist_2 arr;

    State_space state_space;

    double radius;

    bool has_robot_valid_position(Robot &robot) const;//robot position correctness
    bool has_New_hole_valid_position(Rect_2 &NewHole_boundary);//obstacle position correctness

    void change_robots_position(Status &new_pos);
    State_Data *create_state();
};



#endif // ENVIROMENT_H
