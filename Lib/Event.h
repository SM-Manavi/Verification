#ifndef EVENT_H
#define EVENT_H

#include "M_Utility.h"
#include "sequence.h"

class Event
{
public:
    Event(std::vector<Robot> *robots_, std::vector<Rect_Obstacle> *obstacles_, Rect_2 *env_);

    std::vector<Point_2> calculate_move_event(Segment_Object segment_path,
                                         Movement_Direction *direction, Robot &robot, Arr_with_hist_2 &arr);

    void add_event_between_any_two_points(std::vector<Point_2>& points, Movement_Direction * dir);

    void delete_point_out_of_the_path(Segment_2 & path, std::vector<Point_2>& points);

//    get_move_vector();
    std::vector<Point_2> get_center_events_position(Segment_Object &segment_path, Robot* robot);

    std::vector<Point_2> get_disk_events_position(Segment_Object &path, Robot &robot);
	std::vector<Point_2> get_disk_events_position(Robot &robot, Segment_Object &path, Movement_Direction *direction);

    Arr_with_hist_2 construct_arr_of_disk(std::vector<Robot>* robots, Robot* robot);

	std::vector<std::pair<Circle_2, std::string>> creat_circle_vec_for_disk_event_detection(Robot* robot);
	std::vector<std::pair<Segment_2, std::string>> Event::Segment_vec_for_disk_event(Robot *robot, std::string move);

    Arr_with_hist_2 construct_disk_arr_for_event_detection(Robot* robot);// create disk arr ....
    void add_disk_with_extra_radius_to_arr(Arr_with_hist_2 &arr, std::vector<Robot> *robots,
                                           Robot* robot);//add_disk_with_higher(bigger)_radius_to_arr

    void adjust_disk_event_to_center_of_robot(std::vector<Point_2> &disk_event, Robot &robot,
                                              Movement_Direction *direction);

    std::vector<Point_2> get_engulfing_segments_events_position(Movement_Direction *direction,
                                                          Segment_2 path, Robot &robot, Arr_with_hist_2 &arr);

    std::pair<std::vector<Point_2>, std::vector<Point_2> > get_vertices_position_that_located_inside_polygons
                                                    (Arr_with_hist_2 &arr,
                                                     Polygon_2 pull_poly, Polygon_2 push_poly);
    std::vector<Point_2> adjust_pulled_segment_event_to_center_of_disk
                         (std::vector<Point_2> &points, Movement_Direction *direction, Robot &robot);

    std::vector<Point_2> adjust_pushed_segment_event_to_center_of_disk
                         (std::vector<Point_2> &points, Movement_Direction *direction, Robot &robot);

    std::vector<Sequence> get_sequences_for_all_robot();

    Sequence get_sequence(Arr_with_hist_2 &disk_arr, Robot &robot);
    std::vector<std::string> down_sequence(Arr_with_hist_2 &disk_arr, Robot &robot);
    std::vector<std::string> up_sequence(Arr_with_hist_2 &disk_arr, Robot &robot);
    std::vector<std::string> left_sequence(Arr_with_hist_2 &disk_arr, Robot &robot);
    std::vector<std::string> right_sequence(Arr_with_hist_2 &disk_arr, Robot &robot);

	Sequence get_sequence(Robot &robot);
	std::vector<std::string> down_sequence(
		std::vector<std::pair<Circle_2, std::string>> &disk_vec, 
		std::vector<std::pair<Segment_2, std::string>> &seg_vec, 
		Robot &robot);

	std::vector<std::string> up_sequence(
		std::vector<std::pair<Circle_2, std::string>> &disk_vec,
		std::vector<std::pair<Segment_2, std::string>> &seg_vec,
		Robot &robot);

	std::vector<std::string> left_sequence(
		std::vector<std::pair<Circle_2, std::string>> &disk_vec,
		std::vector<std::pair<Segment_2, std::string>> &seg_vec,
		Robot &robot);

	std::vector<std::string> right_sequence(
		std::vector<std::pair<Circle_2, std::string>> &disk_vec,
		std::vector<std::pair<Segment_2, std::string>> &seg_vec,
		Robot &robot);

	std::vector<std::pair<Point_2, std::string>> segments_event_points(
		std::vector<std::pair<Segment_2, std::string>> &vec, Robot *robot,
		Segment_2 &path, Movement_Direction *dir);

//    get_segment_event_position();
//    get_segments_event_position();

//    get_sequences();


private:
    Rect_2 *env;
    std::vector<Robot>* robots;
    std::vector<Rect_Obstacle>* obstacles;

    Utility utility;

    double distance;

    Segment_Object get_segmet_path(double distance,
                                   Movement_Direction *direction, Point_2 robot_center);

};

#endif // EVENT_H
