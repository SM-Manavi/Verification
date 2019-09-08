#ifndef UTILITY_H
#define UTILITY_H

#include "Rect_Obstacle.h"
#include "Movement_Direction.h"
#include "State_data.h"

#include <list>


class Utility
{
public:
    Utility();
    void sort_and_remove_duplicates(std::vector<std::string> &vec);
    std::string concatenate_elements_of_vector(std::vector<std::string> &vec) const;
    std::vector<std::string> get_vertices_name(std::vector<Vertex_handle> &vec);
    std::string accumulate_incident_edges_name_of_vertex(Vertex_handle vertex, bool tag_unimportant_vertex);
    bool is_important_vertex(std::vector<std::string> &prev_incident_edge_name, std::vector<std::string> &next_incident_edge_name);
    std::vector<std::pair<TPoint_2, std::string> > get_arrangement_vertices_position_and_name(Arr_with_hist_2 &arr);
    void remove_vector_of_curves_from_arr(Arr_with_hist_2 &arr, std::vector<Curve_handle> &ch);
    std::vector<Named_Circle_2> point_vector_to_circle_vector(std::vector<std::pair<TPoint_2, std::string> > &vec, double radius);
    void add_vector_of_point_as_circle_to_arr(Arr_with_hist_2 &arr, std::vector<Named_Circle_2> &vec);
    std::vector<std::string> arr_intersection_with_segment(Arr_with_hist_2 &arr, Named_Segment_2 &segment, Movement_Direction *sort_direction);
    void remove_duplicate_by_position(std::vector<Vertex_handle> &vec);
    void add_vector_of_curves_to_arrangement(Arr_with_hist_2 &arr, std::vector<Named_Circle_2> *curves);
    std::vector<Point_2> arr_segment_intersection_point(Arr_with_hist_2 &arr, Named_Segment_2 &segment_path);
    void Arr_with_hist_and_consolidated_curve_data_itrerator(Arr_with_hist_2 &arr_) const;
    std::vector<Curve_handle> add_corner_of_rect_as_circle_to_arr(std::vector<Rect_Obstacle> &rects, Arr_with_hist_2 &arr, double radius);
    std::vector<Curve_handle> add_corner_of_rect_as_circle_to_arr(Rect_Obstacle &rect, Arr_with_hist_2 &arr, double radius);
    Segment_2 segment_intersection_of_line_with_rect(Line_2 line, Rect_2 rect) const;
    Segment_2 segment_intersection_of_ray_with_rect(Ray_2 ray, Rect_2 rect) const;
	std::pair<bool, Point_2> segment_segment_intersection(Segment_2 &seg1, Segment_2 &seg2) const;
    bool connectivity_check(Connectivity_hash_map connection);

	std::vector<std::pair<Circle_2, std::string>> point_vector_to_circle_vector_(std::vector<std::pair<TPoint_2, std::string>> &vec,
		double radius);

	std::vector<std::pair<Point_2, std::string>> disk_segment_intersection(
		std::vector<std::pair<Circle_2, std::string>> &vec, Segment_2 &seg);

private :
    void c_check(Connectivity_hash_map &con, std::vector<std::string> &str, std::string robot_name);

};

#endif // UTILITY_H
