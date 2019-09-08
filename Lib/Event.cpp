#include "Event.h"

Event::Event(std::vector<Robot>* robots_, std::vector<Rect_Obstacle>* obstacles_, Rect_2 *env_)
{
    env = env_;
    robots = robots_;
    obstacles = obstacles_;
}

std::vector<Point_2> Event::calculate_move_event(Segment_Object segment_path,
                                            Movement_Direction *direction,
                                            Robot &robot,
                                            Arr_with_hist_2 &arr)
{
    std::vector<Point_2> events;
    std::vector<Point_2> disk_center_events, disk_events, engulfing_segments_events;
    if (segment_path.get_Segment().source() != segment_path.get_Segment().target())
    {
		disk_center_events = get_center_events_position(segment_path, &robot);

        //disk_events = get_disk_events_position(segment_path, robot);
		disk_events = get_disk_events_position(robot, segment_path, direction);

        engulfing_segments_events = get_engulfing_segments_events_position(
                                                              direction,
                                                              segment_path.get_Segment(),
                                                              robot,
                                                              arr);
        std::vector<Point_2> disk_and_segment = disk_events;
		disk_and_segment.insert(disk_and_segment.end(),
			engulfing_segments_events.begin(), engulfing_segments_events.end());

		direction->sort_event(disk_and_segment);
		disk_and_segment.erase(std::unique(disk_and_segment.begin(), disk_and_segment.end()), disk_and_segment.end());
		delete_point_out_of_the_path(segment_path.get_Segment(), disk_and_segment);

		if (disk_and_segment.size() != 0)
		{
			if (disk_and_segment.back() == segment_path.get_Segment().target())
				disk_and_segment.pop_back();
			//Disk and engulging segments events record the moment of the collision. Adding events between any
			//two points lead to creating new region/regions.
			add_event_between_any_two_points(disk_and_segment, direction);
		}

		events = disk_center_events;
		events.insert(events.end(), disk_and_segment.begin(), disk_and_segment.end());
		events.push_back(segment_path.get_Segment().target());

		delete_point_out_of_the_path(segment_path.get_Segment(), events);
		direction->sort_event(events);
		events.erase(std::unique(events.begin(), events.end()), events.end());

		if (events.front() == segment_path.get_Segment().source())
			events.erase(events.begin());
	}
	else
	{
		events.push_back(segment_path.get_Segment().target());
	}

    //events.erase(std::unique(events.begin(), events.end()), events.end());

    return events;

}

void Event::add_event_between_any_two_points(std::vector<Point_2> &points, Movement_Direction* dir)
{
    std::string dir_name = dir->get_direction_name();
	BigRat temp;
    std::vector<Point_2> new_points;
	for (auto it = points.begin(); it != points.end(); ++it)
	{
		if (next(it) == points.end())
			break;

		if (dir_name == "Left" || dir_name == "Right")
		{
			temp = (it->x() + next(it)->x()) / 2;
			new_points.push_back(Point_2(temp, it->y()));
		}
		else if (dir_name == "Up" || dir_name == "Down")
		{
			temp = (it->y() + next(it)->y()) / 2;
			new_points.push_back(Point_2(it->x(), temp));
		}
	}

	points.insert(points.end(), new_points.begin(), new_points.end());

	dir->sort_event(points);
}

void Event::delete_point_out_of_the_path(Segment_2 &path, std::vector<Point_2> &points)
{
	for (auto point = points.begin(); point != points.end();)
		if (!path.collinear_has_on(*point))
			point = points.erase(point);
		else
			point++;
}

void Event::adjust_disk_event_to_center_of_robot(std::vector<Point_2> &disk_event,
                                                 Robot &robot, Movement_Direction *direction)
{
    std::pair<Point_2, Point_2> diameter_on_direction =
            direction->get_diameter_on_direction(robot);

    std::vector<Point_2>::iterator point;
    for(point = disk_event.begin(); point != disk_event.end(); ++point)
    {
//        *point = direction->adjust_disk_event_to_center_of_disk(*point, diameter_on_direction,
//                                                                   robot.getDisk().get_radius());
    }
}


std::vector<Point_2> Event::get_center_events_position(Segment_Object &segment_path, Robot* robot)
{
    std::vector<Point_2> path_intersect;

    path_intersect = utility.arr_segment_intersection_point(construct_arr_of_disk(robots, robot),
                                                                       segment_path.get_Named_Segment());

    return path_intersect;
}

std::vector<Point_2> Event::get_engulfing_segments_events_position(Movement_Direction *direction,
																   Segment_2 path, Robot &robot,
																   Arr_with_hist_2 &arr)
{

    Polygon_2 pulled_segment_motion_area =
                   direction->get_pulled_segment_motion_area_as_polygon(path, robot);
    
    Polygon_2 pushed_segment_motion_area =
                   direction->get_pushed_segment_motion_area_as_polygon(path, robot);
    
    std::pair<std::vector<Point_2>,std::vector<Point_2>> pulled_and_pushed_engulfing_segments_events =
             get_vertices_position_that_located_inside_polygons(arr,
                                                                pulled_segment_motion_area,
                                                                pushed_segment_motion_area);

    std::vector<Point_2> pulled_segment_event = adjust_pulled_segment_event_to_center_of_disk
                                                (pulled_and_pushed_engulfing_segments_events.first,direction,robot);

    std::vector<Point_2> pushed_segment_event = adjust_pushed_segment_event_to_center_of_disk
                                                (pulled_and_pushed_engulfing_segments_events.second,direction,robot);

    std::vector<Point_2> engulfing_segments_events = pulled_segment_event;
    engulfing_segments_events.insert( engulfing_segments_events.end(),
                                      pushed_segment_event.begin(),
                                      pushed_segment_event.end() );
    return engulfing_segments_events;
}

std::vector<Point_2> Event::adjust_pulled_segment_event_to_center_of_disk
                        (std::vector<Point_2> &points, Movement_Direction *direction, Robot &robot)
{
    std::vector<Point_2> adjusted;
    std::vector<Point_2>::iterator point_;
    for(point_ = points.begin(); point_ != points.end(); ++point_)
        adjusted.push_back(direction->adjust_pulled_segment_event_to_center_of_disk
                                                                     (*point_,robot) );

    return adjusted;
}


std::vector<Point_2> Event::adjust_pushed_segment_event_to_center_of_disk
                        (std::vector<Point_2> &points, Movement_Direction *direction, Robot &robot)
{
    std::vector<Point_2> adjusted;
    std::vector<Point_2>::iterator point_;
    for(point_ = points.begin(); point_ != points.end(); ++point_)
        adjusted.push_back(direction->adjust_pushed_segment_event_to_center_of_disk
                                                                      (*point_,robot) );
    return adjusted;
}

std::pair<std::vector<Point_2>,std::vector<Point_2>> Event::get_vertices_position_that_located_inside_polygons
                                                        (Arr_with_hist_2 &arr,
                                                         Polygon_2 pull_poly, Polygon_2 push_poly)
{ 
    std::vector<Point_2> pull_segment_events,push_segment_events;
    Arr_with_hist_2::Vertex_iterator vit;
    for(vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit)
    {
        Point_2 p = toPoint_2(vit->point());

        if(!pull_poly.has_on_unbounded_side(p))
            pull_segment_events.push_back(p);

        if(!push_poly.has_on_unbounded_side(p))
            push_segment_events.push_back(p);
    }

    return make_pair(pull_segment_events,push_segment_events);
}


//If robots have different radius this method needs to be changed.
std::vector<Sequence> Event::get_sequences_for_all_robot()
{
    std::vector<Sequence> sequences;
    std::vector<Robot>::iterator robot_;
	for (robot_ = robots->begin(); robot_ != robots->end(); ++robot_)
	{
		//Arr_with_hist_2 disk_arr = construct_disk_arr_for_event_detection(&(*robot_));
		//sequences.push_back(get_sequence(disk_arr, *robot_));


		sequences.push_back(get_sequence(*robot_));
	}
       

    return sequences;
}


Sequence Event::get_sequence(Arr_with_hist_2 &disk_arr, Robot &robot)
{
    Sequence sequence(robot.get_Name());

    sequence.set_up_sequence(up_sequence(disk_arr, robot));
    sequence.set_down_sequence(down_sequence(disk_arr, robot));

    sequence.set_left_sequence(left_sequence(disk_arr, robot));
    sequence.set_right_sequence(right_sequence(disk_arr, robot));

    //sequence.print();

    return sequence;
}

Sequence Event::get_sequence(Robot &robot)
{
	Sequence sequence(robot.get_Name());

	std::vector<std::pair<Circle_2, std::string>> disk_vec =
		creat_circle_vec_for_disk_event_detection(&robot);

	std::vector<std::pair<Segment_2, std::string>> seg_vec =
		Segment_vec_for_disk_event(&robot, "vertical");

	sequence.set_up_sequence(up_sequence(disk_vec, seg_vec, robot));
	sequence.set_down_sequence(down_sequence(disk_vec, seg_vec, robot));

	seg_vec = Segment_vec_for_disk_event(&robot, "horizontal");

	sequence.set_left_sequence(left_sequence(disk_vec, seg_vec, robot));
	sequence.set_right_sequence(right_sequence(disk_vec, seg_vec, robot));

	//sequence.print();

	return sequence;
}

std::vector<std::string> Event::left_sequence(Arr_with_hist_2 &disk_arr, Robot &robot)
{
    Point_2 q(robot.get_position().x()-robot.getDisk().get_radius(),
              robot.get_position().y());
//    Create a ray with source robot position and passing through point q.
    Ray_2 ray(robot.get_position(), q);
    Named_Segment_2  segment(utility.segment_intersection_of_ray_with_rect(ray, *env), "");
	if (segment.source() == segment.target())
	{
		std::vector<std::string> null = { "" };
		return null;
	}
    return utility.arr_intersection_with_segment(disk_arr, segment, new Left_Direction());
}

std::vector<std::string> Event::right_sequence(Arr_with_hist_2 &disk_arr, Robot &robot)
{
    Point_2 q(robot.get_position().x()+robot.getDisk().get_radius(),
              robot.get_position().y());
//    introduces a ray with source robot position and passing through point q.
    Ray_2 ray(robot.get_position(), q);
    Named_Segment_2  segment(utility.segment_intersection_of_ray_with_rect(ray, *env), "");
	if (segment.source() == segment.target())
	{
		std::vector<std::string> null = { "" };
		return null;
	}
    return utility.arr_intersection_with_segment(disk_arr, segment, new Right_Direction());
}
std::vector<std::string> Event::up_sequence(Arr_with_hist_2 &disk_arr, Robot &robot)
{
    Point_2 q(robot.get_position().x(),
              robot.get_position().y()+robot.getDisk().get_radius());
//    introduces a ray with source robot position and passing through point q.
    Ray_2 ray(robot.get_position(), q);
    Named_Segment_2  segment(utility.segment_intersection_of_ray_with_rect(ray, *env), "");
	if (segment.source() == segment.target())
	{
		std::vector<std::string> null = { "" };
		return null;
	}
    return utility.arr_intersection_with_segment(disk_arr, segment, new Up_Direction());
}
std::vector<std::string> Event::down_sequence(Arr_with_hist_2 &disk_arr, Robot &robot)
{
    Point_2 q(robot.get_position().x(),
              robot.get_position().y()-robot.getDisk().get_radius());
//    introduces a ray with source robot position and passing through point q.
    Ray_2 ray(robot.get_position(), q);
    Named_Segment_2  segment(utility.segment_intersection_of_ray_with_rect(ray, *env), "");
	if (segment.source() == segment.target())
	{
		std::vector<std::string> null = { "" };
		return null;
	}
    return utility.arr_intersection_with_segment(disk_arr, segment, new Down_Direction());
}

std::vector<std::string> Event::left_sequence(
	std::vector<std::pair<Circle_2, std::string>> &disk_vec,
	std::vector<std::pair<Segment_2, std::string>> &seg_vec,
	Robot &robot)
{
	Point_2 q(robot.get_position().x() - robot.getDisk().get_radius(),
		robot.get_position().y());
	// Create a ray with source robot position and passing through point q.
	Ray_2 ray(robot.get_position(), q);
	Segment_2  segment = utility.segment_intersection_of_ray_with_rect(ray, *env);

	if (segment.source() == segment.target())
	{
		std::vector<std::string> null = { "" };
		return null;
	}

	std::vector<std::pair<Point_2, std::string>> events = 
		utility.disk_segment_intersection(disk_vec, segment);

	Movement_Direction *left = new Left_Direction();

	std::vector<std::pair<Point_2, std::string>> temp =
		segments_event_points(seg_vec, &robot, segment, left);

	events.insert(events.end(), temp.begin(), temp.end());

	left->sort_event(events);

	std::vector<std::string> events_names;

	for (auto it = events.begin(); it != events.end(); ++it)
		events_names.push_back(it->second);

	delete left;

	return events_names;
}

std::vector<std::string> Event::right_sequence(
	std::vector<std::pair<Circle_2, std::string>> &disk_vec,
	std::vector<std::pair<Segment_2, std::string>> &seg_vec,
	Robot &robot)
{
	Point_2 q(robot.get_position().x() + robot.getDisk().get_radius(),
		robot.get_position().y());
	//    introduces a ray with source robot position and passing through point q.
	Ray_2 ray(robot.get_position(), q);
	Segment_2  segment = utility.segment_intersection_of_ray_with_rect(ray, *env);

	if (segment.source() == segment.target())
	{
		std::vector<std::string> null = { "" };
		return null;
	}

	std::vector<std::pair<Point_2, std::string>> events =
		utility.disk_segment_intersection(disk_vec, segment);

	Movement_Direction *right = new Right_Direction();

	std::vector<std::pair<Point_2, std::string>> temp =
		segments_event_points(seg_vec, &robot, segment, right);

	events.insert(events.end(), temp.begin(), temp.end());

	right->sort_event(events);

	std::vector<std::string> events_names;

	for (auto it = events.begin(); it != events.end(); ++it)
		events_names.push_back(it->second);

	delete right;
	return events_names;
}
std::vector<std::string> Event::up_sequence(
	std::vector<std::pair<Circle_2, std::string>> &disk_vec,
	std::vector<std::pair<Segment_2, std::string>> &seg_vec,
	Robot &robot)
{
	Point_2 q(robot.get_position().x(),
		robot.get_position().y() + robot.getDisk().get_radius());
	//    introduces a ray with source robot position and passing through point q.
	Ray_2 ray(robot.get_position(), q);
	Segment_2  segment = utility.segment_intersection_of_ray_with_rect(ray, *env);

	if (segment.source() == segment.target())
	{
		std::vector<std::string> null = { "" };
		return null;
	}

	std::vector<std::pair<Point_2, std::string>> events =
		utility.disk_segment_intersection(disk_vec, segment);

	Movement_Direction *up = new Up_Direction();

	std::vector<std::pair<Point_2, std::string>> temp =
		segments_event_points(seg_vec, &robot, segment, up);

	events.insert(events.end(), temp.begin(), temp.end());

	up->sort_event(events);

	std::vector<std::string> events_names;

	for (auto it = events.begin(); it != events.end(); ++it)
		events_names.push_back(it->second);

	delete up;
	return events_names;
}
std::vector<std::string> Event::down_sequence(
	std::vector<std::pair<Circle_2, std::string>> &disk_vec,
	std::vector<std::pair<Segment_2, std::string>> &seg_vec,
	Robot &robot)
{
	Point_2 q(robot.get_position().x(),
		robot.get_position().y() - robot.getDisk().get_radius());
	//    introduces a ray with source robot position and passing through point q.
	Ray_2 ray(robot.get_position(), q);
	Segment_2  segment = utility.segment_intersection_of_ray_with_rect(ray, *env);

	if (segment.source() == segment.target())
	{
		std::vector<std::string> null = { "" };
		return null;
	}

	std::vector<std::pair<Point_2, std::string>> events =
		utility.disk_segment_intersection(disk_vec, segment);

	Movement_Direction *down = new Right_Direction();

	std::vector<std::pair<Point_2, std::string>> temp =
		segments_event_points(seg_vec, &robot, segment, down);

	events.insert(events.end(), temp.begin(), temp.end());

	down->sort_event(events);

	std::vector<std::string> events_names;

	for (auto it = events.begin(); it != events.end(); ++it)
		events_names.push_back(it->second);

	delete down;
	return events_names;
}


std::vector<Point_2> Event::get_disk_events_position(Segment_Object &path, Robot &robot)
{
    std::vector<Point_2> disk_event_position;

    Arr_with_hist_2 disk_arr;
    disk_arr = construct_disk_arr_for_event_detection(&robot);// construct disk arr for disk event

    std::vector<Curve_handle> handle =utility.add_corner_of_rect_as_circle_to_arr(
                                                      *obstacles, disk_arr,
                                                      robot.getDisk().get_radius().doubleValue());

    disk_event_position = utility.arr_segment_intersection_point(disk_arr,
                                                                 path.get_Named_Segment());

    return disk_event_position;
}


std::vector<Point_2> Event::get_disk_events_position(Robot &robot, Segment_Object &path, Movement_Direction *direction)
{
	std::vector<Point_2> disk_event_position;

	std::vector<std::pair<Circle_2, std::string>> disk_vec =
		creat_circle_vec_for_disk_event_detection(&robot);

	std::vector<std::pair<Point_2, std::string>> events =
		utility.disk_segment_intersection(disk_vec, path.get_Segment());


	for (auto it = events.begin(); it != events.end(); ++it)
		disk_event_position.push_back(it->first);

	return disk_event_position;
}

Arr_with_hist_2 Event::construct_arr_of_disk(std::vector<Robot> *robots, Robot* robot)
{
	Arr_with_hist_2 disk_arr;

	std::vector<Robot>::iterator robot_;
	for (robot_ = robots->begin(); robot_ != robots->end(); ++robot_)
	{
		if (*robot_ != *robot)
		{
			CGAL::insert(disk_arr, robot_->getDisk().get_Named_Disk());
		}
	}

	return disk_arr;
}

std::vector<std::pair<Circle_2, std::string>> Event::creat_circle_vec_for_disk_event_detection(Robot* robot)
{
	Arr_with_hist_2 disk_arr = construct_arr_of_disk(robots, robot);

	double radius = robot->getDisk().get_radius().doubleValue();
	std::vector<std::pair<Circle_2, std::string>>  vec =
		utility.point_vector_to_circle_vector_(
			utility.get_arrangement_vertices_position_and_name(disk_arr), radius);

	for (auto obs = obstacles->begin(); obs != obstacles->end(); ++obs)
		for (int i = 1; i <= 4; i++)
		{
			Circle_2 disk(obs->getObstacle_Rect()[i], robot->getDisk().get_Disk().squared_radius());
			vec.push_back(std::make_pair(disk, obs->get_Name()));
		}

	for (auto robot_ = robots->begin(); robot_ != robots->end(); ++robot_)
	{
		if (*robot_ != *robot)
		{
			Circle_2 disk(robot_->get_position(),
				std::pow(robot_->getDisk().get_radius().doubleValue() + 
					robot->getDisk().get_radius().doubleValue(), 2));

			vec.push_back(std::make_pair(disk, robot_->get_Name()));
		}
	}

	return vec;
}

std::vector<std::pair<Segment_2, std::string>> Event::Segment_vec_for_disk_event(Robot *robot, std::string move)
{
	std::vector<std::pair<Segment_2, std::string>> vec;
	Segment_Object seg;

		for (auto robot_ = robots->begin(); robot_ != robots->end(); ++robot_)
		{
			if (*robot_ != *robot)
			{
				if (move == "vertical")
				{
					seg = robot_->get_bottom_segment();
					vec.push_back(std::make_pair(seg.get_Segment(), seg.getName()));

					seg = robot_->get_top_segment();
					vec.push_back(std::make_pair(seg.get_Segment(), seg.getName()));
				}

				else if (move == "horizontal")
				{
					seg = robot_->get_left_segment();
					vec.push_back(std::make_pair(seg.get_Segment(), seg.getName()));

					seg = robot_->get_right_segment();
					vec.push_back(std::make_pair(seg.get_Segment(), seg.getName()));
				}
			}
		}

		for (auto obs = obstacles->begin(); obs != obstacles->end(); ++obs)
		{
			if (move == "vertical")
			{
				seg = obs->get_top_engulfing_segment();
				vec.push_back(std::make_pair(seg.get_Segment(), seg.getName()));

				seg = obs->get_bottom_engulfing_segment();
				vec.push_back(std::make_pair(seg.get_Segment(), seg.getName()));

				seg = obs->get_top_side_segment();
				vec.push_back(std::make_pair(seg.get_Segment(), seg.getName()));

				seg = obs->get_bottom_side_segment();
				vec.push_back(std::make_pair(seg.get_Segment(), seg.getName()));
			}
			else if (move == "horizontal")
			{
				seg = obs->get_left_engulfing_segment();
				vec.push_back(std::make_pair(seg.get_Segment(), seg.getName()));

				seg = obs->get_right_engulfing_segment();
				vec.push_back(std::make_pair(seg.get_Segment(), seg.getName()));

				seg = obs->get_left_side_segment();
				vec.push_back(std::make_pair(seg.get_Segment(), seg.getName()));

				seg = obs->get_right_side_segment();
				vec.push_back(std::make_pair(seg.get_Segment(), seg.getName()));
			}
		}

		return vec;
}

Arr_with_hist_2 Event::construct_disk_arr_for_event_detection(Robot* robot)
{
    Arr_with_hist_2 disk_arr = construct_arr_of_disk(robots, robot);

	double radius = robot->getDisk().get_radius().doubleValue();
    std::vector<Named_Circle_2>  vec =
            utility.point_vector_to_circle_vector(
                utility.get_arrangement_vertices_position_and_name(disk_arr), radius);

    disk_arr.clear();
    //utility.add_vector_of_curves_to_arrangement(disk_arr, &vec);
    CGAL::insert(disk_arr, vec.begin(), vec.end());

    add_disk_with_extra_radius_to_arr(disk_arr, robots, robot);

    return disk_arr;

}

void Event::add_disk_with_extra_radius_to_arr(Arr_with_hist_2 &arr, std::vector<Robot> *robots,
                                              Robot *robot)
{
    std::vector<Robot>::iterator robot_;
    for(robot_=robots->begin(); robot_ != robots->end(); ++robot_)
	{
		if (*robot_ != *robot)
		{
			Circle_2 disk(robot_->get_position(),
				std::pow(robot_->getDisk().get_radius().doubleValue() + 
					robot->getDisk().get_radius().doubleValue(), 2));

			Named_Circle_2 name_disk(disk, robot_->get_Name());

			CGAL::insert(arr, name_disk);
		}
    }
}

std::vector<std::pair<Point_2, std::string>> Event::segments_event_points(
	std::vector<std::pair<Segment_2, std::string>> &vec, Robot *robot,
	Segment_2 &path, Movement_Direction *dir)
{
	std::vector<std::pair<Point_2, std::string>> result;

	double radius = robot->getDisk().get_radius().doubleValue();
	std::pair<bool, Point_2> intersect;
	for (auto it = vec.begin(); it != vec.end(); ++it)
	{
		intersect = utility.segment_segment_intersection(path, it->first);
		if (intersect.first)
		{
			if (dir->get_direction_name() == "Left" || dir->get_direction_name() == "Right")
			{
				Point_2 p1(intersect.second.x() - radius, intersect.second.y());
				Point_2 p2(intersect.second.x() + radius, intersect.second.y());
				if(path.has_on(p1))
					result.push_back(std::make_pair(p1, it->second));
				else if(path.has_on(p2))
					result.push_back(std::make_pair(p2, it->second));
			}
			else if (dir->get_direction_name() == "Up" || dir->get_direction_name() == "Down")
			{
				Point_2 p1(intersect.second.x(), intersect.second.y() - radius);
				Point_2 p2(intersect.second.x(), intersect.second.y() + radius);
				if (path.has_on(p1))
					result.push_back(std::make_pair(p1, it->second));
				else if (path.has_on(p2))
					result.push_back(std::make_pair(p2, it->second));
			}
		}
	}

	return result;
}


//    Arr_with_hist_2::Vertex_iterator vit;
//    for (vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit)
//    {
//      std::cout<<vit->data().front();
//      degree = vit->degree();
//      if (degree == 0)
//        vit->set_data (BLUE);       // Isolated vertex.
//      else if (degree <= 2)
//        vit->set_data (RED);        // Vertex represents an endpoint.
//      else
//        vit->set_data (WHITE);      // Vertex represents an intersection point.
//    }







