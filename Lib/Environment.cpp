#include "Environment.h"

Environment::Environment(Rect_2 boundary, double radius_) : env_boundary(boundary)
{
    if(radius_ < 0)
    {
        printf ("Radius shoud be greater than or equal 1.");
        exit (EXIT_FAILURE);
    }


    BigRat c = radius_ * 1.5;
	Point_2 p1, p2;
	p1 = Point_2(boundary[1].x(), boundary[1].y() - c);
	p2 = Point_2(boundary[2].x(), boundary[2].y() + c);
    env_segment_boundary.push_back(Segment_Object(
                                       Segment_2(p1, p2), "Enviroment_right_seg"));//right

	p1 = Point_2(boundary[2].x() + c, boundary[2].y());
	p2 = Point_2(boundary[3].x() - c, boundary[3].y());
    env_segment_boundary.push_back(Segment_Object(
                                       Segment_2(p1, p2), "Enviroment_top_seg"));;//up

	p1 = Point_2(boundary[3].x(), boundary[3].y() + c);
	p2 = Point_2(boundary[4].x(), boundary[4].y() - c);
    env_segment_boundary.push_back(Segment_Object(
                                       Segment_2(p1, p2), "Enviroment_left_seg"));//left

	p1 = Point_2(boundary[4].x() - c, boundary[4].y());
	p2 = Point_2(boundary[1].x() + c, boundary[1].y());
    env_segment_boundary.push_back(Segment_Object(
                                       Segment_2(p1, p2), "Enviroment_bottom_seg"));//down


    std::cout<<"The environment created."<<std::endl;

    radius = radius_;

    //creat_virtual_rect();
    arrangment = new Arrangement(&obstacles, &robots, &env_segment_boundary);
}

void Environment::add_obstacle(Rect_2 obstacle)
{
    Rect_Obstacle new_obstacle(obstacle,"Obs"+std::to_string(obstacles.size()+1),
                              env_boundary, radius*1.5);

    if(has_New_hole_valid_position(new_obstacle.getObstacle_Rect()))
    {
        obstacles.push_back(new_obstacle);
        std::cout<<"New hole created."<<std::endl;
    }else
        std::cout<<"New hole does not created."<<std::endl;
}

int Environment::add_robot(Point_2 &robot_position)
{
    Robot new_robot(robot_position, "R"+std::to_string(robots.size()+1), radius, 
                    env_boundary);
    if(has_robot_valid_position(new_robot))
    {
        robots.push_back(new_robot);
		std::cout << "New robot added." << std::endl;
		return robots.size()-1;
    }
    return NULL;
}

bool Environment::has_robot_valid_position(Robot &robot) const
{
    if(env_boundary.has_on_bounded_side(robot.get_position())){
        for(std::vector<Rect_Obstacle>::const_iterator obs = obstacles.begin(); obs != obstacles.end();
            ++obs){
            if(obs->getObstacle_Rect().has_on_bounded_side(robot.get_position())){
                std::cout << "error : New robot loacated inside a hole." <<std::endl;
                return false;
            }
        }
    } else {
        std::cout << "error : New robot loacated outside the environment." <<std::endl;
        return false;
    }

    return true;
}

bool Environment::has_New_hole_valid_position(Rect_2 &NewHole_boundary){

    // New hole must be completely inside the environment and shoud not be
    // completely out of another hole.
    if(is_R1_Rect_Completely_inside_R2_Rect(NewHole_boundary, env_boundary)){
        for(std::vector<Rect_Obstacle>::iterator obstacle = obstacles.begin() ;
                                            obstacle != obstacles.end(); ++obstacle){
            if(is_R1_Rect_Completely_inside_R2_Rect(NewHole_boundary,
                                                    obstacle->getObstacle_Rect())){
                std::cout<<"New hole is completely insid another."<<std::endl;
                return false;
            }else if(is_R1_Rect_Completely_inside_R2_Rect(obstacle->getObstacle_Rect(),
                                                          NewHole_boundary)){
                std::cout<<"A hole in the environment completely was located inside the "
                           "new hole."<<std::endl;
                return false;
            }

        }
    }else{
        std::cout<<"New hole is not completely inside the environment."<<std::endl;
        return false;
    }

    for(std::vector<Robot>::iterator robot_= robots.begin(); robot_ != robots.end(); ++robot_){

        if(NewHole_boundary.has_on_boundary(robot_->get_position()))
        {
            std::cout<<"error: A robot located inside the new hole."<<std::endl;
            return false;
        }
    }

    return true;
}

bool Environment::is_R1_Rect_Completely_inside_R2_Rect(Rect_2 &R1, Rect_2 &R2){

    for(int R1_vertex_index=1; R1_vertex_index<=4; R1_vertex_index++)
        if(R2.has_on_unbounded_side(R1[ R1_vertex_index ])){
            return false;
        }
    return true;
}

Point_2 Environment::get_valid_destination(Point_2 source, Point_2 destination){

    Segment_2 path(source, destination);

    CGAL::Comparison_result compare_result;
    Point_2 temp1,temp2;
    temp1 = get_first_segment_intersection_with_Rect(path, env_boundary, "env");
    Point_2 valid_destination = temp1;
	
    for(std::vector<Rect_Obstacle>::iterator obstacle = obstacles.begin();
                                        obstacle != obstacles.end(); ++obstacle){

        temp2 = get_first_segment_intersection_with_Rect(path, obstacle->getObstacle_Rect(), "obs");

//    compare_distance_to_point(p,q,r) returns CGAL::SMALLER, iff q is closer to p than r,
//    CGAL::LARGER, iff r is closer to p than q, and CGAL::EQUAL otherwise.
        compare_result = CGAL::compare_distance_to_point(source, temp1, temp2);
        if (compare_result==CGAL::SMALLER)
                valid_destination = temp1;
        else if(compare_result==CGAL::LARGER)
                valid_destination = temp2;

        temp1 = valid_destination;
    }
	
	//bool Crash_with_Robot = false;
	//temp1 = valid_destination;
	//if(source != destination)
	//	for(auto robot:robots)
	//		if(robot.get_position() != source)
	//			if (path.has_on(robot.get_position()))
	//			{
	//				temp2 = robot.get_position();
	//				compare_result = CGAL::compare_distance_to_point(source, temp1, temp2);
	//				if (compare_result == CGAL::LARGER || compare_result == CGAL::EQUAL)
	//				{
	//					valid_destination = temp2;
	//					temp1 = temp2;
	//					Crash_with_Robot = true;
	//				}
	//					
	//			}

	//if (Crash_with_Robot)
	//	if (valid_destination.x() == source.x())
	//		if (valid_destination.y() > source.y())
	//			valid_destination = Point_2(valid_destination.x(), valid_destination.y() - 1);
	//		else
	//			valid_destination = Point_2(valid_destination.x(), valid_destination.y() + 1);

	//	else if(valid_destination.y() == source.y())
	//		if (valid_destination.x() > source.x())
	//			valid_destination = Point_2(valid_destination.x() - 1, valid_destination.y());
	//		else
	//			valid_destination = Point_2(valid_destination.x() + 1, valid_destination.y());

    return valid_destination;

}

Point_2 Environment::get_first_segment_intersection_with_Rect(const Segment_2 seg,
                                                                    const Rect_2 rect, std::string obj_type){
		CGAL::cpp11::result_of<Intersect_2(Rect_2, Segment_2)>::type
			result = intersection(rect, seg);

		Point_2 temp;
		if (result)
			if (const Segment_2* s1 = boost::get<Segment_2>(&*result)) {
				BigRat x = s1->target().x() - s1->source().x();
				BigRat y = s1->target().y() - s1->source().y();
				x = (x / 2) + s1->source().x();
				y = (y / 2) + s1->source().y();

				temp = Point_2(x, y);
			}

		if (result)
			if (const Segment_2* s = boost::get<Segment_2>(&*result)) {
				if (s->source() == seg.source() && obj_type == "env")
					//When check segment intersection with enviroment rectangle.
					//(In this case, source of the segment loacated in enviroment.)
					return s->target();
				else if (s->source() == seg.source() && rect.has_on_boundary(temp))
					return seg.target();
				else
					return s->source();
			}
			else
				return *(boost::get<Point_2 >(&*result));
		else
			return seg.target();
}

Segment_Object Environment::get_segmet_path(double distance,
                                      Movement_Direction *direction, Point_2 robot_center)
{
    Segment_2 path = direction->get_segmet_path_on_direction(distance, robot_center);

    Point_2 destination = get_valid_destination(path.source(), path.target());

    return Segment_Object(Segment_2(path.source(),destination), "segment_path");

}


std::vector<Segment_Object> Environment::get_segments_boundary() const
{
    return env_segment_boundary;
}

Point_2 Environment::get_upper_right_corner() const
{
    return env_boundary.max();
}

Point_2 Environment::get_lower_left_corner() const
{
    return env_boundary.min();
}

void Environment::creat_virtual_rect()
{
    virtual_rect = Rect_2(Point_2(env_boundary.max().x()+radius,
                                    env_boundary.max().y()+radius) ,
                            Point_2(env_boundary.min().x()-radius,
                                    env_boundary.min().y()-radius) );

    virtual_rect_segment_boundary.push_back(Segment_Object(
                                       Segment_2(virtual_rect[1], virtual_rect[2]),
                                               "Imaginary_rec_right_seg"));//right

    virtual_rect_segment_boundary.push_back(Segment_Object(
                                       Segment_2(virtual_rect[2], virtual_rect[3]),
                                               "Imaginary_rec_up_seg"));;//up

    virtual_rect_segment_boundary.push_back(Segment_Object(
                                       Segment_2(virtual_rect[3], virtual_rect[4]),
                                               "Imaginary_rec_left_seg"));//left

    virtual_rect_segment_boundary.push_back(Segment_Object(
                                       Segment_2(virtual_rect[4], virtual_rect[1]),
                                               "Imaginary_rec_down_seg"));//down
}

std::vector<Segment_Object> Environment::getImaginary_segment_boundary() const
{
    return virtual_rect_segment_boundary;
}

Rect_2 Environment::getEnvironment_Rect() const
{
    return env_boundary;
}

std::vector<Rect_Obstacle> Environment::getObstacles() const
{
    return obstacles;
}

std::vector<Robot>* Environment::getRobots()
{
    return &robots;
}

Rect_2 Environment::get_virtual_rect() const
{
    return virtual_rect;
}

int Environment::move(Robot_mem *robot_mem, int alpha, double distance, Movement_Direction *direction)
{
	Robot *robot = robot_mem->getRobot();

    std::string move_inf = robot->get_Name() + ", Direction :" + direction->get_direction_name();

	if (distance == 0)
	{
		State_Data *state = create_state();
		state_space.add_state(state, move_inf);

		return 0;
	}


    Segment_Object segment_path = get_segmet_path(distance, direction, robot->get_position());

    Event event(&robots, &obstacles, &env_boundary);
    std::vector<Point_2> robot_events = event.calculate_move_event(segment_path, direction, *robot,
                                                                arrangment->creat_arr_without_(robot));

    for(std::vector<Point_2>::iterator ev = robot_events.begin(); ev != robot_events.end(); ++ev)
    {
        robot->GoTo(*ev, env_boundary);
        arr = arrangment->creat_arr_of_all_object();

        State_Data *state = create_state();
        state_space.add_state(state, move_inf);

		if (!check_move_condition(*robot_mem,alpha))
			break;

		//int new_neighbor = getNum_of_neighbor(robot);
		//if (new_neighbor < robot_mem->getCurrent_neighbor() && new_neighbor < alpha)
		//	break;
    }
	return robot_events.size();
}

bool Environment::check_move_condition(Robot_mem &mem, int &k)
{
	Connectivity_hash_map mat = get_connectivity_matrix();

	std::vector<std::string> robot_neighbor = mat[mem.getRobot()->get_Name()];
	bool move = 1;

	if (!robot_neighbor.empty())
		for (auto it = robot_neighbor.begin(); it != robot_neighbor.end(); ++it)
		{
			std::vector<std::string> neighbor_of_neighbor;
			neighbor_of_neighbor = mat[*it];
			if (scan_intersection(robot_neighbor, neighbor_of_neighbor).size() < k)
			{
				move = 0;
				break;
			}
		}
	else
		move = 0;

	return move;
}

std::vector<std::string> Environment::scan_intersection(std::vector<std::string> &v1,
	std::vector<std::string> &v2) {
	std::vector<std::string> v3;

	std::sort(v1.begin(), v1.end());
	std::sort(v2.begin(), v2.end());

	std::set_intersection(v1.begin(), v1.end(),
		v2.begin(), v2.end(),
		back_inserter(v3));
	return v3;
}

void Environment::jump(Status &new_situation)
{
    state_space.change_current_state(new_situation.getState());
    change_robots_position(new_situation);
    arr = arrangment->creat_arr_of_all_object();
}

void Environment::move_robot(Robot* robot, Point_2 &new_position)
{
        robot->GoTo(new_position, env_boundary);
}

void Environment::change_robots_position(Status &new_pos)
{
	for (auto it = new_pos.getRobot_status()->begin(); it != new_pos.getRobot_status()->end(); ++it)
        move_robot(it->getRobot(), it->getPosition());
}

bool Environment::check_collision(Robot* robot , Point_2 destination)
{
    Point_2 valid_destination = get_valid_destination(robot->get_position(), destination);

    //Check collision with the environment.
	if (destination != valid_destination)
		return true;
	else
	{
		//Check collision with other robots.
		Segment_2 seg(robot->get_position(), destination);
		for (auto it = robots.begin(); it != robots.end(); ++it)
			if (*it != *robot && seg.has_on(it->get_position()))
				return true;
	}

    return false;
}

std::pair<bool, Point_2> Environment::check_collision(Robot* robot , Point_2 destination, Movement_Direction *dir)
{
    std::vector<Point_2> collision_points;
    collision_points.push_back(get_valid_destination(robot->get_position(), destination));

    for(auto it = robots.begin() ; it != robots.end() ; ++it)
        if(robot->get_position() != it->get_position())
            collision_points.push_back(it->get_position());

    dir->sort_event(collision_points);

    if(collision_points.size() == 0)
        return std::make_pair(false, destination);
    else
        return std::make_pair(true, collision_points.front());

}

void Environment::initialization()
{
    arr = arrangment->creat_arr_of_all_object();
    state_space.first_state(create_state());
}

State_Data* Environment::create_state()
{
    Event event(&robots, &obstacles, &env_boundary);

    State_Data* new_state = new State_Data;
    new_state->faces_names = arrangment->get_sorted_faces_names(arr);
    new_state->disk_sequences = event.get_sequences_for_all_robot();
    new_state->connectivity_matrix = get_connectivity_matrix();
	
    return new_state;
}

Connectivity_hash_map Environment::get_connectivity_matrix()
{
    Connectivity_hash_map connectivity_table;
    for(auto robot = robots.begin(); robot != robots.end(); robot++)
    {
        connectivity_table[robot->get_Name()] = check_connections(*robot);
    }

    return connectivity_table;
}

std::vector<std::string> Environment::check_connections(Robot &robot_)
{
    std::vector<std::string> connection;
    for(auto robot = robots.begin(); robot != robots.end(); robot++)
    {
        if(robot_ == *robot)
            continue;

        if(robot_.communication_check(*robot))
            connection.push_back(robot->get_Name());
    }

    std::sort(connection.begin(), connection.end());

    return connection;
}

State_Data* Environment::current_state()
{
    return state_space.getCurrent_state();
}

int Environment::getNum_of_neighbor(Robot *robot)
{
    return (get_connectivity_matrix()[robot->get_Name()]).size();
}

int Environment::getNum_of_State()
{
	return state_space.getSize();
}

void Environment::createLTS()
{
    state_space.creatLTS();
}

double Environment::getRadius()
{
	return radius;
}
