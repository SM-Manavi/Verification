#include "Arrangement.h"

void Arrangement::add_obstacles_to_arrangement()
{
    for(auto obstacle = obstacles->begin(); obstacle != obstacles->end(); ++obstacle){
        add_obstacle_boundaries_to_arrangement(*obstacle);
        add_obstacle_engulfing_segments_to_arrangement(*obstacle);
    }

}

void Arrangement::add_obstacle_boundaries_to_arrangement(Rect_Obstacle &obstacle)
{
    add_vector_of_segment_to_arrangement(obstacle.getObstacles_boundaries());
}

void Arrangement::add_obstacle_engulfing_segments_to_arrangement(Rect_Obstacle &obstacle)
{
    add_vector_of_segment_to_arrangement(obstacle.getEngulfing_Segments());
}

void Arrangement::add_all_robots_to_arrangement()
{
    std::vector<Robot>::iterator robot;
    for(robot = robots->begin(); robot != robots->end(); ++robot)
    {
        add_robot_to_arrangement(*robot);
    }

}

void Arrangement::add_all_robots_to_arrangement_exept(Robot* ex_robot)
{
    std::vector<Robot>::iterator robot;
    for(robot = robots->begin(); robot != robots->end(); ++robot)
    {
        if(*robot != *ex_robot)
            add_robot_to_arrangement(*robot);
    }

}

Arr_with_hist_2 Arrangement::creat_arr_without_(Robot* robot)
{
    arr.clear();
    add_all_robots_to_arrangement_exept(robot);
    add_obstacles_to_arrangement();
    add_environment_boundary_to_arr();

    return arr;
}


Arr_with_hist_2 Arrangement::creat_arr_of_all_object()
{
    arr.clear();
    add_all_robots_to_arrangement();
    add_obstacles_to_arrangement();
    add_environment_boundary_to_arr();

    return arr;
}

void Arrangement::add_robot_engulfing_segments_to_arrangement(Robot &robot)
{
    add_vector_of_segment_to_arrangement(robot.getEngulfing_segments());
}

void Arrangement::add_robot_to_arrangement(Robot &robot)
{
    CGAL::insert(arr, robot.getDisk().get_Named_Disk());
    add_robot_engulfing_segments_to_arrangement(robot);
}

void Arrangement::add_environment_boundary_to_arr()
{
    add_vector_of_segment_to_arrangement(env_boundary);
}

void Arrangement::add_vector_of_segment_to_arrangement(std::vector<Segment_Object> *segments)
{
    std::vector<Segment_Object>::iterator segment;
    for(segment = segments->begin(); segment != segments->end(); ++segment)
        CGAL::insert(arr, segment->get_Named_Segment());

}

//void Arrangement::insert_environment_in_arrangement()
//{
//    vector<Segment_Object> env_boundary = environment.get_segments_boundary();
//    for(vector<Segment_Object>::iterator seg = env_boundary.begin();
//                                         seg != env_boundary.end();  ++seg){

//        seg->setSegment_Handle(insert(arr, seg->get_Named_Segment()));

//    }

//    vector<Segment_Object> virtual_env_boundary = environment.getImaginary_segment_boundary();
//    for(vector<Segment_Object>::iterator seg = virtual_env_boundary.begin();
//                                         seg != virtual_env_boundary.end();  ++seg){

//        seg->setSegment_Handle(insert(arr, seg->get_Named_Segment()));

//    }
//}

std::vector<std::string>* Arrangement::get_sorted_faces_names(Arr_with_hist_2 &arr)
{
    std::vector<std::string> *faces_names = get_arrangement_faces_names(arr);
    std::sort(faces_names->begin(), faces_names->end());

    return faces_names;
}


std::vector<std::string>* Arrangement::get_arrangement_faces_names(Arr_with_hist_2 &arr){
    std::vector<std::string> *faces_name = new std::vector<std::string>;
    Face_const_iterator face_ite;
    for(face_ite = arr.faces_begin(); face_ite != arr.faces_end(); ++face_ite)
    {
        if(face_ite->is_unbounded()) {continue;}

        std::vector<std::string> face_engulfing_curves_name = Ccb_curves_name(face_ite->outer_ccb());

        //maybe a curve split by one or more vertex. Therefore duplicates names
        //should be remove.
		Utility uti;
        uti.sort_and_remove_duplicates(face_engulfing_curves_name);
        faces_name->push_back(
                    uti.concatenate_elements_of_vector(face_engulfing_curves_name));
    }
    return faces_name;
}

std::vector<std::string> Arrangement::Ccb_curves_name(Ccb_halfedge_const_circulator circ)
{
    std::vector<std::string> face_curve_name;
    Arr_with_hist_2::Ccb_halfedge_const_circulator curve = circ ;
    do {
        CGAL::_Unique_list<Traits_2::Curve_2*>::const_iterator dit ;
     for (dit = curve->curve().data().begin(); dit != curve->curve().data().end();
          ++dit)
     {
         face_curve_name.push_back((**dit).data());
     }

    } while (++curve != circ);

    return face_curve_name;
}


