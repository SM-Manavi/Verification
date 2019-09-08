#include "M_Utility.h"

Utility::Utility()
{

}

void Utility::sort_and_remove_duplicates(std::vector<std::string>& vec)
{
  std::sort(vec.begin(), vec.end());
  vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
}

std::string Utility::concatenate_elements_of_vector(std::vector<std::string> &vec) const
{
    std::string concatenate = "";
    for(std::vector<std::string>::iterator element = vec.begin(); element != vec.end(); ++element)
    {
        if(*element != "")
            concatenate += *element + ",";
    }

    if(!concatenate.empty())
        concatenate.pop_back();

    return concatenate;
}

std::vector<std::string> Utility::get_vertices_name(std::vector<Vertex_handle> &vec)
{
    std::vector<std::string> names;
    std::vector<Vertex_handle>::iterator vertex;
    for(vertex = vec.begin(); vertex != vec.end(); ++vertex)
        names.push_back(accumulate_incident_edges_name_of_vertex(*vertex, false));

    return names;
}

std::string Utility::accumulate_incident_edges_name_of_vertex(Vertex_handle vertex,
                                                        bool tag_unimportant_vertex=true )
{
    std::vector<std::string> incident_edge_name;
    Arr_with_hist_2::Halfedge_around_vertex_circulator curve, circ;
    curve = circ = vertex->incident_halfedges();

	std::vector<std::string> prev_temp, next_temp;
    do {
            std::list<Traits_2::Curve_2*>::const_iterator dit ;
         for (dit = curve->curve().data().begin(); dit != curve->curve().data().end();
              ++dit)
         {
             next_temp.push_back( (**dit).data() );
         }

         if(tag_unimportant_vertex)
         {
             if(is_important_vertex(prev_temp, next_temp))
             {
                 prev_temp = next_temp;
                 incident_edge_name.insert(incident_edge_name.end(), next_temp.begin(),
                                           next_temp.end());
                 next_temp.clear();

             }else
             {
                 return "not_important";
             }
         }

    } while (++curve != circ);

    if(!tag_unimportant_vertex)
        incident_edge_name.insert(incident_edge_name.end(),
                                  next_temp.begin(), next_temp.end());

    Utility::sort_and_remove_duplicates(incident_edge_name);

    return Utility::concatenate_elements_of_vector(incident_edge_name);
}

bool Utility::is_important_vertex(std::vector<std::string> &prev_incident_edge_name,
                         std::vector<std::string> &next_incident_edge_name)
{
    if(prev_incident_edge_name.empty())
    {
        return true;
    }else
    {
        std::sort(prev_incident_edge_name.begin(), prev_incident_edge_name.end());
        std::sort(next_incident_edge_name.begin(), next_incident_edge_name.end());

        if(prev_incident_edge_name == next_incident_edge_name)
            return false;
        else
            return true;
    }
}

std::vector<std::pair<TPoint_2,std::string>> Utility::get_arrangement_vertices_position_and_name(
                                                                          Arr_with_hist_2 &arr)
{
    std::pair<TPoint_2,std::string> vertex_position_and_name;
    std::vector<std::pair<TPoint_2,std::string>> vec;
    Arr_with_hist_2::Vertex_iterator vit;
    for(vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit)
    {
        vertex_position_and_name = make_pair(vit->point(),
                                                 accumulate_incident_edges_name_of_vertex(vit));
        vec.push_back(vertex_position_and_name);
    }

    return vec;
}

void Utility::remove_vector_of_curves_from_arr(Arr_with_hist_2 &arr, std::vector<Curve_handle> &ch )
{
    std::vector<Curve_handle>::iterator curve_ite;
    for(curve_ite = ch.begin(); curve_ite != ch.end(); ++curve_ite)
    {
        Curve_handle curve = *curve_ite;
        CGAL::remove_curve(arr, curve);
    }
}

std::vector<Named_Circle_2> Utility::point_vector_to_circle_vector(std::vector<std::pair<TPoint_2,std::string>> &vec,
                                                     double radius)
{
    std::vector<Named_Circle_2> circle_vec;

    std::vector<std::pair<TPoint_2,std::string>>::iterator ite;
    for(ite = vec.begin(); ite != vec.end(); ++ite)
    {
        if(ite->second != "not_important")
        {
            Point_2 po = toPoint_2(ite->first);
            Circle_2 circle(po, pow(radius,2));
            circle_vec.push_back(Named_Circle_2(circle, ite->second));
        }
    }

    return circle_vec;
}

std::vector<std::pair<Circle_2, std::string>> Utility::point_vector_to_circle_vector_(std::vector<std::pair<TPoint_2, std::string>> &vec,
	double radius)
{
	std::vector<std::pair<Circle_2, std::string>>  circle_vec;

	std::vector<std::pair<TPoint_2, std::string>>::iterator it;
	for (it = vec.begin(); it != vec.end(); ++it)
	{
		if (it->second != "not_important")
		{
			Point_2 po = toPoint_2(it->first);
			Circle_2 circle(po, pow(radius, 2));
			circle_vec.push_back(std::make_pair(circle, it->second));
		}
	}

	return circle_vec;
}

std::vector<std::pair<Point_2, std::string>> Utility::disk_segment_intersection(
								std::vector<std::pair<Circle_2, std::string>> &vec, Segment_2 &seg)
{
	std::vector<std::pair<Point_2, std::string>> events;
	for (auto it = vec.begin(); it != vec.end(); ++it)
	{
		std::vector<std::pair<Point_2, std::string>> points;
		points = circle_segment_intersect(*it, seg);

		events.insert(events.end(), points.begin(), points.end());
	}

	return events;
}


void Utility::add_vector_of_point_as_circle_to_arr(Arr_with_hist_2 &arr,
                                                 std::vector<Named_Circle_2> &vec)
{
        CGAL::insert(arr, vec.begin(), vec.end());
}

std::vector<std::string> Utility::arr_intersection_with_segment(Arr_with_hist_2 &arr,
                                                       Named_Segment_2 &segment,
                                             Movement_Direction* sort_direction = new Right_Direction())
// arr_intersection_with_segment
{
    Curve_handle curve_handle;
    curve_handle = CGAL::insert(arr,segment);

    Arr_with_hist_2::Induced_edge_iterator induce_edge;
    Arr_with_hist_2::Halfedge_handle halfedge;
    std::vector<Vertex_handle> path_intersections;

    for(induce_edge = arr.induced_edges_begin(curve_handle);
        induce_edge != arr.induced_edges_end(curve_handle); ++induce_edge)
    {
        halfedge = *induce_edge;
        path_intersections.push_back(halfedge->source());
        path_intersections.push_back(halfedge->target());

    }

    remove_duplicate_by_position(path_intersections);

    sort_direction->sort_seq(path_intersections);

    std::vector<std::string> vertices_name = get_vertices_name(path_intersections);

    CGAL::remove_curve(arr, curve_handle);

    delete sort_direction;

    return vertices_name;

}

void Utility::remove_duplicate_by_position(std::vector<Vertex_handle> &vec)
{
    std::vector<Vertex_handle> temp;
    std::vector<Vertex_handle>::iterator t1,t2;
    for(t1 = vec.begin(); t1 != vec.end(); ++t1)
    {

        bool dup = false;
        for(t2 = temp.begin(); t2 != temp.end(); ++t2)
        {
            if((*t1)->point() == (*t2)->point())
            {
                dup = true;
                break;
            }
        }

        if(!dup)
            temp.push_back(*t1);
    }
    vec = temp;
}



void Utility::add_vector_of_curves_to_arrangement(Arr_with_hist_2 &arr, std::vector<Named_Circle_2> *curves)
{
    std::vector<Named_Circle_2>::iterator curve;
    for(curve = curves->begin(); curve != curves->end(); ++curve)
    {
        Named_Circle_2 circle = *curve;
        CGAL::insert(arr, circle);
    }
}

std::vector<Point_2> Utility::arr_segment_intersection_point(Arr_with_hist_2 &arr, Named_Segment_2 &segment_path)
{
    Curve_handle curve_handle;
    curve_handle = CGAL::insert(arr,segment_path);

    Arr_with_hist_2::Induced_edge_iterator induce_edge;
    Arr_with_hist_2::Halfedge_handle halfedge;
    std::vector<Point_2> path_intersections;
    for(induce_edge = arr.induced_edges_begin(curve_handle);
        induce_edge != arr.induced_edges_end(curve_handle); ++induce_edge)
    {
        halfedge = *induce_edge;

        Point_2 point = toPoint_2(halfedge->source()->point());
        path_intersections.push_back(point);
        point = toPoint_2(halfedge->target()->point());
        path_intersections.push_back(point);
    }

    CGAL::remove_curve(arr, curve_handle);

    return path_intersections;
}


void Utility::Arr_with_hist_and_consolidated_curve_data_itrerator(
        Arr_with_hist_2 &arr_) const{
    // nokte in function in ast ke choon khod Arr_with_hist bray modiriyat
    // curve hay khod niaz be ezafe kardan data darad zamani ke ma ham
    // dar an dade migozarim dast resi be dade ghozashte shode tavasot
    // ma kheyli sakht mishavad ke code zir in kar ra bray halati ke
    // roye edge ha harkat mikonim va mikhahim az etela-at curve an
    // ba khabar shavim ra neshan midahad.
    Arr_with_hist_2::Edge_iterator eit;
    for (eit = arr_.edges_begin(); eit != arr_.edges_end(); ++eit)
    {
//         Traits_2::Curve_2 *a;
//         a = eit->curve().data().back();
//         std::cout<< a->data();

         CGAL::_Unique_list<Traits_2::Curve_2*>::const_iterator dit ;
      for (dit = eit->curve().data().begin(); dit != eit->curve().data().end();
           ++dit)
      {
          std::cout<< (**dit).data()<< std::endl;
      }
    }
}

std::vector<Curve_handle> Utility::add_corner_of_rect_as_circle_to_arr(
                                         std::vector<Rect_Obstacle> &rects,
                                         Arr_with_hist_2 &arr, double radius)
{
    std::vector<Curve_handle> handle;
    std::vector<Rect_Obstacle>::iterator rect;
    for(rect = rects.begin(); rect != rects.end(); ++rect)
    {
        std::vector<Curve_handle> temp;
        temp = add_corner_of_rect_as_circle_to_arr(*rect, arr, radius);
        handle.insert(handle.end(), temp.begin(), temp.end());
    }

    return handle;
}

std::vector<Curve_handle> Utility::add_corner_of_rect_as_circle_to_arr(Rect_Obstacle &rect,
                                                        Arr_with_hist_2 &arr, double radius)
{
    std::vector<Curve_handle> handle;
    for (int corner=0; corner<=3; corner++)
    {
        Circle_2 circle(rect.getObstacle_Rect()[corner], pow(radius,2));
        handle.push_back(CGAL::insert(arr,Named_Circle_2(circle, rect.get_Name())) );
    }

    return handle;
}

Segment_2 Utility::segment_intersection_of_line_with_rect(Line_2 line, Rect_2 rect) const
{
    CGAL::cpp11::result_of<Intersect_2(Rect_2, Line_2)>::type
            result = intersection(rect, line);

    if (result)
        if (const Segment_2* s = boost::get<Segment_2>(&*result))
            return *s;

    printf ("Input line has not intesect with the rectangle.");
    exit (EXIT_FAILURE);
}

std::pair<bool, Point_2> Utility::segment_segment_intersection(Segment_2 &seg1, Segment_2 &seg2) const
{
	CGAL::cpp11::result_of<Intersect_2(Segment_2, Segment_2)>::type
		result = intersection(seg1, seg2);
	if (result) {
		if (const Point_2* s = boost::get<Point_2>(&*result)) {
			return std::make_pair(true, *s);
		}
	}

	return std::make_pair(false, Point_2(0,0));
}

Segment_2 Utility::segment_intersection_of_ray_with_rect(Ray_2 ray, Rect_2 rect) const
{
    CGAL::cpp11::result_of<Intersect_2(Rect_2, Ray_2)>::type
            result = intersection(rect, ray);

    if (result)
        if (const Segment_2* s = boost::get<Segment_2>(&*result))
            return *s;
		else {
			const Point_2* p = boost::get<Point_2 >(&*result);
			return Segment_2(*p, *p);
		}

    printf ("Input line has not intesect with the rectangle.");
    exit (EXIT_FAILURE);
}

bool Utility::connectivity_check(Connectivity_hash_map connection)
{
    std::vector<std::string> conn;
    c_check(connection, conn, "R1");

    if(conn.size() == connection.size())
        return true;
    else
        return false;
}

void Utility::c_check(Connectivity_hash_map &con, std::vector<std::string> &str, std::string robot_name)
{
    str.push_back(robot_name);

    std::vector<std::string> connection = con[robot_name];

    for(auto it = connection.begin(); it != connection.end(); ++it)
        if(std::find(str.begin(), str.end(), (*it)) == str.end())
            c_check(con, str, *it);

}
