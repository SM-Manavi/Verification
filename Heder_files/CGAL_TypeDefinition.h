#ifndef CGAL_TYPEDEFINITION_H
#define CGAL_TYPEDEFINITION_H

#include <ostream>
#include <string>
#include <vector>
#include <iostream>
#include <vector>
#include <list>


#include <CGAL/Cartesian.h>
#include <CGAL/Exact_rational.h>
#include <CGAL/Arr_circle_segment_traits_2.h>
#include <CGAL/Arrangement_with_history_2.h>
#include <CGAL/Arr_accessor.h>
#include <CGAL/Arr_extended_dcel.h>
#include <CGAL/Arr_consolidated_curve_data_traits_2.h>
#include <CGAL/CORE_BigRat.h>
#include <CGAL/CORE_algebraic_number_traits.h>
#include <CGAL/Arr_conic_traits_2.h>
#include <CGAL/Arr_polycurve_traits_2.h>
#include <CGAL/Polygon_2.h>

#include <CGAL/Exact_circular_kernel_2.h>
#include "CGAL/intersections.h"
#include <CGAL/Circular_kernel_intersections.h>




//#include <CGAL/Arr_algebraic_segment_traits_2.h>
//#include <CGAL/Quotient.h>
//#include <CGAL/MP_Float.h>
////
//typedef CGAL::Quotient<CGAL::MP_Float>                        Num_Type;
//typedef CGAL::Cartesian<Num_Type>                             Kernel;

typedef CORE::BigRat                                            BigRat;
typedef CORE::Expr                                              Expr;

typedef CGAL::Cartesian<BigRat>                                 Kernel;
typedef Kernel::Circle_2                                        Circle_2;
typedef Kernel::Segment_2                                       Segment_2;
typedef Kernel::Iso_rectangle_2                                 Rect_2;
typedef Kernel::Point_2                                         Point_2;
typedef Kernel::Line_2                                          Line_2;
typedef Kernel::Direction_2                                     Direction_2;
typedef Kernel::Ray_2                                           Ray_2;
typedef CGAL::Polygon_2<Kernel>                                 Polygon_2;

typedef CGAL::Arr_circle_segment_traits_2<Kernel>               Circle_Segment_traits_2;

typedef std::string                                             Curve_Name;
typedef CGAL::Arr_consolidated_curve_data_traits_2
                   <Circle_Segment_traits_2, Curve_Name>        Traits_2;

typedef Traits_2::CoordNT                                       CoordNT;
typedef Traits_2::Point_2                                       TPoint_2;
typedef Traits_2::Curve_2                                       Named_Circle_2;
typedef Traits_2::Curve_2                                       Named_Segment_2;
typedef Traits_2::Curve_2                                       Named_Line_2;

//typedef std::vector<std::string>                                Vertex_incident_curve;

//typedef CGAL::Arr_vertex_extended_dcel<Traits_2,
//                                  Vertex_incident_curve>        Dcel;

typedef CGAL::Arrangement_with_history_2<Traits_2>         Arr_with_hist_2;
typedef Arr_with_hist_2::Curve_handle                      Curve_handle;
typedef Arr_with_hist_2::Vertex_handle                     Vertex_handle;
typedef Arr_with_hist_2::Vertex							   Vertex;
typedef Arr_with_hist_2::Face_handle                       Face_handle;
typedef Arr_with_hist_2::Face_const_iterator               Face_const_iterator;
typedef Arr_with_hist_2::Ccb_halfedge_const_circulator     Ccb_halfedge_const_circulator;


typedef Kernel::Intersect_2                           Intersect_2;


typedef CGAL::Exact_circular_kernel_2             Circular_k;
typedef CGAL::Point_2<Circular_k>                 CK_Point_2;
typedef CGAL::Circle_2<Circular_k>                CK_Circle_2;
typedef CGAL::Line_arc_2<Circular_k>              Line_arc_2;
typedef CGAL::Segment_2<Circular_k>               CK_Segment_2;
typedef CGAL::Circular_arc_point_2<Circular_k>    Circular_arc_point_2;

inline std::vector<std::pair<Point_2, std::string>> circle_segment_intersect(
	std::pair<Circle_2, std::string> &cir, Segment_2 &seg)
{
	std::vector<std::pair<Point_2, std::string>> intersection_points;
    typedef typename CGAL::CK2_Intersection_traits<Circular_k, CK_Circle_2, Line_arc_2>::type
             Intersection_result;
        std::vector<Intersection_result> result;

		CK_Point_2 circle_center(cir.first.center().x().doubleValue(), cir.first.center().y().doubleValue());
        CK_Circle_2 cir1(circle_center, cir.first.squared_radius().doubleValue());
		CK_Segment_2 seg2(CK_Point_2(seg.source().x().doubleValue(), seg.source().y().doubleValue()) ,
						  CK_Point_2(seg.target().x().doubleValue(), seg.target().y().doubleValue()) );

        Line_arc_2 line_arc(seg2);

        CGAL::intersection(cir1, line_arc, std::back_inserter(result));
        typedef std::pair<Circular_arc_point_2, unsigned int> intersect;
        intersect  intersect_date;
		
		for (auto it = result.begin(); it != result.end(); ++it)
		{
			intersect_date = boost::get<intersect>(*it);
			Circular_arc_point_2 intersection_point = intersect_date.first;

			Point_2 p(CGAL::to_double(intersection_point.x()),
					  CGAL::to_double(intersection_point.y()));

			intersection_points.push_back(std::make_pair(p, cir.second));
		}
		
		return intersection_points;
}


inline Point_2 toPoint_2(const TPoint_2 &point)
{
    CoordNT x = point.x(), y = point.y();
	Expr x_root = x.gamma(), y_root = y.root();
    BigRat x_ = x.alpha() + x.beta() * (CORE::sqrt(x_root).BigRatValue());
    BigRat y_ = y.alpha() + y.beta() * (CORE::sqrt(y_root).BigRatValue());

	return Point_2(x_, y_);
}

inline std::string toStr(const Point_2 &point)
{
	return point.x().get_str() + "  " + point.y().get_str();
}

inline std::ostream& operator <<(std::ostream &os, std::vector<Point_2> &vec)
{
	for (auto item = vec.begin(); item != vec.end(); ++item)
		os << item->x() << "  " << item->y() << std::endl;

	return os;
}


#endif // CGAL_TYPEDEFINITION_H
