#ifndef DIRECTION_OF_MOVE_H
#define DIRECTION_OF_MOVE_H

#include"Robot.h"

class Movement_Direction
{
public:
    Movement_Direction() {}

    virtual std::string get_direction_name()=0;

    virtual std::pair<Point_2, Point_2> get_diameter_on_direction(Robot &robot) = 0;

    virtual Point_2 adjust_pulled_segment_event_to_center_of_disk
                                       (Point_2 &current_point,Robot &robot)=0;

    virtual Point_2 adjust_pushed_segment_event_to_center_of_disk
                                       (Point_2 &current_point,Robot &robot)=0;

    virtual Polygon_2 get_pulled_segment_motion_area_as_polygon (Segment_2 &path, Robot &robot)=0;
    
    virtual Polygon_2 get_pushed_segment_motion_area_as_polygon (Segment_2 &path, Robot &robot)=0;

    virtual Segment_2 get_segmet_path_on_direction(double distance, Point_2 source)=0;




	virtual void sort_event(std::vector<Point_2> &events) = 0;
	virtual void sort_event(std::vector<std::pair<Point_2, std::string>> &events) = 0;
	virtual void sort_seq(std::vector<Vertex_handle> &seq) = 0;

    virtual Polygon_2 create_polygon_of_engulfing_segment_motion_area
                                   (Line_2 &source, Line_2 &destination, Robot &robot)=0;

    virtual Movement_Direction* turn_180() = 0;
    virtual std::vector<Movement_Direction*> turn_90() = 0;

    struct Descending_sort_by_position
    {
        bool operator()(Point_2 &first, Point_2 &second)
        {
            return second < first ;
        }

		bool operator()(std::pair<Point_2, std::string> &first, std::pair<Point_2, std::string> &second)
		{
			return second.first < first.first;
		}

        bool operator()(Vertex_handle &first, Vertex_handle &second)
        {

            return toPoint_2(second->point()) < toPoint_2(first->point()) ;
        }
    };

    struct Ascending_sort_by_position
    {
        bool operator()(Point_2 &first, Point_2 &second)
        {
            return first < second;
        }

		bool operator()(std::pair<Point_2, std::string> &first, std::pair<Point_2, std::string> &second)
		{
			return second.first > first.first;
		}

        bool operator()(Vertex_handle &first, Vertex_handle &second)
        {
            return toPoint_2(first->point()) < toPoint_2(second->point());
        }
    };

};



inline Movement_Direction* toDirecrion_type(std::string str);


class Left_and_Right_Direction : public Movement_Direction
{
public:
    Left_and_Right_Direction() {}

    virtual std::string get_direction_name()=0;

    std::pair<Point_2, Point_2> get_diameter_on_direction(Robot &robot)
    {
        std::pair<Point_2,Point_2> diameter_range;
        Point_2 robot_center = robot.get_position();
        BigRat radius = robot.getDisk().get_radius();

        return diameter_range = std::make_pair(Point_2(robot_center.x()-radius, robot_center.y()),
                                               Point_2(robot_center.x()+radius, robot_center.y()));
    }

    virtual Point_2 adjust_pulled_segment_event_to_center_of_disk
                                       (Point_2 &current_point,Robot &robot)=0;

    virtual Point_2 adjust_pushed_segment_event_to_center_of_disk
                                       (Point_2 &current_point,Robot &robot)=0;

    virtual Segment_2 get_segmet_path_on_direction(double distance, Point_2 source)=0;
    
    virtual Polygon_2 get_pulled_segment_motion_area_as_polygon (Segment_2 &path,Robot &robot)=0;
    
    virtual Polygon_2 get_pushed_segment_motion_area_as_polygon (Segment_2 &path,Robot &robot)=0;

	virtual void sort_event(std::vector<Point_2> &events) = 0;
	virtual void sort_event(std::vector<std::pair<Point_2, std::string>> &events) = 0;
	virtual void sort_seq(std::vector<Vertex_handle> &seq) = 0;

    virtual Movement_Direction* turn_180() = 0;

    std::vector<Movement_Direction*> turn_90()
    {
        std::vector<Movement_Direction*> turns = {toDirecrion_type("Up"), toDirecrion_type("Down")};
        return turns;
    }

    Polygon_2 create_polygon_of_engulfing_segment_motion_area
                           (Line_2 &source, Line_2 &destination, Robot &robot)
    {
        BigRat y1, y2;
        y1 = robot.get_left_segment().get_Segment().source().y();
        y2 = robot.get_left_segment().get_Segment().target().y();

        // Creating a four side polygon that one side is the source of engulfing segment
        // and another side is the destination of the engulfing segment and other sides
        // of polygon form connecting the vertices of two engufig segment are formed.
        Point_2 points[] = {
            Point_2(source.x_at_y(y1), y1),
            Point_2(source.x_at_y(y2), y2),
            Point_2(destination.x_at_y(y2), y2),
            Point_2(destination.x_at_y(y1), y1)
        };
        //Atomaticaly, the last point in the points array will connecte to the first point
        // of that.
        return Polygon_2(points, points+4);
    }

};






class Up_and_Down_Direction : public Movement_Direction
{
public:
    Up_and_Down_Direction() {}

    virtual std::string get_direction_name()=0;

    std::pair<Point_2, Point_2> get_diameter_on_direction(Robot &robot)
    {
        std::pair<Point_2,Point_2> diameter_range;
        Point_2 robot_center = robot.get_position();
        BigRat radius = robot.getDisk().get_radius();

        return diameter_range = std::make_pair(Point_2(robot_center.x(), robot_center.y()+radius),
                                               Point_2(robot_center.x(), robot_center.y()-radius));

    }

    virtual Point_2 adjust_pulled_segment_event_to_center_of_disk
                                       (Point_2 &current_point,Robot &robot)=0;

    virtual Point_2 adjust_pushed_segment_event_to_center_of_disk
                                       (Point_2 &current_point,Robot &robot)=0;

    virtual Segment_2 get_segmet_path_on_direction(double distance, Point_2 source)=0;

    
    virtual Polygon_2 get_pulled_segment_motion_area_as_polygon (Segment_2 &path,Robot &robot)=0;
    
    virtual Polygon_2 get_pushed_segment_motion_area_as_polygon (Segment_2 &path,Robot &robot)=0;

	virtual void sort_event(std::vector<Point_2> &events) = 0;
	virtual void sort_event(std::vector<std::pair<Point_2, std::string>> &events) = 0;
    virtual void sort_seq(std::vector<Vertex_handle> &seq) = 0;

    virtual Movement_Direction* turn_180() = 0;

    std::vector<Movement_Direction*> turn_90()
    {
        std::vector<Movement_Direction*> turns = {toDirecrion_type("Left"), toDirecrion_type("Right")};
        return turns;
    }

    Polygon_2 create_polygon_of_engulfing_segment_motion_area
                           (Line_2 &source, Line_2 &destination, Robot &robot)
    {
        BigRat x1, x2;
        x1 = robot.get_bottom_segment().get_Segment().source().x();
        x2 = robot.get_bottom_segment().get_Segment().target().x();


        // Creating a four side polygon that one side is the source of engulfing segment
        // and another side is the destination of the engulfing segment and other sides
        // of polygon form connecting the vertices of two engufig segment are formed.
        Point_2 points[] = {
            Point_2(x1, source.y_at_x(x1)),
            Point_2(x2, source.y_at_x(x2)),
            Point_2(x2, destination.y_at_x(x2)),
            Point_2(x1, destination.y_at_x(x1))
        };
        //Atomaticaly, the last point in the points array will connecte to the first point
        // of that.
        return Polygon_2(points, points+4);
    }
};


class Left_Direction : public Left_and_Right_Direction
{
public:
    Left_Direction() {}

    std::string get_direction_name(){return "Left";}

    Point_2 adjust_pulled_segment_event_to_center_of_disk
                                       (Point_2 &current_point,Robot &robot)
    {
        return Point_2(current_point.x()-robot.getDisk().get_radius()
                       ,robot.get_position().y());
    }

    Point_2 adjust_pushed_segment_event_to_center_of_disk
                                       (Point_2 &current_point,Robot &robot)
    {
        return Point_2(current_point.x()+robot.getDisk().get_radius()
                       ,robot.get_position().y());
    }

    Segment_2 get_segmet_path_on_direction(double distance, Point_2 source)
    {
        return Segment_2(source, Point_2(source.x()-distance, source.y()) );
    }
    

    Polygon_2 get_pulled_segment_motion_area_as_polygon (Segment_2 &path,Robot &robot)
    {
        Line_2 source = robot.get_right_segment().get_line();
        
        Direction_2 dir = robot.get_right_segment().get_direction();

        Point_2 dest = Point_2(path.target().x()+robot.getDisk().get_radius(),
                               path.target().y());

        Line_2 destination = Line_2(dest,dir);
        
        return create_polygon_of_engulfing_segment_motion_area(source, destination, robot);
    }
    
    Polygon_2 get_pushed_segment_motion_area_as_polygon (Segment_2 &path, Robot &robot)
    {
        Line_2 source = robot.get_left_segment().get_line();
        
        Direction_2 dir = robot.get_left_segment().get_direction();

        Point_2 dest = Point_2(path.target().x()-robot.getDisk().get_radius(),
                               path.target().y());

        Line_2 destination = Line_2(dest,dir);

        return create_polygon_of_engulfing_segment_motion_area(source, destination, robot);
    }

    void sort_event(std::vector<Point_2> &events)
    {
        std::sort(events.begin(), events.end(), Descending_sort_by_position());
    }

	void sort_event(std::vector<std::pair<Point_2, std::string>> &events)
	{
		std::sort(events.begin(), events.end(), Descending_sort_by_position());
	}

    void sort_seq(std::vector<Vertex_handle> &seq)
    {
        std::sort(seq.begin(), seq.end(),Descending_sort_by_position());
    }

    Movement_Direction* turn_180()
    {
        return toDirecrion_type("Right");
    }

};





class Right_Direction : public Left_and_Right_Direction
{
public:
    Right_Direction() {}

    std::string get_direction_name(){return "Right";}

    Point_2 adjust_pulled_segment_event_to_center_of_disk
                                       (Point_2 &current_point,Robot &robot)
    {
        return Point_2(current_point.x()+robot.getDisk().get_radius()
                       ,robot.get_position().y());
    }

    Point_2 adjust_pushed_segment_event_to_center_of_disk
                                       (Point_2 &current_point,Robot &robot)
    {
        return Point_2(current_point.x()-robot.getDisk().get_radius()
                       ,robot.get_position().y());
    }

    Segment_2 get_segmet_path_on_direction(double distance, Point_2 source)
    {
        return Segment_2(source, Point_2(source.x()+distance, source.y()) );
    }

    
    Polygon_2 get_pulled_segment_motion_area_as_polygon (Segment_2 &path,Robot &robot)
    {
        Line_2 source = robot.get_left_segment().get_line();
        
        Direction_2 dir = robot.get_left_segment().get_direction();
        Point_2 dest = Point_2(path.target().x()-robot.getDisk().get_radius(),
                               path.target().y());
        Line_2 destination = Line_2(dest,dir);
        
        return create_polygon_of_engulfing_segment_motion_area(source, destination, robot);
    }
    
    Polygon_2 get_pushed_segment_motion_area_as_polygon (Segment_2 &path,Robot &robot)
    {
        Line_2 source = robot.get_right_segment().get_line();
        
        Direction_2 dir = robot.get_right_segment().get_direction();
        Point_2 dest = Point_2(path.target().x()+robot.getDisk().get_radius(),
                               path.target().y());
        Line_2 destination = Line_2(dest,dir);
        
        return create_polygon_of_engulfing_segment_motion_area(source, destination, robot);
    }

    void sort_event(std::vector<Point_2> &events)
    {
        std::sort(events.begin(), events.end(),Ascending_sort_by_position());
    }

	void sort_event(std::vector<std::pair<Point_2, std::string>> &events)
	{
		std::sort(events.begin(), events.end(), Ascending_sort_by_position());
	}

    void sort_seq(std::vector<Vertex_handle> &seq)
    {
        std::sort(seq.begin(), seq.end(),Ascending_sort_by_position());
    }

    Movement_Direction* turn_180()
    {
        return toDirecrion_type("Left");
    }
};






class Up_Direction : public Up_and_Down_Direction
{
public:
    Up_Direction() {}

    std::string get_direction_name(){return "Up";}

    Point_2 adjust_pulled_segment_event_to_center_of_disk
                                       (Point_2 &current_point,Robot &robot)
    {
        return Point_2(robot.get_position().x(),
                      current_point.y()+robot.getDisk().get_radius());
    }

    Point_2 adjust_pushed_segment_event_to_center_of_disk
                                       (Point_2 &current_point,Robot &robot)
    {
        return Point_2(robot.get_position().x(),
                      current_point.y()-robot.getDisk().get_radius());
    }

    Segment_2 get_segmet_path_on_direction(double distance, Point_2 source)
    {
       return Segment_2(source, Point_2(source.x(), source.y()+distance) );
    }
    
    Polygon_2 get_pulled_segment_motion_area_as_polygon (Segment_2 &path,Robot &robot)
    {
        Line_2 source = robot.get_bottom_segment().get_line();
        
        Direction_2 dir = robot.get_bottom_segment().get_direction();
        Point_2 dest = Point_2(path.target().x(),
                               path.target().y()-robot.getDisk().get_radius());
        Line_2 destination = Line_2(dest,dir);
        
        return create_polygon_of_engulfing_segment_motion_area(source, destination, robot);
    }
    
    Polygon_2 get_pushed_segment_motion_area_as_polygon (Segment_2 &path,Robot &robot)
    {
        Line_2 source = robot.get_top_segment().get_line();
        
        Direction_2 dir = robot.get_top_segment().get_direction();
        Point_2 dest = Point_2(path.target().x(),
                               path.target().y()+robot.getDisk().get_radius());
        Line_2 destination = Line_2(dest,dir);
        
        return create_polygon_of_engulfing_segment_motion_area(source, destination, robot);
    }
    
    void sort_event(std::vector<Point_2> &events)
    {
        std::sort(events.begin(), events.end(),Ascending_sort_by_position());
    }

	void sort_event(std::vector<std::pair<Point_2, std::string>> &events)
	{
		std::sort(events.begin(), events.end(), Ascending_sort_by_position());
	}

    void sort_seq(std::vector<Vertex_handle> &seq)
    {
        std::sort(seq.begin(), seq.end(),Ascending_sort_by_position());
    }

    Movement_Direction* turn_180()
    {
        return toDirecrion_type("Down");
    }

};






class Down_Direction : public Up_and_Down_Direction
{
public:
    Down_Direction() {}

    std::string get_direction_name(){return "Down";}

    Point_2 adjust_pulled_segment_event_to_center_of_disk
                                       (Point_2 &current_point,Robot &robot)
    {
        return Point_2(robot.get_position().x(),
                      current_point.y()-robot.getDisk().get_radius());
    }

    Point_2 adjust_pushed_segment_event_to_center_of_disk
                                       (Point_2 &current_point,Robot &robot)
    {
        return Point_2(robot.get_position().x(),
                      current_point.y()+robot.getDisk().get_radius());
    }

    Segment_2 get_segmet_path_on_direction(double distance, Point_2 source)
    {
       return Segment_2(source, Point_2(source.x(), source.y()-distance) );
    }

    Polygon_2 get_pulled_segment_motion_area_as_polygon (Segment_2 &path,Robot &robot)
    {
        Line_2 source = robot.get_top_segment().get_line();

        Direction_2 dir = robot.get_top_segment().get_direction();
        Point_2 dest = Point_2(path.target().x(),
                               path.target().y()+robot.getDisk().get_radius());
        Line_2 destination = Line_2(dest,dir);

        return create_polygon_of_engulfing_segment_motion_area(source, destination, robot);
    }

    Polygon_2 get_pushed_segment_motion_area_as_polygon (Segment_2 &path,Robot &robot)
    {
        Line_2 source = robot.get_bottom_segment().get_line();

        Direction_2 dir = robot.get_bottom_segment().get_direction();
        Point_2 dest = Point_2(path.target().x(),
                               path.target().y()-robot.getDisk().get_radius());
        Line_2 destination = Line_2(dest,dir);

        return create_polygon_of_engulfing_segment_motion_area(source, destination, robot);
    }

    void sort_event(std::vector<Point_2> &events)
    {
        std::sort(events.begin(), events.end(),Descending_sort_by_position());
    }

	void sort_event(std::vector<std::pair<Point_2, std::string>> &events)
	{
		std::sort(events.begin(), events.end(), Descending_sort_by_position());
	}

    void sort_seq(std::vector<Vertex_handle> &seq)
    {
        std::sort(seq.begin(), seq.end(),Descending_sort_by_position());
    }

    Movement_Direction* turn_180()
    {
        return toDirecrion_type("Up");
    }
};


inline Movement_Direction* toDirecrion_type(std::string str)
{
    if(str == "Right")
        return new Right_Direction();
    else if(str == "Left")
        return new Left_Direction();
    else if(str == "Up")
        return new Up_Direction();
    else if(str == "Down")
        return new Down_Direction();
}

#endif // DIRECTION_OF_MOVE_H
