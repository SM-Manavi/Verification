#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "Segment_obj.h"

struct Rect_Obstacle
{
    //environment needed for calculate surrounding segments position.

public:
//    Rect_Obstacle(std::string name_){}
    Rect_Obstacle(Rect_2 obstacle, std::string name_, Rect_2 &environment_, BigRat margin_size){
            name = name_;
            obstacle_rect = obstacle;
            obstacles_boundaries.push_back(Segment_Object(
                                               Segment_2(obstacle[1], obstacle[2]),
                                                name+"_right_side"));

            obstacles_boundaries.push_back(Segment_Object(
                                               Segment_2(obstacle[2], obstacle[3]),
                                                name+"_up_side"));

            obstacles_boundaries.push_back(Segment_Object(
                                               Segment_2(obstacle[3], obstacle[4]),
                                                name+"_left_side"));

            obstacles_boundaries.push_back(Segment_Object(
                                               Segment_2(obstacle[4], obstacle[1]),
                                                name+"_down_side"));
            create_engulfing_segment(environment_, margin_size);
    }

    void create_engulfing_segment(Rect_2 &env, BigRat &margin_size)
    {
		Point_2 p1, p2;

		p1 = Point_2(obstacle_rect[1].x(), env[1].y() - margin_size);
		p2 = Point_2(obstacle_rect[2].x(), env[2].y() + margin_size);

		obstacle_engulfing_segments.push_back(Segment_Object(Segment_2(p1, p2),
																			name + "_right_segment"));

		p1 = Point_2(env[2].x() + margin_size, obstacle_rect[2].y());
		p2 = Point_2(env[3].x() - margin_size, obstacle_rect[3].y());

        obstacle_engulfing_segments.push_back(Segment_Object(Segment_2(p1, p2),
																			name + "_top_segment"));

		p1 = Point_2(obstacle_rect[3].x(), env[3].y() + margin_size);
		p2 = Point_2(obstacle_rect[4].x(), env[4].y() - margin_size);

		obstacle_engulfing_segments.push_back(Segment_Object(Segment_2(p1, p2),
																			name + "_left_segment"));

		p1 = Point_2(env[4].x() - margin_size, obstacle_rect[4].y());
		p2 = Point_2(env[1].x() + margin_size, obstacle_rect[1].y());

		obstacle_engulfing_segments.push_back(Segment_Object(Segment_2(p1, p2),
																			name + "_bottom_segment"));

    }

    Rect_2 getObstacle_Rect() const { return obstacle_rect; }
    std::string get_Name() const { return name; }


    std::vector<Segment_Object>* getObstacles_boundaries()
    {
		return &obstacles_boundaries;
    }

    std::vector<Segment_Object>* getEngulfing_Segments()
    {
        return &obstacle_engulfing_segments;
    }



	Segment_Object get_right_engulfing_segment()
	{
		return obstacle_engulfing_segments[0];
	}

	Segment_Object get_left_engulfing_segment()
	{
		return obstacle_engulfing_segments[2];
	}

	Segment_Object get_top_engulfing_segment()
	{
		return obstacle_engulfing_segments[1];
	}

	Segment_Object get_bottom_engulfing_segment()
	{
		return obstacle_engulfing_segments[3];
	}



	Segment_Object get_right_side_segment()
	{
		return obstacles_boundaries[0];
	}

	Segment_Object get_left_side_segment()
	{
		return obstacles_boundaries[2];
	}

	Segment_Object get_top_side_segment()
	{
		return obstacles_boundaries[1];
	}

	Segment_Object get_bottom_side_segment()
	{
		return obstacles_boundaries[3];
	}


private:

    std::string name;
    Rect_2 obstacle_rect;
    std::vector<Segment_Object> obstacles_boundaries;

    std::vector<Segment_Object> obstacle_engulfing_segments;

//    Segment_2 surrounded_left_segment, surrounded_right_segment,
//              surrounded_up_segment, surrounded_down_segment;

};



#endif // OBSTACLE_H

