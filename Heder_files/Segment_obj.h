#ifndef SEGMENT_OBJ_H
#define SEGMENT_OBJ_H

#include "CGAL_TypeDefinition.h"

struct Segment_Object
{

public:
	Segment_Object() {}

    Segment_Object(Segment_2 Segment_, std::string Name_)
    {
		name = Name_;
        segment = Segment_;
        named_segment = Named_Segment_2(Segment_, Name_);
    }

    Named_Segment_2 get_Named_Segment() const { return named_segment; }

    Segment_2 get_Segment() const { return segment; }

    Direction_2 get_direction() const { return segment.direction(); }

    Line_2 get_line() const { return Line_2(segment); }

	std::string getName() const { return name; }

private:

    std::string name;
    Segment_2 segment;
    Named_Segment_2 named_segment;

};

#endif // SEGMENT_OBJ_H

