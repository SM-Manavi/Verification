#ifndef CIRCLE_OBJ_H
#define CIRCLE_OBJ_H

#include "CGAL_TypeDefinition.h"

class Disk_Object
{

public:
    Disk_Object(Point_2 &position, double &radius_, std::string &Name_)
    {
        name = Name_;
        disk = Circle_2(position, pow(radius_,2));
        named_disk = Named_Circle_2(disk, Name_);
        radius = radius_;
    }

    void reset(Point_2 &position, double &radius_, std::string &Name_)
    {
        name = Name_;
        disk = Circle_2(position, pow(radius_,2));
        named_disk = Named_Circle_2(disk, Name_);
        radius = radius_;
    }

	Named_Circle_2 get_Named_Disk() const { return named_disk; }

    Circle_2 get_Disk() const { return disk; }

    BigRat get_radius() const { return BigRat(radius); }
	//double get_radius() const { return radius; }

private:

    std::string name;
    Circle_2 disk;
    double radius;
    Named_Circle_2 named_disk;

};

#endif // CIRCLE_OBJ_H
