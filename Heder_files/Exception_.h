#ifndef EXCEPTION__H
#define EXCEPTION__H

#include <CGAL/Cartesian.h>
#include <CGAL/Exact_rational.h>

#include <iostream>
#include <QString>

typedef CGAL::Cartesian<CGAL::Exact_rational>         Kernel;
typedef Kernel::Point_2                               Point_2;

class Exception_
{
public:
    Exception_() {}

     QString invalid_hole_position(Point_2 p){
         QString error = "The point (" + QString::number(CGAL::to_double(p.x())) +
                         ", " + QString::number(CGAL::to_double(p.y())) +
                         "of hole is out of the environment.";

         std::cout<<"The point ("<< p.x() <<", "
                  << p.y() << "of hole is out of the environment.";
         return error;
    }


private:


};



#endif // EXCEPTION__H
