#include "dispatch.h"
#include <Windows.h>

int main()
{
    Environment state(Rect_2(Point_2(200,200),Point_2(-200,-200)), 10);

    state.add_robot(Point_2(-1, 4));
    state.add_robot(Point_2(0, 6));
	state.add_robot(Point_2(6, 0));

    state.initialization();

    Dispatch dis(&state);
	dis.Turn_taking();

    state.createLTS();

}







