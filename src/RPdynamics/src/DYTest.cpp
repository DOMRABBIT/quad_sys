#include "RPBody.h"
#include "RPJoint.h"
#include "RPRobot.h"
#include "Build_A1.h"

int main()
{
    Robot *a1 = new Robot(12, 4);
    build_a1(a1);
    // Loop joint and floating base settings are not completed!

    return 0;
}