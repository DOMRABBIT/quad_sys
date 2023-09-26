#include "RPBody.h"
#include "RPJoint.h"
#include "RPRobot.h"
#include "Build_A1.h"

using namespace std;
int main()
{
    Robot *a1 = new Robot(12, 4);
    build_a1(a1);

    cout << a1->T_dwtree[1] << endl;
    return 0;
}