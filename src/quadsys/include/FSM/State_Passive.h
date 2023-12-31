#ifndef PASSIVE_H
#define PASSIVE_H

#include "FSM/FSMState.h"

class State_Passive : public FSMState
{
public:
    State_Passive(CtrlComponents *ctrlComp);
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
};

#endif // PASSIVE_H