#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/CmdPanel.h"
#include <string>

class IOinterface
{
public:
    IOinterface() {}
    ~IOinterface() {}
    virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0;
    void zeroCmdPanel() { cmdPanel->setZero(); }
    void setPassive() { cmdPanel->setPassive(); }

protected:
    CmdPanel *cmdPanel;
};

#endif // IOINTERFACE_H