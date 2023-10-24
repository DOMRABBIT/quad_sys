#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOinterface.h"
#include "interface/CmdPanel.h"
#include "common/robot.h"
#include "gait/WaveGenerator.h"
#include "control/Estimator.h"
#include "control/BalanceCtrl.h"
#include <string>
#include <iostream>

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif // COMPILE_DEBUG

struct CtrlComponents
{
public:
    CtrlComponents(IOinterface *ioInter, Dynamics *dy_) : ioInter(ioInter),dy(dy_)
    {
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState();
        contact = new VecInt4;
        phase = new Vec4;
        *contact = VecInt4(0, 0, 0, 0);
        *phase = Vec4(0.5, 0.5, 0.5, 0.5);
    }
    ~CtrlComponents()
    {
        delete lowCmd;
        delete lowState;
        delete ioInter;
        delete robotModel;
        delete waveGen;
        delete estimator;
        delete balCtrl;
        delete dy;
    }
    IOinterface *ioInter;
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    QuadrupedRobot *robotModel;
    WaveGenerator *waveGen;
    Estimator *estimator;
    BalanceCtrl *balCtrl;
    Dynamics *dy;

#ifdef COMPILE_DEBUG
    PyPlot *plot;
#endif // COMPILE_DEBUG

    VecInt4 *contact;
    Vec4 *phase;

    double dt;
    bool *running;
    CtrlPlatform ctrlPlatform;

    void sendRecv()
    {
        ioInter->sendRecv(lowCmd, lowState);
    }
    void runWaveGen()
    {
        waveGen->calcContactPhase(*phase, *contact, _waveStatus);
    }

    void setAllStance()
    {
        _waveStatus = WaveStatus::STANCE_ALL;
    }

    void setAllSwing()
    {
        _waveStatus = WaveStatus::SWING_ALL;
    }

    void setStartWave()
    {
        _waveStatus = WaveStatus::WAVE_ALL;
    }

    void geneObj()
    {
        estimator = new Estimator(robotModel, lowState, contact, phase, dt);
        balCtrl = new BalanceCtrl(robotModel);
    }

#ifdef COMPILE_DEBUG
        plot = new PyPlot();
        balCtrl->setPyPlot(plot);
        estimator->setPyPlot(plot);
#endif // COMPILE_DEBUG

private:
        WaveStatus _waveStatus = WaveStatus::SWING_ALL;
    
};

#endif // CTRLCOMPONENTS_H