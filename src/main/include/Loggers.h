#pragma once

#include <string.h>
#include <string_view>
#include "wpi/DataLog.h"
#include "frc/DataLogManager.h"
#include <wpi/DataLogReader.h>
#include <frc/smartdashboard/SmartDashboard.h>
#define LOGENABLED (true)

using namespace std;

class TalonXXV;

class CLoggers
{
public:
    CLoggers(string logname)
    {
        m_logName = logname;
        // Robot *ptrueRobot;
    }
    void Init();
    void LogStrings(vector<string> xyz);
    void LogDoubles(vector<double> xyz);
    void LogString(string xyz);
    void LogDouble(double xyz);

private:
    TalonXXV *m_mainRobot;
    wpi::log::BooleanLogEntry m_logBool;
    wpi::log::DoubleLogEntry m_logDouble;
    wpi::log::StringLogEntry m_logString;
    string m_logName;
};