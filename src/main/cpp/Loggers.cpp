#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/DataLogReader.h>
#include <iostream>
#include <Loggers.h>
#include <string.h>

using namespace std;

// One function works for all data types.  This would work
// even for user defined types if operator '>' is overloaded

void CLoggers::Init()
{
  // Read m_LogEnabled setting files.
  if (LOGENABLED)
  {
    // Required for custom logging
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();

    m_logBool = wpi::log::BooleanLogEntry(log, m_logName + ("/boolean"));
    m_logDouble = wpi::log::DoubleLogEntry(log, m_logName + ("/double"));
    m_logString = wpi::log::StringLogEntry(log, m_logName + ("/string"));
  }
}

/**
 * @brief Logs a single string
 *
 * @param xyz is a string
 */
void CLoggers::LogString(string xyz)
{
  if (LOGENABLED)
  {
    m_logString.Append(xyz);
  }
}

/**
 * @brief Logs a single double
 *
 * @param xyz is a double
 */
void CLoggers::LogDouble(double xyz)
{
  if (LOGENABLED)
  {
    m_logDouble.Append(xyz);
  }
}

/**
 * @brief LogStrings logs an infinite number of strings
 *
 * @param xyz is a vector(infinite amount of storage) of type string
 */
void CLoggers::LogStrings(vector<string> xyz)
{
  if (LOGENABLED)
  {
    string endResultStr = "";
    for (const string &word : xyz)
    {
      endResultStr += word + ",";
    }
    endResultStr.pop_back();
    m_logString.Append(endResultStr);
  }
}

/**
 * @brief LogDoubles logs an infinite number of doubles
 *
 * @param xyz is a vector(infinite amount of storage) of type double
 */
void CLoggers::LogDoubles(vector<double> xyz)
{
  if (LOGENABLED)
  {
    string endResultDouble = "";
    for (const double &number : xyz)
    {
      endResultDouble += std::to_string(number) + ",";
    }
    endResultDouble.pop_back();
    m_logString.Append(endResultDouble);
  }
}
