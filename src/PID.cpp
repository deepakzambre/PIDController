#include "PID.h"

using namespace std;

PID::PID()
{
}

PID::~PID()
{
}

void
PID::Init(double kp_in, double ki_in, double kd_in)
{
  kp = kp_in;
  ki = ki_in;
  kd = kd_in;

  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void
PID::UpdateError(double cte)
{
  p_error = cte;
  i_error += cte;
  d_error = cte - d_error;
}

double
PID::TotalError()
{
  return -1.0 * (kp * p_error + ki * i_error + kd * d_error);
}
