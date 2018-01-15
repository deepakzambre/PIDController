#include "PID.h"
#include <math.h>

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
}

void
PID::UpdateError(double cte)
{
  d_error = cte - p_error;
  p_error = cte;
  i_error = i_error + cte;
}

double
PID::TotalError()
{
  return -1.0 * (kp * p_error + ki * i_error + kd * d_error);
}
