// PID implementation file by Hengjiu Kang
// For Competition: NATCAR
// Team by Jonathan Tao, Hengjiu Kang, Sirus

#ifndef PID_H
#define PID_H

typedef struct 
{
	double dState; // Last position input
	double iState; // Integrator state
	double iMax, iMin; // Maximum and minimum allowable integrator stat
	double iGain; // integral gain
	double pGain; // proportional gain
	double dGain; // derivative gain
}PID_Profile;


/**
  * @brief  initialize and setup a PID_profile
  * @param  profile
  * @param  integral gain
  * @param  proportional gain
  * @param  differential gain
  * @param  integral max value 
  * @param  integral min value
  * @retval None
  */
void PID_setup(PID_Profile* profile, double ig, double pg, double dg, double imax, double imin);


/**
  * @brief  run a PID calculation
  * @param  profile
  * @param  error between FB and current position
  * @param  current position
  * @retval None
  */
double PID_run(PID_Profile* profile, double error, double position);

#endif
