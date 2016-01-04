// PID implementation file by Hengjiu Kang
// For Competition: NATCAR
// Team by Jonathan Tao, Hengjiu Kang, Sirus
#include "PID.h"

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
void PID_setup(PID_Profile* profile, double ig,double pg,double dg, double imax, double imin){
	profile->dGain=dg;
	profile->iGain=ig;
	profile->pGain=pg;
	profile->iMax=imax;
	profile->iMin=imin;
}


/**
  * @brief  run a PID calculation
  * @param  profile
  * @param  error between FB and current position
  * @param  current position
  * @retval None
  */
double PID_run(PID_Profile* pid, double error, double position){
	double pTerm, dTerm, iTerm;
	pTerm = pid->pGain * error; // calculate the proportional term
	// calculate the integral state with appropriate limiting
	pid->iState += error;
	if (pid->iState > pid->iMax) pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) pid->iState = pid->iMin;
	iTerm = pid->iGain * (pid->iState); // calculate the integral term
	dTerm = pid->dGain * (pid->dState - position);
	pid->dState = position;
	return pTerm + dTerm + iTerm;
		//return pTerm + dTerm;
}
