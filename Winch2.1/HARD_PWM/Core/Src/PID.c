/*
 * MPU9250.h
 *
 *  Created on: 5-Nov-2021
 *      Author: Venom
 */

#include <PID.h>

/*
    In the main do the following,

    Set the Kp, Ki, Kd.
    Set the Max/Min lim for antiwind up.
    Set the Max/Min lim for System anti saturation.
*/

/*Initializer*/
uint8_t PID_Init(PID_Handle_t *pid)
{

    /*Clear all the residuals*/

    pid->derivative = 0.0f;
    pid->integrator = 0.0f;
    pid->propotional = 0.0f;

    pid->prevErr = 0.0f;
    pid->prevMeasure = 0.0f;

    pid->pidout = 0.0f;

    return 1;
}

/*PID computation API*/
float PID_Compute(PID_Handle_t *pid, float measurement, float setPoint)
{
    float err = setPoint - measurement;

    /*propotional*/
    pid->propotional = pid->kp * err;

    /*intergral*/
    pid->integrator += 0.5f * pid->Ts * pid->ki * (err + pid->prevErr);//summation

    /*
        Clamping logic for integral antiwindup
    */
    if(pid->integrator > pid->limMaxInt)
    	pid->integrator = pid->limMaxInt;
    else if(pid->integrator < pid->limMinInt)
    	pid->integrator = pid->limMinInt;


    /*Diffrentiator*/ //(Band Limited )//
    //After accounting the effect of increased BW aka increased High freq response 
    //And the Derivative kick effect.
    pid->derivative = - (2.0f * pid->kd * (measurement - pid->prevMeasure) + (2.0f *pid->tau + pid->Ts) * pid->derivative)
                        /(2.0f * pid->tau + pid->Ts);


    //And finally the output 
    
    pid->pidout = pid->propotional + pid->integrator + pid->derivative;
    //Wait wait, where you running at??

    //Acount the system saturation effect // scale and offset
    if(pid->pidout > pid->limMax)
        pid->pidout = pid->limMax;

    else if(pid->pidout < pid->limMin)
        pid->pidout = pid->limMin;

    //Variable exchange
    pid->prevErr = err;
    pid->prevMeasure = measurement;

    //Okay now!
    return pid->pidout;
}
