#include "PID.h"
#include <iostream>
#include <ros/ros.h>
double maxDutySpeed = 1660, minDutySpeed = 1275;
void PIDInit (struct PID *pp){
    pp->LastError = 0;
    pp->PreError = 0;
    pp->intSum = 0;  
    pp->out = 0;              
}

double PIDCal(struct PID *pp, double ThisError, double dutySpeed){ 
    if(pp->mode == 1)
    {
        /*---------变速积分---------------*/
        double intFactor;
        if(fabs(ThisError) > 2.9)
        {
            intFactor = 0;
        }
        else if(fabs(ThisError) <= 2.9 && fabs(ThisError) > 2.7)
        {
            intFactor = (2.9 - fabs(ThisError)) / 0.2;
        }
        else
        {
            intFactor = 1;
        }

        /*-----------积分抗饱和-----------------*/
        if(dutySpeed >= maxDutySpeed - 20 && ThisError < 0)
        {
            pp->intSum += intFactor * ThisError;
        }
        if(dutySpeed <= 1500 && ThisError >0)
        {
            pp->intSum += intFactor * ThisError;
        }
        if(dutySpeed <= maxDutySpeed && dutySpeed >= minDutySpeed)
        {
            pp->intSum += intFactor * ThisError;
        }
        /*-----------积分限幅------------------*/
        pp->intSum = pp->intSum > 150 ? 150 : pp->intSum;
        pp->intSum = pp->intSum < -150 ? -150 : pp->intSum;
        ROS_INFO("intSum=%.2f",pp->intSum);
        pp->out = pp->Proportion * ThisError + pp->Int * pp->intSum + pp->Derivative * (ThisError - pp->LastError);
        ROS_INFO("out=%.2f",pp->out);
        pp->LastError = ThisError; 
        return pp->out;
    }
    if(pp->mode == 0)
    {
        pp->intSum += ThisError;
        pp->out = pp->Proportion * ThisError + pp->Int * pp->intSum + pp->Derivative * (ThisError - pp->LastError);
        pp->LastError = ThisError;
        return pp->out;
    }
}
