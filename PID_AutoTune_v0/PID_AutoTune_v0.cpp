#include "PID_AutoTune_v0.h"


static void FinishUp(PID_ATUNE *pid_atune);


void PIDA_ATuneInit(PID_ATUNE_T *pid_atune, float* Input, float* Output)
{
	memset(pid_atune,0x0,sizeof(pid_atune));
	pid_atune->input = Input;
	pid_atune->output = Output;
	pid_atune->controlType = 0 ; //default to PI
	pid_atune->noiseBand = 0.5;
	pid_atune->running = FALSE;
	pid_atune->oStep = 30;
	SetLookbackSec(10);
	pid_atune->lastTime = millis();
}

void PIDA_Cancel(PID_ATUNE_T *pid_atune)
{
	pid_atune->running = FALSE;
} 
 
int PIDA_Runtime(PID_ATUNE_T *pid_atune)
{
	unsigned long now;
	float refVal;
	int i;
	
	pid_atune->justevaled = FALSE;
	if(pid_atune->peakCount > 9 && pid_atune->running)
	{
		pid_atune->running = FALSE;
		FinishUp(pid_atune);
		return TRUE;
	}
	
	now = millis();
	if((now - pid_atune->lastTime) < pid_atune->sampleTime) 
	{
		return FALSE;
	}
	pid_atune->lastTime = now;
	refVal = *pid_atune->input;
	pid_atune->justevaled = TRUE;
	if(!pid_atune->running)
	{ //initialize working variables the first time around
		pid_atune->peakType = 0;
		pid_atune->peakCount = 0;
		pid_atune->justchanged = FALSE;
		pid_atune->absMax = refVal;
		pid_atune->absMin = refVal;
		pid_atune->setpoint = refVal;
		pid_atune->running = TRUE;
		pid_atune->outputStart = *pid_atune->output;
		*pid_atune->output = pid_atune->outputStart + pid_atune->oStep;
	}
	else
	{
		if(pid_atune->refVal > pid_atune->absMax)
		{
			pid_atune->absMax = pid_atune->refVal;
		}
		if(pid_atune->refVal < pid_atune->absMin)
		{
			pid_atune->absMin = pid_atune->refVal;
		}
	}
	
	//oscillate the output base on the input's relation to the setpoint
	if(pid_atune->refVal > pid_atune->setpoint + pid_atune->noiseBand) 
	{
		*pid_atune->output = pid_atune->outputStart - pid_atune->oStep;
	}
	else if (pid_atune->refVal < pid_atune->setpoint - pid_atune->noiseBand) 
	{
		*pid_atune->output = pid_atune->outputStart + pid_atune->oStep;
	}
	
	
  //bool isMax=true, isMin=true;
  pid_atune->isMax = TRUE;
  pid_atune->isMin = TRUE;
  //id peaks
  for(i= pid_atune->nLookBack - 1;i >= 0; i--)
  {
    float val = pid_atune->lastInputs[i];
    if(pid_atune->isMax)
    {
    	pid_atune->isMax = pid_atune->refVal>val;
    }
    if(pid_atune->isMin)
    {
    	pid_atune->isMin = pid_atune->refVal<val;
    }
    pid_atune->lastInputs[i+1] = pid_atune->lastInputs[i];
  }
  pid_atune->lastInputs[0] = refVal;  
  if(pid_atune->nLookBack < 9)
  {  //we don't want to trust the maxes or mins until the inputs array has been filled
	return FALSE;
  }
  
  if(pid_atune->isMax)
  {
    if(pid_atune->peakType == 0)
    {
    	pid_atune->peakType = 1;
    }
    if(pid_atune->peakType == -1)
    {
      pid_atune->peakType = 1;
      pid_atune->justchanged = TRUE;
      pid_atune->peak2 = pid_atune->peak1;
    }
    pid_atune->peak1 = now;
    pid_atune->peaks[pid_atune->peakCount] = refVal;
   
  }
  else if(pid_atune->isMin)
  {
    if(pid_atune->peakType == 0)
    {
    	peakType = -1;
    }
    if(pid_atune->peakType == 1)
    {
      pid_atune->peakType = -1;
      pid_atune->peakCount++;
      pid_atune->justchanged = TRUE;
    }
    
    if(pid_atune->peakCount < 10)
    {
    	pid_atune->peaks[pid_atune->peakCount] = refVal;
    }
  }
  
  if(pid_atune->justchanged && pid_atune->peakCount > 2)
  { //we've transitioned.  check if we can autotune based on the last peaks
    float avgSeparation = (abs(pid_atune->peaks[pid_atune->peakCount-1] - pid_atune->peaks[pid_atune->peakCount-2])
			 + abs(pid_atune->peaks[pid_atune->peakCount-2] - pid_atune->peaks[pid_atune->peakCount - 3]))/2;
    if( avgSeparation < 0.05*(absMax-absMin))
    {
	FinishUp(pid_atune);
	pid_atune->running = FALSE;
	return TRUE;
	 
    }
  }
   pid_atune->justchanged = FALSE;
	return FALSE;
}
static void FinishUp(PID_ATUNE_T *pid_atune)
{
	  *pid_atune->output = pid_atune->outputStart;
      //we can generate tuning parameters!
      pid_atune->Ku = 4*(2*pid_atune->oStep)/((pid_atune->absMax - pid_atune->absMin)*3.14159);
      pid_atune->Pu = (float)(pid_atune->peak1 - pid_atune->peak2) / 1000;
}

float PIDA_GetKp(PID_ATUNE_T *pid_atune)
{
	return pid_atune->controlType == 1 ? 0.6 * pid_atune->Ku : 0.4 * pid_atune->Ku;
}

float PIDA_GetKi(PID_ATUNE_T *pid_atune)
{
	return pid_atune->controlType == 1? 1.2*pid_atune->Ku / pid_atune->Pu : 0.48 * pid_atune->Ku / pid_atune->Pu;  // Ki = Kc/Ti
}

float PIDA_GetKd(PID_ATUNE_T *pid_atune)
{
	return pid_atune->controlType == 1? 0.075 * pid_atune->Ku * pid_atune->Pu : 0;  //Kd = Kc * Td
}

void PIDA_SetOutputStep(PID_ATUNE_T *pid_atune, float Step)
{
	pid_atune->oStep = Step;
}

float PIDA_GetOutputStep(PID_ATUNE_T *pid_atune)
{
	return pid_atune->oStep;
}

void PIDA_SetControlType(PID_ATUNE_T *pid_atune, int Type) //0=PI, 1=PID
{
	pid_atune->controlType = Type;
}
int PIDA_GetControlType(PID_ATUNE_T *pid_atune)
{
	return pid_atune->controlType;
}
	
void PIDA_SetNoiseBand(PID_ATUNE_T *pid_atune, float Band)
{
	pid_atune->noiseBand = Band;
}

float PIDA_GetNoiseBand(PID_ATUNE_T *pid_atune)
{
	return pid_atune->noiseBand;
}

void PIDA_SetLookbackSec(PID_ATUNE_T *pid_atune, int value)
{
    if (value < 1) value = 1;
	
	if(value < 25)
	{
		pid_atune->nLookBack = value * 4;
		pid_atune->sampleTime = 250;
	}
	else
	{
		pid_atune->nLookBack = 100;
		pid_atune->sampleTime = value*10;
	}
}

int PIDA_GetLookbackSec(PID_ATUNE_T *pid_atune)
{
	return pid_atune->nLookBack * pid_atune->sampleTime / 1000;
}
