#ifndef PID_AutoTune_v0
#define PID_AutoTune_v0
#define LIBRARY_VERSION	0.0.1

typedef struct
{
	char isMax, isMin;
	float *input, *output;
	float setpoint;
	float noiseBand;
	int controlType;
	char running;
	unsigned long peak1, peak2, lastTime;
	int sampleTime;
	int nLookBack;
	int peakType;
	float lastInputs[101];
	float peaks[10];
	int peakCount;
	char justchanged;
	char justevaled;
	float absMax, absMin;
	float oStep;
	float outputStart;
	float Ku, Pu;	
}PID_ATUNE_T;


void PIDA_ATuneInit(PID_ATUNE_T *pid_atune, float* Input, float* Output);
void PIDA_Cancel(PID_ATUNE_T *pid_atune);
int PIDA_Runtime(PID_ATUNE_T *pid_atune);
float PIDA_GetKp(PID_ATUNE_T *pid_atune);
float PIDA_GetKi(PID_ATUNE_T *pid_atune);
float PIDA_GetKd(PID_ATUNE_T *pid_atune);
void PIDA_SetOutputStep(PID_ATUNE_T *pid_atune, float Step);
float PIDA_GetOutputStep(PID_ATUNE_T *pid_atune);
void PIDA_SetControlType(PID_ATUNE_T *pid_atune, int Type); //0=PI, 1=PID
int PIDA_GetControlType(PID_ATUNE_T *pid_atune);
void PIDA_SetNoiseBand(PID_ATUNE_T *pid_atune, float Band);
float PIDA_GetNoiseBand(PID_ATUNE_T *pid_atune);
void PIDA_SetLookbackSec(PID_ATUNE_T *pid_atune, int value);
int PIDA_GetLookbackSec(PID_ATUNE_T *pid_atune);


#endif

