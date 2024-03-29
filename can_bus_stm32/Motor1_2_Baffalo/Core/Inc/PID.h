#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
	/* Derivative low-pass filter time constant */
	float tau;
	/* Output limits */
	float limMin;
	float limMax;
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;
	/* Sample time (in seconds) */
	float T;
	/* Controller "memory" */
	float integrator[4];
	float prevError[4];			/* Required for integrator */
	float differentiator[4];
	float prevMeasurement[4];		/* Required for differentiator */

	/* Controller output */
	float out[4];

} PIDController;

void  PID_Init(PIDController *pid, int nMotor);
float PID(PIDController *pid, float setpoint, float measurement,float Kp,float Ki,float Kd, int i);

#endif
