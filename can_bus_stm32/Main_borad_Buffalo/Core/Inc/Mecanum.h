#ifndef Mecanum_H
#define Mecanum_H

#include <math.h>

typedef struct {

	float R;
	float Lx;
	float Ly;
	float V[3];
	float W[4];


} Mecanum;

void Mecanum_Init(Mecanum *mec, float r, float lx, float ly);

void Forward_Kinematic(Mecanum *mec,float W1, float W2, float W3, float W4);

void Invers_Kinematic(Mecanum *mec,float Vx, float Vy, float Vyaw);
#endif
