#include "Mecanum.h"
#include "math.h"


void Mecanum_Init(Mecanum *mec,float r,float lx,float ly)
{
	mec->R = r;
	mec->Lx = lx;
	mec->Ly = ly;
	mec->W[4] = 0.0f;
	mec->V[3] = 0.0f;

}

void Forward_Kinematic(Mecanum *mec,float W1, float W2, float W3, float W4)
{
	mec->V[0] = (W1 + W2 + W3 + W4)*(mec->R/4);
	mec->V[1] = (-W1 + W2 + W3 - W4)*(mec->R/4);
	mec->V[2] = (-W1 + W2 - W3 + W4)*(mec->R/4*(mec->Lx+mec->Ly));
	
}

void Invers_Kinematic(Mecanum *mec,float Vx, float Vy, float Vyaw)
{

	mec->W[0] = (Vx - Vy - (mec->Lx + mec->Ly)*Vyaw)/mec->R;
	mec->W[1] = (Vx + Vy + (mec->Lx + mec->Ly)*Vyaw)/mec->R;
	mec->W[3] = (Vx + Vy - (mec->Lx + mec->Ly)*Vyaw)/mec->R;
	mec->W[4] = (Vx - Vy + (mec->Lx + mec->Ly)*Vyaw)/mec->R;

}
        
