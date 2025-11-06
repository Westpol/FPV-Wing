#include <stdio.h>
#include <stdint.h>
#include <math.h>

void quaternion_product(const float* q1,const float* q2, float* q3){		// calculates q1*q2, saves value in q3
	float q_new[4];
	q_new[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
	q_new[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
	q_new[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
	q_new[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];

	q3[0] = q_new[0]; q3[1] = q_new[1]; q3[2] = q_new[2]; q3[3] = q_new[3];
}

float radians(float degrees){
	return degrees * (M_PI / 180.0f);
}

float degrees(float radians){
	return radians * (180.0f / M_PI)
}

int main(char argv[], int argc){
    float q[4] = {1, 0, 0, 0};
	float wx = radians(15);
	float wy = radians(25);
    float q_x[4] = {};
    float q_y[4] = {};
    float q_z[4] = {};
    return 0;
}