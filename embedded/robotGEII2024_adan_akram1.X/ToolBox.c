#include "Toolbox.h"
#include "main.h"
#define PI 3.141592653589793


float RadianToDegree(float value) {
    return value / PI * 180.0;
}

float DegreeToRadian(float value) {
    return value * PI / 180.0;
}
