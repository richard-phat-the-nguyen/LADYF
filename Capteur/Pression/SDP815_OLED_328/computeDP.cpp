#include "computeDP.h"

float computeDP(int N) {
    float x = (float)N / 1023.0f;   // AOut / VDD
    float s = (x > 0.5f) ? 1.0f : -1.0f;
    float term = (x / 0.4f) - 1.25f;
    return s * (term * term) * 525.0f;
}