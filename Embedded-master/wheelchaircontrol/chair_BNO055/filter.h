#ifndef FILTER_H
#define FILTER_H

float lowPass(float sample);
float complement(float x, float y , float ratio);
float boxcar(float sample);
#endif