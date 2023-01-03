#ifndef FILTERS_H
#define FILTERS_H

typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquadFilter_t;

typedef struct pt1Filter_s{
	float k;
	float state;
}pt1Filter_t;



void init_biquad_filter (biquadFilter_t *filter,float f_cut, float frequency);
float biquadFilterApply(biquadFilter_t *filter,float input);

void init_pt1Filter(pt1Filter_t *filter,float f_cut, float dT);
float pt1FilterApply(pt1Filter_t *filter, float input);

float pt1Filter(pt1Filter_t *filter, float input,float K);

#endif
