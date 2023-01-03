#include "main.h"
#include "filters.h"
#include "math.h"


void init_biquad_filter (biquadFilter_t *filter,float f_cut, float frequency)
{

	filter->x1 = filter->x2 = 0;
	filter->y1 = filter->y2 = 0;

    const float omega = 2.0f * M_PI * (f_cut/frequency);
    const float sn = sin(omega);
    const float cs = cos(omega);
    const float alpha = sn / (2.0f * (1.0f / sqrtf(2.0f)));

    filter->b1 = 1 - cs;
    filter->b0 = filter->b1 * 0.5f;
    filter->b2 = filter->b0;
    filter->a1 = -2 * cs;
    filter->a2 = 1 - alpha;

    const float a0 = 1 + alpha;

    filter->b0 /= a0;
    filter->b1 /= a0;
    filter->b2 /= a0;
    filter->a1 /= a0;
    filter->a2 /= a0;

}

float biquadFilterApply(biquadFilter_t *filter,float input)
{
    const float result = filter->b0 * input + filter->x1;
    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;
    return result;
}


void init_pt1Filter(pt1Filter_t *filter,float f_cut, float dT)
{
    float RC = 1 / (2 * M_PI * f_cut);
    filter->k  = dT / (RC + dT);
    filter->state = 0.0f;
}

 float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}
 float pt1Filter(pt1Filter_t *filter, float input,float K)
{
    filter->state = filter->state + K * (input - filter->state);
    return filter->state;
}

