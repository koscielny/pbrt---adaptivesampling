


// samplers/MyAdaptive.cpp*
#include "stdafx.h"
#include "MyAdaptive.h"
#include "paramset.h"
#include "film.h"
#include "primitive.h"
#include "intersection.h"
#include "camera.h"
#include "montecarlo.h"
#include "../renderers/samplerrenderer.h"
#include <fstream>
#include <iostream>

/****
*****使用myadaptive要设置bool MyAdaptiveSampler
****/
//extern float** VTable;
// AdaptiveSampler Method Definitions
MYAdaptiveSampler::MYAdaptiveSampler(int xstart, int xend,
	int ystart, int yend, int mins, int maxs,
	float sopen, float sclose,int xres,int yres, int BX)
	: Sampler(xstart, xend, ystart, yend, RoundUpPow2(max(mins, maxs)),
	sopen, sclose) {
	BucketX = BX;
	xPos = xPixelStart;
	yPos = yPixelStart;
	xRes = xres;
	yRes = yres;
	if (mins > maxs) std::swap(mins, maxs);

//	if (!IsPowerOf2(mins)) {
//		Warning("Minimum pixel samples being rounded up to power of 2");
//		minSamples = RoundUpPow2(mins);
//	}
//	else
		minSamples = mins;
//	if (!IsPowerOf2(maxs)) {
//		Warning("Maximum pixel samples being rounded up to power of 2");
//		maxSamples = RoundUpPow2(maxs);
//	}
//	else
		maxSamples = maxs;

	if (minSamples < 2) {
		Warning("Adaptive sampler needs at least two initial pixel samples.  Using two.");
		minSamples = 2;
	}
	if (minSamples == maxSamples) {
		maxSamples *= 2;
		Warning("Adaptive sampler must have more maximum samples than minimum.  Using %d - %d",
			minSamples, maxSamples);
	}

	sampleBuf = NULL;
}


MYAdaptiveSampler::~MYAdaptiveSampler() {
	delete[] sampleBuf;
}


Sampler *MYAdaptiveSampler::GetSubSampler(int num, int count) {
	int x0, x1, y0, y1;
	ComputeSubWindow(num, count, &x0, &x1, &y0, &y1);
	if (x0 == x1 || y0 == y1) return NULL;
	return new MYAdaptiveSampler(x0, x1, y0, y1, minSamples, maxSamples,
		shutterOpen, shutterClose,xRes,yRes,BucketX);
}


int MYAdaptiveSampler::GetMoreSamples(Sample *samples, RNG &rng, float** vt, float tv ) {
	if (!sampleBuf)
		sampleBuf = new float[LDPixelSampleFloatsNeeded(samples,
		maxSamples)];

	if (yPos == yPixelEnd) return 0;
	//extern float** VTable;
	float factor;
//	Warning("%d,%d", xPos, yPos);
//	Warning("%d,%d,%f", xRes, yRes,tv);

	if (vt[xPos][yPos] != -1){
		factor = vt[xPos][yPos] * xRes * yRes / tv;	//这几个是samplerrenderer.h里定义，samplerrenderer.cpp里获得值的全局变量
	}
	else factor = 0;//表项为空。。投射的cast_ray没交到体素，1、有可能真没有，2、有可能太细小，反而需要大量采样？：假设该像素大部分区域是空（黑），它自然是黑的更好
	std::ofstream LOG2("自适应采样率系数_GetMoreSamples", std::ios::app);

	int SamplingRate = minSamples + factor*BucketX;//16or8//ly:bucket
	SamplingRate = Clamp(SamplingRate,minSamples,maxSamples);

	LOG2 << "\"" << xPos << "\t" << yPos << "\"" << "\t" << factor << "\t" << vt[xPos][yPos] << "\t" << SamplingRate << std::endl;//Warning("%f", TotalV);

	LDPixelSample(xPos, yPos, shutterOpen, shutterClose, SamplingRate,
		samples, sampleBuf, rng);
	return SamplingRate;
}


bool MYAdaptiveSampler::ReportResults(Sample *samples,
	const RayDifferential *rays, const Spectrum *Ls,
	const Intersection *isects, int count) {
	/*
	if (supersamplePixel) {
		supersamplePixel = false;
		// Advance to next pixel for sampling for _AdaptiveSampler_
		if (++xPos == xPixelEnd) {
			xPos = xPixelStart;
			++yPos;
		}
		return true;
	}
	else if (needsSupersampling(samples, rays, Ls, isects, count)) {
		PBRT_SUPERSAMPLE_PIXEL_YES(xPos, yPos);
		supersamplePixel = true;
		return false;
	}

	else
	*/ 
	{
		PBRT_SUPERSAMPLE_PIXEL_NO(xPos, yPos);
		// Advance to next pixel for sampling for _AdaptiveSampler_
		if (++xPos == xPixelEnd) {
			xPos = xPixelStart;
			++yPos;
		}
		return true;
	}
}


MYAdaptiveSampler *CreateMyAdaptiveSampler(const ParamSet &params, const Film *film,
		const Camera *camera) {
	// Initialize common sampler parameters
	int xstart, xend, ystart, yend;
	film->GetStableSampleExtent(&xstart, &xend, &ystart, &yend);//万岁。。。边界收回到0-Resolution。。
	int minsamp = params.FindOneInt("minsamples", 4);
	int maxsamp = params.FindOneInt("maxsamples", 64);
	int bucketX = params.FindOneInt("bucketX", 8);
	if (PbrtOptions.quickRender) { minsamp = 2; maxsamp = 4; }
	//string method = params.FindOneString("method", "contrast");
	return new MYAdaptiveSampler(xstart, xend, ystart, yend, minsamp, maxsamp, 
		camera->shutterOpen, camera->shutterClose, camera->film->xResolution, camera->film->yResolution, bucketX);
}




/*

bool MYAdaptiveSampler::needsSupersampling(Sample *samples,
const RayDifferential *rays, const Spectrum *Ls,
const Intersection *isects, int count) {
switch (method) {
case ADAPTIVE_COMPARE_SHAPE_ID:
// See if any shape ids differ within samples
for (int i = 0; i < count - 1; ++i)
if (isects[i].shapeId != isects[i + 1].shapeId ||
isects[i].primitiveId != isects[i + 1].primitiveId)
return true;
return false;
case ADAPTIVE_CONTRAST_THRESHOLD:
// Compare contrast of sample differences to threshold
float Lavg = 0.f;
for (int i = 0; i < count; ++i)
Lavg += Ls[i].y();
Lavg /= count;
const float maxContrast = 0.5f;
for (int i = 0; i < count; ++i)
if (fabsf(Ls[i].y() - Lavg) / Lavg > maxContrast)
return true;
return false;
}
return false;
}

*/