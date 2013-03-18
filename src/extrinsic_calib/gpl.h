#ifndef GPL_H
#define GPL_H

#include <algorithm>
#include <cmath>
#include <opencv2/core/core.hpp>

#include "vcharge.h"

namespace vcharge
{

template<class T>
const T VCHARGE_DLL_EXPORT clamp(const T& v, const T& a, const T& b)
{
	return std::min(b, std::max(a, v));
}

double VCHARGE_DLL_EXPORT hypot3(double x, double y, double z);
float VCHARGE_DLL_EXPORT hypot3f(float x, float y, float z);

template<class T>
const T VCHARGE_DLL_EXPORT normalizeTheta(const T& theta)
{
	T normTheta = theta;

	while (normTheta < - M_PI)
	{
		normTheta += 2.0 * M_PI;
	}
	while (normTheta > M_PI)
	{
		normTheta -= 2.0 * M_PI;
	}

	return normTheta;
}

double VCHARGE_DLL_EXPORT d2r(double deg);
float VCHARGE_DLL_EXPORT d2r(float deg);
double VCHARGE_DLL_EXPORT r2d(double rad);
float VCHARGE_DLL_EXPORT r2d(float rad);

double sinc(double theta);

template<class T>
const T VCHARGE_DLL_EXPORT square(const T& x)
{
	return x * x;
}

template<class T>
const T VCHARGE_DLL_EXPORT cube(const T& x)
{
	return x * x * x;
}

template<class T>
const T VCHARGE_DLL_EXPORT random(const T& a, const T& b)
{
	return static_cast<double>(rand()) / RAND_MAX * (b - a) + a;
}

template<class T>
const T VCHARGE_DLL_EXPORT randomNormal(const T& sigma)
{
    T x1, x2, w;

    do
    {
        x1 = 2.0 * random(0.0, 1.0) - 1.0;
        x2 = 2.0 * random(0.0, 1.0) - 1.0;
        w = x1 * x1 + x2 * x2;
    }
    while (w >= 1.0 || w == 0.0);

    w = sqrt((-2.0 * log(w)) / w);

    return x1 * w * sigma;
}

unsigned long long VCHARGE_DLL_EXPORT timeInMicroseconds(void);

double VCHARGE_DLL_EXPORT timeInSeconds(void);

void VCHARGE_DLL_EXPORT colorDepthImage(cv::Mat& imgDepth,
										cv::Mat& imgColoredDepth,
										float minRange, float maxRange);

bool VCHARGE_DLL_EXPORT colormap(const std::string& name, unsigned char idx,
								 float& r, float& g, float& b);

std::vector<cv::Point2i> VCHARGE_DLL_EXPORT bresLine(int x0, int y0, int x1, int y1);
std::vector<cv::Point2i> VCHARGE_DLL_EXPORT bresCircle(int x0, int y0, int r);

void VCHARGE_DLL_EXPORT LLtoUTM(double latitude, double longitude,
								double& utmNorthing, double& utmEasting,
								std::string& utmZone);
void VCHARGE_DLL_EXPORT UTMtoLL(double utmNorthing, double utmEasting,
								const std::string& utmZone,
								double& latitude, double& longitude);

long int timestampDiff(uint64_t t1, uint64_t t2);

}

#endif
