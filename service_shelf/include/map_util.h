/*
 * map_util.h
 *
 *  Created on: Oct 20, 2020
 *      Author: meijian
 */

#ifndef UTIL_MAP_UTIL_H_
#define UTIL_MAP_UTIL_H_

#include <math.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include "angle_util.h"

template <class T>
struct point
{
	inline point() : x(0), y(0) {}
	inline point(T _x, T _y) : x(_x), y(_y) {}
	inline void Swap() { std::swap(x, y); }

	T x, y;
};

struct LaserPoint {
  //! lidar angle　[rad]
  float angle;
  //! lidar range [m]
  float range;
  //! lidar intensity
  float intensity;
  LaserPoint &operator = (const LaserPoint &data) {
    this->angle = data.angle;
    this->range = data.range;
    this->intensity = data.intensity;
    return *this;
  }
};
template <class T>
inline point<T> operator * (const T& factor, const point<T>& p)
{
	return point<T>(p.x*factor, p.y*factor);
}

template <class T>
inline point<T> operator + (const point<T>& p1, const point<T>& p2)
{
	return point<T>(p1.x+p2.x, p1.y+p2.y);
}

template <class T>
inline point<T> operator - (const point<T>& p1, const point<T>& p2)
{
	return point<T>(p1.x-p2.x, p1.y-p2.y);
}

template <class T>
inline T operator * (const point<T>& p1, const point<T>& p2)
{
	return (p1.x*p2.x + p1.y*p2.y);
}


template <class T>
inline T Distance(const point<T>& p1, const point<T>& p2)
{
	return sqrtf( (p1-p2) * (p1-p2) );
}

typedef point<int> PointI;
typedef point<float> PointF;

struct Pose : public PointF
{
	inline Pose() : point(), a(0) {}
	inline Pose(float _x, float _y, float _a):
			point(_x, _y), a(_a) {}

	inline Pose Move(Pose relPose)
	{
		float dx = relPose.x, dy = relPose.y, da = relPose.a;
		float sina = sin(a), cosa = cos(a);
		float ex = dx * cosa - dy * sina + x;
		float ey = dx * sina + dy * cosa + y;
		float ea = Mod2Pi(da + a);
		return Pose(ex, ey, ea);
	}

	inline void Reset()
	{
		x = 0;
		y = 0;
		a = 0;
	}

	float a;
};

inline Pose operator * (const float& factor, const Pose& p)
{
	return Pose(p.x*factor, p.y*factor, p.a * factor);
}

inline Pose operator + (const Pose& p1, const Pose& p2)
{
	return Pose(p1.x + p2.x, p1.y + p2.y, p1.a + p2.a);
}

inline Pose operator - (const Pose& p1, const Pose& p2)
{
	return Pose(p1.x - p2.x, p1.y - p2.y, p1.a - p2.a);
}

inline bool LaserHitPoint(Pose lidarPose, LaserPoint laserPoint, PointF& pHit)
{
	if( laserPoint.range > 0 )
	{
		//float laserAngle = laserPoint.angle + PI/2;
		float laserAngle = laserPoint.angle ;
		pHit = /*lidarPose + */laserPoint.range * PointF(cos(laserAngle), sin(laserAngle));
		return true;
	}
	return false;
}


// TODO: This could be more accurate with PointF as input arguments
inline std::vector<PointI> GetLinePoints(PointI p1, PointI p2)
{
	std::vector<PointI> linePoints;

	PointI start, end;
	bool reverse = false;
	if( p1.x <= p2.x )
	{
		start = p1; end = p2;
	}
	else
	{
		start = p2; end = p1;
		reverse = true;
	}

	// Flip around X axis, to make start.y <= end.y
	bool flipX = false;
	if( start.y > end.y )
	{
		flipX = true;
		start.y = -start.y;
		end.y = -end.y;
	}
	// Flip around line x=y
	bool flip1 = false;
	int dx = end.x - start.x;
	int dy = end.y - start.y;
	if(dy > dx)
	{
		flip1 = true;
		start.Swap();
		end.Swap();
		std::swap(dx, dy);
	}

	int d = 2 * dy - dx;
	int incr1 = 2 * dy;
	int incr2 = 2 * ( dy - dx );
	int x = start.x, y = start.y;
	linePoints.push_back(start);
	while(x < end.x)
	{
		x++;
		if( d < 0 )
		{
			d += incr1;
		}
		else
		{
			y++;
			d += incr2;
		}
		linePoints.push_back(PointI(x, y));
	}

	// Flip back
	if( flip1 )
	{
		for(std::vector<PointI>::iterator it = linePoints.begin(); it != linePoints.end(); it++)
		{
			it->Swap();
		}
	}
	if( flipX )
	{
		for(std::vector<PointI>::iterator it = linePoints.begin(); it != linePoints.end(); it++)
		{
			it->y = -it->y;
		}
	}

	if(reverse)
		std::reverse(linePoints.begin(), linePoints.end());
	return linePoints;
}


inline PointF PolarToPlanar(float range, float angle)
{
	float x = range * cos(angle);
	float y = range * sin(angle);
	return PointF(x, y);
}


inline float PointDist(PointF p1, PointF p2)
{
	return sqrt( ( p1-p2 ) * ( p1-p2 ) );
}


inline float PointLineDist(PointF pnt, Pose line)
{
	float a = line.a;
	float cosa = cos(a);
	float sina = sin(a);
	float dx = line.x - pnt.x;
	float dy = line.y - pnt.y;
	float t = -dx*cosa - dy*sina;
	return sqrtf( pow(t*cosa+dx,2) + pow(t*sina+dy,2) );
}

#endif /* UTIL_MAP_UTIL_H_ */
