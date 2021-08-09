/*
 * angle_util.h
 *
 *  Created on: Nov 9, 2020
 *      Author: meijian
 */

#ifndef UTIL_ANGLE_UTIL_H_
#define UTIL_ANGLE_UTIL_H_

#include <math.h>

#define PI 3.1415927

inline float Mod2Pi(float a)
{
	float b = a;
	while( b < 0 )
		b += 2*PI;
	while( b >= 2*PI )
		b -= 2*PI;
	return b;
	//return a;
}
inline void ToPi(float &A)
{
    while (A > M_PI)
        A -= 2 * M_PI;
    while (A < -M_PI)
        A += 2 * M_PI;
}
inline bool PlanarToPolar(float x, float y, float &r, float &p)
{
	r = sqrt( x*x + y*y );
	if( r < 0.00001 )
		return false;
	else
	{
		p = atan2(y, x);
		p = Mod2Pi(p);
		return true;
	}
}

inline float AbsAngleDiff( float a, float b )
{
	return fabs( Mod2Pi( a - b ) );
}

inline float DegreeToRad( float deg )
{
	return Mod2Pi( deg * PI / 180.0 );
}

#endif /* UTIL_ANGLE_UTIL_H_ */
