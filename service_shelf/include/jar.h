/*
 * jar.h
 *
 *  Created on: Nov 6, 2020
 *      Author: meijian
 */

#ifndef MAP_JAR_H_
#define MAP_JAR_H_


#include <opencv2/core/base.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include "map_util.h"
#include "angle_util.h"
//#include "../motion/reeds_shepp.h"
#include <math.h>

struct Jar
{
	std::vector<LaserPoint> laser_points_;
	std::vector<PointF> planar_points_;
	PointF cen_;   //center point
	PointF mid_;	// middle point
	float diam_;  // diameter
};


struct JarLine
{
	std::vector<Jar> jars_; 	// 2 jars
	Pose LiftPose();
};

struct Pillar
{
	std::vector<LaserPoint> laser_points_;
	std::vector<PointF> planar_points_;
	PointF cen_;   //center point
	PointF mid_;	// middle point
};
struct PillarLine
{
	std::vector<Pillar> pollar_lines_;
	void ShelfPose(std::vector<Pose> &lps);
};
// blog.csdn.net/liyuanbhu/article/details/50889951
bool CircleRegression(std::vector<PointF> seg, PointF& center, float& radius, float& error);

bool IsJar(Jar &jar);

std::vector<Pose>  DetectJar(Pose carpose,std::vector<LaserPoint> polarLaserPoints);
std::vector<PillarLine> DetectPillar(Pose carpose,std::vector<LaserPoint> polarLaserPoints);
Pose AveragePose(std::vector<Pose> poses);
#define WORLD_CENTER_X 0.f
#define WORLD_CENTER_Y 0.f
#define MAP_SIZE_X 1000
#define MAP_SIZE_Y 1000
#define MAP_SIZE_X2 500
#define MAP_SIZE_Y2 500
#define DELTA 0.01


PointI LWorld2Map(const PointF p);

void DrawPose(cv::Mat &mapImage, Pose pose, cv::Scalar color);

//void PlotCmdSeq(cv::Mat &image, CmdSeq cmd, Pose carPose);

void DrawLaserMapAndJars(const char* winName, std::vector<LaserPoint> lps, Pose carPose, Pose liftPose/*, CmdSeq cmdSeq*/);
void DrawLaserMapAndPillar(const char* winName, std::vector<LaserPoint> lps, Pose carPose, Pose liftPose ,Pose secondPose);
#endif /* MAP_JAR_H_ */
