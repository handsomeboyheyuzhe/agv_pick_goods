/*
 * jar.cpp
 *
 *  Created on: Dec 25, 2020
 *      Author: meijian
 */

#include "../include/jar.h"
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;
const double shelf_with = 0.92;
void PillarLine::ShelfPose(std::vector<Pose> &lps)
{
	std::vector<Pose> shelf_poses;
	for (int i = 0; i < pollar_lines_.size() - 1; i++)
	{
		PointF pollar1 = pollar_lines_[i].cen_;
		PointF pollar2 = pollar_lines_[i + 1].cen_;
		//std::cout<<"pollar1 and pollar2="<<pollar1.x<<" "<<pollar1.y<<"----- "<<pollar2.x<<" "<<pollar2.y<<std::endl;
		// double dis=sqrt(pow(pollar1.x-pollar2.x,2)+pow(pollar1.y-pollar2.y,2)) ;
		// std::cout<<"the again dist="<<dis<<std::endl;
		// if (abs(dis-shelf_with)<0.15) 	//两点是货架的两点,仅仅按照宽度来匹配
		// {

		float dxj = pollar1.x - pollar2.x;
		float dyj = pollar1.y - pollar2.y;
		float jDir = atan2(dyj, dxj);

		PointF tPos = 0.5f * (pollar1 + pollar2);
		Pose relPose;
		relPose.x = tPos.x;
		relPose.y = tPos.y;
		//std::cout<<"tar x y="<<tPos.x<<" "<<tPos.y<<std::endl;
		relPose.a = Mod2Pi(jDir - PI / 2);

		// float offset = 0.0;
		// relPose.x -= offset * cos(relPose.a);
		// relPose.y -= offset * sin(relPose.a);

		lps.push_back(relPose);
		// }
	}
}
Pose JarLine::LiftPose()
{
	PointF jar1 = jars_.back().cen_;
	PointF jar2 = jars_.front().cen_;
	float dxj = jar1.x - jar2.x;
	float dyj = jar1.y - jar2.y;
	float jDir = atan2(dyj, dxj);

	PointF tPos = 0.5f * (jar1 + jar2);
	Pose relPose;
	relPose.x = tPos.x;
	relPose.y = tPos.y;

	relPose.a = Mod2Pi(jDir - PI / 2);

	float offset = 0.0;
	relPose.x -= offset * cos(relPose.a);
	relPose.y -= offset * sin(relPose.a);

	return relPose;
}
// blog.csdn.net/liyuanbhu/article/details/50889951
bool CircleRegression(std::vector<PointF> seg, PointF &center, float &radius, float &error)
{
	uint sz = seg.size();
	if (sz == 0)
		return false;

	float xM = 0.f;
	float yM = 0.f;

	for (uint i = 0; i < sz; i++)
	{
		xM += seg[i].x;
		yM += seg[i].y;
	}
	xM = xM / sz;
	yM = yM / sz;

	float uuu = 0.f, vvv = 0.f, uu = 0.f, vv = 0.f, uv = 0.f, uuv = 0.f, uvv = 0.f;
	for (uint i = 0; i < sz; i++)
	{
		float u = seg[i].x - xM;
		float v = seg[i].y - yM;
		uuu += (u * u * u);
		vvv += (v * v * v);
		uu += (u * u);
		vv += (v * v);
		uv += (u * v);
		uuv += (u * u * v);
		uvv += (u * v * v);
	}
	float uc = (uuv * uv - uuu * vv - uvv * vv + uv * vvv) / (uv * uv - uu * vv) / 2;
	float vc = (-uu * uuv + uuu * uv + uv * uvv - uu * vvv) / (uv * uv - uu * vv) / 2;
	center.x = uc + xM;
	center.y = vc + yM;

	radius = 0.f;
	for (uint i = 0; i < sz; i++)
	{
		float dx = seg[i].x - center.x;
		float dy = seg[i].y - center.y;
		radius += (dx * dx + dy * dy);
	}
	radius = std::sqrt(radius / sz);

	error = 0.f;
	for (uint i = 0; i < sz; i++)
	{
		float dx = seg[i].x - center.x;
		float dy = seg[i].y - center.y;
		float realR = std::sqrt(dx * dx + dy * dy);
		error += std::fabs(realR - radius);
	}
	error = error / sz;
	return true;
}

bool IsPillar(Jar &pillar) //并且宽度不超过一定距离
{
	PointF head = pillar.planar_points_.front();
	PointF tail = pillar.planar_points_.back();
	double distance = sqrt(pow(head.x - tail.x, 2) + pow(head.y - tail.y, 2));
	pillar.cen_ = 0.5f * PointF(head + tail);
	//pillar.cen_=pillar.planar_points_[pillar.planar_points_.size()/2];
	pillar.mid_ = pillar.planar_points_[pillar.planar_points_.size() / 2];
	float tocar = sqrt(pow(pillar.cen_.x, 2) + pow(pillar.cen_.y, 2));
	// std::cout<<"the pillar with="<<distance<<std::endl;
	if (distance > 0.02 && distance < 0.15 /*&&tocar>0.6*/)
		return true;
	return false;
}
bool isPareller(cv::Vec4f A,cv::Vec4f B)
{
	float angela = atan2(A[2] - A[0], A[3] - A[1]) * 180 / M_PI;
	float angelb = atan2(B[2] - B[0], B[3] - B[1]) * 180 / M_PI;
	if(abs(angela-angelb)<10||abs(abs(angela-angelb)-180)<10)
		return true;
	return false;
}
Pose AveragePose(std::vector<Pose> poses)
{	Pose pose(0.0,0.0,0.0);
	for(int i=0;i<poses.size();i++)
	{
		pose.x += poses[i].x;
		pose.y += poses[i].y;
		pose.a += poses[i].a;
	}
	pose.x=pose.x/poses.size();
	pose.y=pose.y/poses.size();
	pose.a=pose.a/poses.size();
	return pose;
}
bool classification(vector<cv::Vec4f> plines, vector<cv::Vec4f> &lines)
{
	int n = plines.size();
	// if(plines.size()>=1)
	// 	n=1;
	// std::cout<<" plines size="<<plines.size()<<std::endl;
	// for (int i = 0; i < plines.size(); i++)
	// {
	// 	cv::Vec4f hline = plines[i];
	// 	float angel = atan2(hline[2] - hline[0], hline[3] - hline[1]) * 180 / M_PI;
	// 	std::cout << "thepline" << i << "==" << hline[0] << " " << hline[1] << "------" << hline[2] << " " << hline[3] << " angel==" << angel << std::endl;
	// }
	
	for (int i = 0; i < plines.size() - 1; i++)
	{
		lines.push_back(plines[i]);
		for (int j = i + 1; j < plines.size(); j++)
		{
			cv::Vec4f hline = plines[i];
			cv::Vec4f hline2 = plines[j];
			PointF cent((hline[2] + hline[0]) / 2, (hline[1] + hline[3]) / 2);
			PointF cent2((hline2[2] + hline2[0]) / 2, (hline2[1] + hline2[3]) / 2);
			//std::cout<<"disstance="<<Distance(cent, cent2)<<std::endl;
			if (Distance(cent, cent2) < 30.0&&isPareller(hline,hline2))
			{
				n--;
				cv::Vec4f midline = 0.5f * (hline + hline2); // average the x,y
				plines[j] = midline;
				lines.pop_back();
				break;
			}
		}
	}
	lines.push_back(plines[plines.size() - 1]);
	for (int i = 0; i < lines.size(); i++) //problem
	{
		cv::Vec4f hline = lines[i];
		float angel = atan2(hline[2] - hline[0], hline[3] - hline[1]) * 180 / M_PI;
		int k = 0;
		for (int j = 0; j< lines.size(); j++)
		{
			cv::Vec4f hline2 = lines[j];
			float angel2 = atan2(hline2[2] - hline2[0], hline2[3] - hline2[1]) * 180 / M_PI;
			if (abs(abs(angel - angel2)-90)< 10)
			{
				k++;
			}
		}
		if (k == 2)
		{
			cv::Vec4f swich = lines[0];
			lines[0] = hline;
			lines[i] = swich;
			// for (int i = 0; i < lines.size(); i++)
			// {
			// 	cv::Vec4f hline = lines[i];
			// 	float angel = atan2(hline[2] - hline[0], hline[3] - hline[1]) * 180 / M_PI;
			// 	std::cout << "theline" << i << "==" << hline[0] << " " << hline[1] << "------" << hline[2] << " " << hline[3] << " angel==" << angel << std::endl;
			// }
			return true;
		}
	}
	
	return false;
}
void ModPi(float &a)
{
	float b = a;
	while (b < -M_PI)
		b += 2 * M_PI;
	while (b >= M_PI)
		b -= 2 * M_PI;
	a = b;
}
void Turn180(float &a)
{
	if (a > 80)
	{
		a += 180;
		a = a / 180 * M_PI;
		ModPi(a);
	}
	a=a*180/M_PI;
}
bool IsRectangle(Jar &pillar,Pose &lp) //并且宽度不超过一定距离
{
	PointF head = pillar.planar_points_.front();
	PointF tail = pillar.planar_points_.back();
	cv::Mat mapImage(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Point2f offset(mapImage.cols / 2, mapImage.rows / 2);
	std::vector<cv::Point2f> points;
	for (std::vector<PointF>::iterator it = pillar.planar_points_.begin(); it != pillar.planar_points_.end(); it++)
	{
		PointF fLp = PointF(it->x * 100, it->y * 100) + PointF(500, 500);
		//PointI iLp = LWorld2Map(fLp);
		points.push_back(cv::Point2f(fLp.x,fLp.y));
		cv::circle(mapImage, cv::Point(fLp.y, fLp.x), 1, cv::Scalar(200, 200, 200), 1, 8);
	}
	cv::Mat src_gray;
	Canny(mapImage, src_gray, 150, 200);						   //提取边缘 ,输入图像可以是RGB每个通道占8位就行，输出8位灰度图像
	vector<cv::Vec4f> plines;									   //定义一个浮点数，二维数组
	HoughLinesP(src_gray, plines, 3.0, CV_PI / 180.0, 20, 50, 10); //概率霍夫线变换直线检测，灰度输入，像素步长，角度步长，交点阈值，最短长度，最大间隔（经canny的梯度可能不连续，间隔调大）
	//julei
	//
	if (plines.size() >= 3)
	{
		vector<cv::Vec4f> lines;
		bool sort = classification(plines, lines);
		if (lines.size() == 3&&sort)
		{
			for (size_t i = 0; i < lines.size(); i++)
			{
				cv::Vec4f hline = lines[i];
				float angel = atan2(hline[2] - hline[0], hline[3] - hline[1]) * 180 / M_PI;										//数组获取直线
				line(mapImage, cv::Point2f(hline[0], hline[1]), cv::Point2f(hline[2], hline[3]), cv::Scalar(255, 0, 0), 2, 16); //画直线，LINE_AA反锯齿
			}
			Pose target;
			cv::Vec4f line = lines[0]; //computer best angel
			cv::Vec4f line2 = lines[1];
			cv::Vec4f line3 = lines[2];
			float tara = atan2(line[2] - line[0], line[3] - line[1]) * 180 / M_PI;
			float tarb = atan2(line2[2] - line2[0], line2[3] - line2[1]) * 180 / M_PI;
			float tarc = atan2(line3[2] - line3[0], line3[3] - line3[1]) * 180 / M_PI;
			if (tarc > 80)
			{
				Turn180(tarc);
			}
			 if (tarb > 80)
			{
				Turn180(tarb);
			}
			float condition1 =abs(tara-tarb)- 90;
			float condition2 = tarb - tarc;
			float condition3 =abs(tara-tarc)- 90;
			
			std::cout<<"the condition="<<abs(condition1)<<" "<<abs(condition2)<<" "<<abs(condition3)<<std::endl;
			if (abs(condition1) < 10 && abs(condition2)<10&&abs(condition3) < 10)
			{
				RotatedRect box = minAreaRect(Mat(points)); //点集的最小外接旋转矩形
				Point2f tr[4];
				box.points(tr);
				circle(mapImage, Point(box.center.y,box.center.x), 5, Scalar(0, 255, 0), -1, 8); //绘制最小外接矩形的中心点

				for (int i = 0; i < 4; i++)
				{
					cv::line(mapImage,  cv::Point(tr[i].y, tr[i].x),cv::Point(tr[(i + 1) % 4].y, tr[(i + 1) % 4].x), Scalar(0, 255, 255), 1, CV_AA);
				}
				tara -= 90;
				target.a = (tarb + tarc) / 2.0 / 180 * M_PI;
				target.y = box.center.y;
				target.x = box.center.x;
				PointF fDir;
				fDir.x = 50 * cos(target.a) + target.x;
				fDir.y = 50 * sin(target.a) + target.y;
				
				//PointI iDir = LWorld2Map(fDir);
				cv::circle(mapImage, cv::Point(tr[2].y, tr[2].x), 3, cv::Scalar(255, 255, 255), 2, 8);
				cv::circle(mapImage, cv::Point(tr[3].y, tr[3].x), 3, cv::Scalar(255, 255, 255), 2, 8);
				cv::circle(mapImage, cv::Point(tr[0].y, tr[0].x), 3, cv::Scalar(255, 255, 255), 2, 8);
				cv::circle(mapImage, cv::Point(tr[1].y, tr[1].x), 3, cv::Scalar(255, 255, 255), 2, 8);
				float angle1,angle2,angle3,angle4,angle;
				angle1=atan2(tr[0].y - tr[1].y,tr[0].x-tr[1].x)*180/M_PI ;
				angle2=atan2(tr[1].y - tr[2].y,tr[1].x-tr[2].x)*180/M_PI ;
				angle3=atan2(tr[2].y - tr[3].y,tr[2].x-tr[3].x)*180/M_PI ;
				angle4=atan2(tr[3].y - tr[0].y,tr[3].x-tr[0].x)*180/M_PI ;
				std::cout<<"4 angel="<<angle1<<" "<<angle2<<" "<<angle3<<" "<<angle4<<std::endl;
				if(angle1<45)
				{
					angle=((angle2-90)+(angle4+90))/2.0;
				}
				else
				{
					angle=((angle1-90)+(angle3+90))/2.0;
				}
				
				Pose target2;
				target2.a = angle/180*M_PI;
				target2.y = box.center.y;
				target2.x = box.center.x;
				PointF fDir2;
				fDir2.x = 50 * cos(target2.a) + target2.x;
				fDir2.y = 50 * sin(target2.a) + target2.y;
				// cv::circle(mapImage, cv::Point(target.y, target.x), 3, cv::Scalar(255, 255, 0), 2, 8);
				// cv::line(mapImage, cv::Point(target.y, target.x), cv::Point(fDir.y, fDir.x), cv::Scalar(255, 255, 0), 2, 8);
				cv::circle(mapImage, cv::Point(target2.y, target2.x), 3, cv::Scalar(255, 255, 0), 2, 8);
				cv::line(mapImage, cv::Point(target2.y, target2.x), cv::Point(fDir2.y, fDir2.x), cv::Scalar(255, 255, 255), 2, 8);
				cv::imshow("huofu", mapImage);
				cv::waitKey(10);
				lp=target2;
				lp.x=(lp.x-500)/100.0;
				lp.y=(lp.y-500)/100.0;
				return true;
			}
		}
	}
	return false;
}
bool IsJar(Jar &jar)
{
	std::vector<PointF> seg = jar.planar_points_;
	jar.diam_ = PointDist(seg.front(), seg.back());
	if (jar.diam_ > 0.8 || jar.diam_ < 0.15 || seg.size() <= 3)
		return false;

	jar.cen_ = seg.front() + seg.back();
	jar.cen_ = 0.5f * jar.cen_;
	jar.mid_ = seg[seg.size() / 2];
	float distCen = sqrtf(jar.cen_ * jar.cen_);
	float distMid = sqrtf(jar.mid_ * jar.mid_);

	if (distCen < distMid)
		return false;

	PointF center;
	float radius, error;
	CircleRegression(seg, center, radius, error);
	jar.cen_ = center;
	distCen = sqrtf(jar.cen_ * jar.cen_);
	if (distCen < distMid || radius < 0.1 /*|| radius > 0.15*/ || error > 0.01)
		return false;
	/*
	std::cout << "DDDDDDDDDDddist [" << lidarPose.x << " " << lidarPose.y << "] ["
			<< jar.cen_.x << " " << jar.cen_.y << " " << distCen << "] ["
			<< jar.mid_.x << " " << jar.mid_.y << " " <<  distMid << "] [ "
			<< radius << " " << error << " " << seg.size() << "] \n";
*/
	return true;
}

std::vector<PillarLine> DetectPillar(Pose lidarPose, std::vector<LaserPoint> polarLaserPoints)
{
	std::vector<Pillar> pillars;

	bool firstRound = true;
	std::vector<PointF> seg;
	std::vector<LaserPoint> lps;
	for (std::vector<LaserPoint>::iterator it = polarLaserPoints.begin(); it != polarLaserPoints.end(); it++)
	{
		LaserPoint lp = *it;
		if (lp.range <= 0)
			continue;

		PointF lpPlanar;
		LaserHitPoint(lidarPose, lp, lpPlanar); //转换成坐标 seq 坐标集合  lps雷达原始数据集合
		if (firstRound)
		{
			firstRound = false;
			seg.push_back(lpPlanar);
			lps.push_back(lp);
		}
		else
		{
			PointF prevP = seg.back();
			float dist = PointDist(prevP, lpPlanar);
			if (dist < 0.1)
			{
				seg.push_back(lpPlanar);
				lps.push_back(lp);
			}
			else
			{
				Pillar pillar;
				pillar.planar_points_.swap(seg);
				pillar.laser_points_.swap(lps);
				seg.push_back(lpPlanar);
				lps.push_back(lp);
				//std::cout<<"new part=";
				// for (std::vector<LaserPoint>::iterator pit = pillar.laser_points_.begin(); pit != pillar.laser_points_.end(); pit++)
				// {
				// 	std::cout << "[" << pit->angle * 180 / M_PI << ", " << pit->range << "]\t";
				// }
				// std::cout<<std::endl;
				// for(std::vector<PointF>::iterator pit = pillar.planar_points_.begin(); pit != pillar.planar_points_.end(); pit++)
				// 	{
				// 		std::cout << "[" << pit->x << ", " << pit->y << "]\t";
				// 	}
				// std::cout<<std::endl;
				// if( IsPillar( pillar ))
				// {
				// 	float distance = sqrt(pow(pillar.cen_.x , 2) + pow(pillar.cen_.y, 2));
				// 	if(distance<3.0)
				// 	{
				// 			pillars.push_back(pillar);
				// 	// for(std::vector<PointF>::iterator pit = pillar.planar_points_.begin(); pit != pillar.planar_points_.end(); pit++)
				// 	// {
				// 	// 	std::cout << "[" << pit->x << ", " << pit->y << "]\t";
				// 	// }
				// 	// std::cout << "\nTotal " << pillar.planar_points_.size()<< "\n";
				// 	}
				// }
			}
		}
	}

	std::vector<PillarLine> jarLines;
	if (pillars.size() > 1)
	{
		for (uint i = 0; i < pillars.size() - 1; i++)
		{
			Pillar jar1 = pillars[i];
			for (uint j = i + 1; j < pillars.size(); j++)
			{
				Pillar jar2 = pillars[j];
				float dist = PointDist(jar1.cen_, jar2.cen_);
				if (dist > 0.88 && dist < 1.12) //两个柱子间的距离，1m
				{
					PillarLine jl;
					jl.pollar_lines_.push_back(jar1);
					jl.pollar_lines_.push_back(jar2);
					jarLines.push_back(jl);
					std::cout << " ~~~~~~~ Jars: [" << jar1.cen_.x << ", " << jar1.cen_.y << "] ["
							  << jar2.cen_.x << ", " << jar2.cen_.y << "]\n";
				}
			}
		}
	}
	return jarLines;
}
std::vector<Pose> DetectJar(Pose lidarPose, std::vector<LaserPoint> polarLaserPoints)
{
	std::vector<Pose> jars;
	
	bool firstRound = true;
	std::vector<PointF> seg;
	std::vector<LaserPoint> lps;
	for (std::vector<LaserPoint>::iterator it = polarLaserPoints.begin(); it != polarLaserPoints.end(); it++)
	{
		LaserPoint lp = *it;
		if (lp.range <= 0)
			continue;

		PointF lpPlanar;
		LaserHitPoint(lidarPose, lp, lpPlanar); //转换成坐标 seq 坐标集合  lps雷达原始数据集合
		if (firstRound)
		{
			firstRound = false;
			seg.push_back(lpPlanar);
			lps.push_back(lp);
		}
		else
		{
			PointF prevP = seg.back();
			float dist = PointDist(prevP, lpPlanar);
			if (dist < 0.1)
			{
				seg.push_back(lpPlanar);
				lps.push_back(lp);
			}
			else
			{
				Jar jar;
				jar.planar_points_.swap(seg);
				jar.laser_points_.swap(lps);
				seg.push_back(lpPlanar);
				lps.push_back(lp);
				Pose lp;
				if (IsRectangle(jar,lp))
				{
					jars.push_back(lp);
				}
			}
		}
	}
	return jars;
}

PointI LWorld2Map(const PointF p)
{
	int mapX = (int)round((p.x - WORLD_CENTER_X) / DELTA) + MAP_SIZE_X2;
	int mapY = (int)round((p.y - WORLD_CENTER_Y) / DELTA) + MAP_SIZE_Y2;
	return PointI(mapX, mapY);
}

void DrawPose(cv::Mat &mapImage, Pose pose, cv::Scalar color)
{
	PointI pos = LWorld2Map(pose);
	PointF fDir;
	fDir.x = .5 * cos(pose.a) + pose.x;
	fDir.y = .5 * sin(pose.a) + pose.y;
	PointI iDir = LWorld2Map(fDir);
	cv::circle(mapImage, cv::Point(pos.y, pos.x), 3, color, 2, 8);
	cv::line(mapImage, cv::Point(pos.y, pos.x), cv::Point(iDir.y, iDir.x), color, 2, 8);
}

// void PlotCmdSeq(cv::Mat &image, CmdSeq cmd, Pose carPose)
// {
// 	float sx = carPose.x;
// 	float sy = carPose.y;
// 	float orient = carPose.a;

// 	float values[3] = {cmd.t_, cmd.u_, cmd.v_};
// 	for( int i = 0; i < 3; i++ )
// 	{
// 		CmdType ct = cmd.type_seq_[i];
// 		float vl = values[i];
// 		if( ct == Sp || ct == Sm )
// 		{
// 			float dx = vl * cos(orient) * g_turn_radius;
// 			float dy = vl * sin(orient) * g_turn_radius;
// 			float ex = sx + dx;
// 			float ey = sy + dy;
// 			PointI sI = LWorld2Map( PointF( sx, sy ) );
// 			PointI eI= LWorld2Map( PointF( ex, ey ) );
// 			cv::Point start(sI.y, sI.x);
// 			cv::Point end(eI.y, eI.x);
// 			cv::line(image, start, end, cv::Scalar(100, 150, 255), 1, 8);
// 			sx = ex;
// 			sy = ey;
// 		}
// 		else if(ct == Lp || ct == Lm)
// 		{
// 			float co = orient + PI/2;
// 			float cx = sx + cos(co) * g_turn_radius;
// 			float cy = sy + sin(co) * g_turn_radius;

// 			PointI cI = LWorld2Map( PointF( cx, cy ) );
// 			cv::Point center(cI.y, cI.x);
// 			cv::Size axes(g_turn_radius / DELTA, g_turn_radius / DELTA);
// 			double angle = 0;
// 			double startAngle = - orient * 180 / PI + 180;
// 			orient += vl;
// 			double endAngle = - orient * 180 / PI + 180;
// 			sx = cx + sin(orient) * g_turn_radius;
// 			sy = cy - cos(orient) * g_turn_radius;

// //			std::cout << "LLLLLLLLLLLLLL    " << center << " " << startAngle << " " << endAngle << "\n";
// 			cv::ellipse(image, center, axes, angle,	startAngle, endAngle, cv::Scalar(0, 255, 0) );
// 		}
// 		else if(ct == Rp || ct == Rm)
// 		{
// 			float co = orient - PI/2;
// 			float cx = sx + cos(co) * g_turn_radius;
// 			float cy = sy + sin(co) * g_turn_radius;

// 			PointI cI = LWorld2Map( PointF( cx, cy ) );
// 			cv::Point center(cI.y, cI.x);
// 			cv::Size axes(g_turn_radius / DELTA, g_turn_radius / DELTA);
// 			double angle = 0;
// 			double startAngle = - orient * 180 / PI;
// 			orient -= vl;
// 			double endAngle = - orient * 180 / PI;
// 			sx = cx - sin(orient) * g_turn_radius;
// 			sy = cy + cos(orient) * g_turn_radius;

// //			std::cout << "RRRRRRRRRRRRRRR     " << center << " " << startAngle << " " << endAngle << " " << vl << "\n";
// 			cv::ellipse(image, center, axes, angle,	startAngle, endAngle, cv::Scalar(0, 0, 255) );
// 		}
// 	}
// }

void DrawLaserMapAndPillar(const char *winName, std::vector<LaserPoint> lps, Pose carPose ,Pose liftPose,Pose secondPose )
{
	cv::Mat mapImage(MAP_SIZE_X, MAP_SIZE_Y, CV_8UC3);
	for (uint x = 0; x < MAP_SIZE_X; x++)
	{
		for (uint y = 0; y < MAP_SIZE_Y; y++)
		{
			mapImage.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 0, 0);
		}
	}

	PointF fOrig(0, 0), fXAxis(1, 0), fYAxis(0, 1);
	PointI iOrig = LWorld2Map(fOrig);
	PointI iXAxis = LWorld2Map(fXAxis);
	PointI iYAxis = LWorld2Map(fYAxis);
	cv::line(mapImage, cv::Point(iOrig.y, iOrig.x), cv::Point(iXAxis.y, iXAxis.x), cv::Scalar(255, 255, 255), 1, 8);
	cv::line(mapImage, cv::Point(iOrig.y, iOrig.x), cv::Point(iYAxis.y, iYAxis.x), cv::Scalar(255, 255, 255), 1, 8);

	for (std::vector<LaserPoint>::iterator it = lps.begin(); it != lps.end(); it++)
	{
		PointF fLp;
		//float laserAngle = it->angle+PI/2+carPose.a;
		float laserAngle = it->angle + carPose.a;
		fLp.x = it->range * cos(laserAngle)+0.25 + carPose.x;// 0.25 is radar to car
		fLp.y = it->range * sin(laserAngle) + carPose.y;
		PointI iLp = LWorld2Map(fLp);
		cv::circle(mapImage, cv::Point(iLp.y, iLp.x), 1, cv::Scalar(200, 200, 200), 1, 8);
		//cv::circle(mapImage, cv::Point(iLp.x, iLp.y), 1, cv::Scalar(200, 200, 200), 1, 8);

		//		mapImage.at<cv::Vec3b>(iLp.x, iLp.y) = cv::Vec3b(200, 200, 200);
	}
	DrawPose(mapImage, carPose, cv::Scalar(255, 255, 0));
	DrawPose(mapImage, liftPose, cv::Scalar(0, 255, 255));
	DrawPose(mapImage, secondPose, cv::Scalar(255, 255, 255));

	// for(int i=0;i<dests.size();i++)
	// {
	// 	DrawPose(mapImage, dests[i], cv::Scalar(0, 255, 255));
	// }

	//PlotCmdSeq(mapImage, cmdSeq, carPose);

	cv::imshow(winName, mapImage);
	cv::waitKey(10);
}
void DrawLaserMapAndJars(const char *winName, std::vector<LaserPoint> lps, Pose carPose, Pose liftPose)
{
	cv::Mat mapImage(MAP_SIZE_X, MAP_SIZE_Y, CV_8UC3);
	for (uint x = 0; x < MAP_SIZE_X; x++)
	{
		for (uint y = 0; y < MAP_SIZE_Y; y++)
		{
			mapImage.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 0, 0);
		}
	}

	PointF fOrig(0, 0), fXAxis(1, 0), fYAxis(0, 1);
	PointI iOrig = LWorld2Map(fOrig);
	PointI iXAxis = LWorld2Map(fXAxis);
	PointI iYAxis = LWorld2Map(fYAxis);
	cv::line(mapImage, cv::Point(iOrig.y, iOrig.x), cv::Point(iXAxis.y, iXAxis.x), cv::Scalar(255, 255, 255), 1, 8);
	cv::line(mapImage, cv::Point(iOrig.y, iOrig.x), cv::Point(iYAxis.y, iYAxis.x), cv::Scalar(255, 255, 255), 1, 8);

	for (std::vector<LaserPoint>::iterator it = lps.begin(); it != lps.end(); it++)
	{
		PointF fLp;
		//float laserAngle = it->angle+PI/2+carPose.a;
		float laserAngle = it->angle + carPose.a;
		fLp.x = it->range * cos(laserAngle) + carPose.x;
		fLp.y = it->range * sin(laserAngle) + carPose.y;
		PointI iLp = LWorld2Map(fLp);
		cv::circle(mapImage, cv::Point(iLp.y, iLp.x), 1, cv::Scalar(200, 200, 200), 1, 8);
		//cv::circle(mapImage, cv::Point(iLp.x, iLp.y), 1, cv::Scalar(200, 200, 200), 1, 8);

		//		mapImage.at<cv::Vec3b>(iLp.x, iLp.y) = cv::Vec3b(200, 200, 200);
	}

	DrawPose(mapImage, carPose, cv::Scalar(255, 255, 0));
	DrawPose(mapImage, liftPose, cv::Scalar(0, 255, 255));

	// for (std::vector<JarLine>::iterator jlit = jarLines.begin(); jlit != jarLines.end(); jlit++)
	// {
	// 	std::vector<Jar> jars = jlit->jars_;
	// 	for (std::vector<Jar>::iterator jit = jars.begin(); jit != jars.end(); jit++)
	// 	{
	// 		PointF worldCent;
	// 		float angle = carPose.a;
	// 		worldCent.x = jit->cen_.x * cos(angle) - jit->cen_.y * sin(angle) + carPose.x;
	// 		worldCent.y = jit->cen_.x * sin(angle) + jit->cen_.y * cos(angle) + carPose.y;
	// 		PointI cen = LWorld2Map(worldCent);
	// 		cv::circle(mapImage, cv::Point(cen.y, cen.x), 4, cv::Scalar(0, 255, 255), 0.5, 8);
	// 	}
	// }

	//PlotCmdSeq(mapImage, cmdSeq, carPose);

	cv::imshow(winName, mapImage);
	cv::waitKey(10);
}
