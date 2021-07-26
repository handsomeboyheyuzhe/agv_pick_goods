 
// #include "ros/ros.h"
// #include "service_shelf/shelf_statu.h"

// bool add_execute(service_shelf::shelf_statu::Request &req,
// service_shelf::shelf_statu::Response &res)
// {
//   res.sum = req.a + req.b;
//   ROS_INFO("recieve request: a=%ld,b=%ld",(long int)req.a,(long int)req.b);
//   ROS_INFO("send response: sum=%ld",(long int)res.sum);
//   return true;
// } 

// int main(int argc,char **argv)
// {
//   ros::init(argc,argv,"server_node");
//   ros::NodeHandle nh;

//   ros::ServiceServer service = nh.advertiseService("add_two_ints",add_execute);
//   ROS_INFO("service is ready!!!");
//   ros::spin();

//   return 0;
// }
#include <service_shelf/DoDishesAction.h> 
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "tf/transform_datatypes.h"
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int16.h"
//Twist.h是机器运动速度消息包的格式定义文件
ros::Publisher vel_pub;
ros::Publisher vel_int16;
typedef actionlib::SimpleActionServer<service_shelf::DoDishesAction> Server;
Server *server_nh;
static int Task=0,moveStage=3;
static double car_x,car_y,car_a;
static float cmd_speed_=0,cmd_steer_=0,cmd_lift_=0,lift_speed_=0,befor_angeldiff_lp_cp_=0;
const double shelf_with=0.92;
#define WORLD_CENTER_X 0.f
#define WORLD_CENTER_Y 0.f
#define MAP_SIZE_X 1000
#define MAP_SIZE_Y 1000
#define MAP_SIZE_X2 500
#define MAP_SIZE_Y2 500
#define DELTA 0.01

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
struct Pillar
{
	std::vector<LaserPoint> laser_points_;
	std::vector<cv::Point2f> planar_points_;
  
	cv::Point2f cen_;   //center point
	cv::Point2f mid_;	// middle point
};
struct Pose 
{
  float x;
  float y;
  float a;

  Pose()
  {
    x=0;
    y=0;
    a=0;
  }
  Pose(double a,double b,double c)
  {
    x=(float)a;
    y=(float)b;
    a=(float)c;
  }
}befor_lp_,destpose_;
struct PillarLine
{
	std::vector<Pillar> pollar_lines_;
	void ShelfPose(std::vector<Pose> &lps);
};
std::vector<LaserPoint> polar_laser_points_;

inline bool LaserHitPoint(Pose lidarPose, LaserPoint laserPoint, cv::Point2f& pHit)
{
	if( laserPoint.range > 0 )
	{
		//float laserAngle = laserPoint.angle + PI/2;
		float laserAngle = laserPoint.angle ;
		pHit = /*lidarPose + */laserPoint.range * cv::Point2f(cos(laserAngle), sin(laserAngle));
		return true;
	}
	return false;
}

inline float PointDist(cv::Point2f p1, cv::Point2f p2)
{
  float dx=p1.x-p2.x;
  float dy=p1.y-p2.y;
	return sqrt( dx*dx+dy*dy );
}

inline cv::Point2i LWorld2Map(const cv::Point2f p)
{
	int mapX = (int)round( (p.x - WORLD_CENTER_X) / DELTA ) + MAP_SIZE_X2;
	int mapY = (int)round( (p.y - WORLD_CENTER_Y) / DELTA ) + MAP_SIZE_Y2;
	return cv::Point2i( mapX, mapY );

}

inline void DrawPose(cv::Mat &mapImage, Pose pose, cv::Scalar color)
{
	cv::Point2i pos = LWorld2Map(cv::Point2f(pose.x,pose.y));
	cv::Point2f fDir;
	fDir.x = .5 * cos(pose.a) + pose.x;
	fDir.y = .5 * sin(pose.a) + pose.y;
	cv::Point2i iDir = LWorld2Map(fDir);
	cv::circle(mapImage, cv::Point(pos.y, pos.x), 3, color, 2, 8);
	cv::line(mapImage,cv::Point(pos.y, pos.x), cv::Point(iDir.y, iDir.x) ,color,2,8);
}

inline void DrawLaserMapAndPillar(const char* winName, std::vector<LaserPoint> lps, Pose carPose,  std::vector<Pose> dests, Pose liftPose /*CmdSeq cmdSeq*/)
{
	cv::Mat mapImage(MAP_SIZE_X, MAP_SIZE_Y, CV_8UC3);
	for( uint x = 0; x < MAP_SIZE_X; x++)
	{
		for( uint y = 0; y < MAP_SIZE_Y; y++ )
		{
			mapImage.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 0, 0);
		}
	}

	cv::Point2f fOrig(0,0), fXAxis(1,0), fYAxis(0,1);
	cv::Point2i iOrig = LWorld2Map(fOrig);
	cv::Point2i iXAxis = LWorld2Map(fXAxis);
	cv::Point2i iYAxis = LWorld2Map(fYAxis);
	cv::line(mapImage, cv::Point(iOrig.y, iOrig.x), cv::Point(iXAxis.y, iXAxis.x), cv::Scalar(255, 255, 255), 1, 8);
	cv::line(mapImage, cv::Point(iOrig.y, iOrig.x), cv::Point(iYAxis.y, iYAxis.x), cv::Scalar(255, 255, 255), 1, 8);

	for(std::vector<LaserPoint>::iterator it = lps.begin(); it != lps.end(); it++)
	{
		cv::Point2f fLp;
		//float laserAngle = it->angle+PI/2+carPose.a;
		float laserAngle = it->angle+carPose.a;
		fLp.x = it->range * cos(laserAngle) + carPose.x;
		fLp.y = it->range * sin(laserAngle) + carPose.y;
		cv::Point2i iLp = LWorld2Map(fLp);
		cv::circle(mapImage, cv::Point(iLp.y, iLp.x), 1, cv::Scalar(200, 200, 200), 1, 8);
		//cv::circle(mapImage, cv::Point(iLp.x, iLp.y), 1, cv::Scalar(200, 200, 200), 1, 8);

//		mapImage.at<cv::Vec3b>(iLp.x, iLp.y) = cv::Vec3b(200, 200, 200);
	}
	DrawPose(mapImage, carPose, cv::Scalar(255, 255, 0));
	DrawPose(mapImage, liftPose, cv::Scalar(0, 255, 255));

	// for(int i=0;i<dests.size();i++)
	// {
	// 	DrawPose(mapImage, dests[i], cv::Scalar(0, 255, 255));
	// }

	//PlotCmdSeq(mapImage, cmdSeq, carPose);

	cv::imshow(winName, mapImage);
	cv::waitKey(10);
}
inline void ToPi(float &A)
{
    if (A > M_PI)
        A -= 2 * M_PI;
    if (A < -M_PI)
        A += 2 * M_PI;
}
inline void To2Pi(float &A)
{
   float b = A;
	while( b < 0 )
		b += 2*M_PI;
	while( b >= 2*M_PI )
		b -= 2*M_PI;
	A=b;
}
inline bool IsPillar(Pillar & pillar) //并且宽度不超过一定距离
{
	cv::Point2f head=pillar.planar_points_.front();
	cv::Point2f tail=pillar.planar_points_.back();
	double distance=sqrt(pow(head.x-tail.x,2)+pow(head.y-tail.y,2));
	 pillar.cen_=0.5f*cv::Point2f(head+tail);
	//pillar.cen_=pillar.planar_points_[pillar.planar_points_.size()/2];
	pillar.mid_=pillar.planar_points_[pillar.planar_points_.size()/2];
	float tocar=sqrt(pow(pillar.cen_.x,2)+pow(pillar.cen_.y,2));
    std::cout<<"the pillar with="<<distance<<std::endl;
	if(distance>0.03&&distance<0.15&&tocar>0.6)
		return true;
	return false;
}
inline void SetMoveCommand(float speed, float steer, double lift)
{
  //boost::shared_lock<boost::shared_mutex> guard(sendData_);
  cmd_speed_ = speed;
  cmd_steer_ = steer;
  cmd_lift_ = lift;
  //std::cout << "~~~~~~~~~~~ Move Cmd [" << speed << ", " << steer << ", " << (int)lift << "]\n";
}
inline void MotionPlan_shelf(Pose cp,Pose lp,int &moveStage,Pose &befor_lp,float diss)
{
		To2Pi(cp.a);
		To2Pi(lp.a);
		float angeldiff=(cp.a-lp.a)*180/M_PI;
		
		float Dx=lp.x-cp.x;
		float Dy=lp.y-cp.y;
		// float lp_to_cp=atan2(Dy,Dx);
		// To2Pi(lp_to_cp);
		float disstance=sqrt(Dx*Dx+Dy*Dy);
		cv::Point2f prolongPoint=0.5f*cv::Point2f(cos(lp.a),sin(lp.a))+cv::Point2f(lp.x,lp.y);//在目标方向延伸0.5m 做一个延伸点
		float firstdestA;
		
		if (atan2(Dy, Dx) > 0)
		{
			firstdestA = lp.a + M_PI / 2;
		}
		else
		{
			firstdestA = lp.a - M_PI / 2;
		}

		ToPi(firstdestA);
		//float disstance=sqrt(lp.x*lp.x+lp.y*lp.y);//到目标点的距离
		float firstAngelDiff=cp.a-firstdestA;//到目标的垂线的角度差
		ToPi(firstAngelDiff);
		ToPi(cp.a);
		ToPi(lp.a);
		float secondanglediff=(cp.a-atan2(Dy,Dx))*180/M_PI;
		// To2Pi(cp.a);
		// To2Pi(lp.a);
		float disstancepro=sqrt(pow(prolongPoint.x-cp.x,2)+pow(prolongPoint.y-cp.y,2));//到延伸目标的距离
		float angelspeed=0,movespeed=0,liftpeed=0;
		float xangeldiff=atan2(Dy,Dx)*180/M_PI;;
		// if(tar % 2 == 0)
		// {
		// 	xangeldiff+=90;//error
		// }
		// if(turn_over_<0)
		// {
		// 	xangeldiff+=90;//error
		// }
		std::cout<<"Dy Dx="<<Dx<<" "<<Dy<<std::endl;
		std::cout<<"car lp="<<cp.x<<" "<<cp.y<<" "<<cp.a*180/M_PI<<"   "<<lp.x<<" "<<lp.y<<" "<<lp.a*180/M_PI<<std::endl;
		//std::cout<<"xangeldiff dis="<<xangeldiff<<" "<<befor_anel_<<std::endl;
		std::cout<<"secondanglediff="<<secondanglediff<<std::endl;
		//std::cout<<"change_target_="<<change_target_<<" "<<disstancepro<<" "<<abs(angeldiff)<<std::endl;
		std::cout<<"movsetage="<<moveStage<<" firstdes="<<firstdestA*180/M_PI<<" car a="<<cp.a*180/M_PI<<" lp.a="<<lp.a*180/M_PI<<" firstdissangel="<<firstAngelDiff*180/M_PI<<std::endl;

		// if (moveStage == 1 && abs(xangeldiff) < 5&&return_to3_==true/*&&xangeldiff*befor_anel_<0*/)
		// {
		// 	moveStage = 3;
		// }
		// return_to3_=false;
		// if (befor_firstanglediff_*firstAngelDiff<0&& moveStage == 1&&abs(firstAngelDiff * 180 / M_PI)<5)
		// {	
		// moveStage = 2;
		// }
		// if(moveStage==2&&xangeldiff*befor_anel_<0&&abs(xangeldiff) <3)
		// {
		// 	moveStage=3;
		// }

		if(moveStage==3&&angeldiff*befor_angeldiff_lp_cp_<0&&abs(angeldiff)<3)
		{
			moveStage=4;
		}
		if(disstancepro<0.55&&moveStage==4)
		{
			moveStage=5;
		}
		if(disstancepro<0.25&&moveStage==5&&abs(angeldiff)<3)
		{
			moveStage=6;
		}	
		if(moveStage==7&&disstance>=0.4)
		{
			moveStage=8;
		}
		if(moveStage==8&&disstance>=1.28)
		{
			// if(tar%2==1)
			// {
			//  change_target_=-turn_over_+lp.a*180/M_PI;
			//  turn_over_=-turn_over_;
			// }
			// moveStage=9;
			movespeed = 0;
			liftpeed = 0;
			angelspeed=0;
      befor_angeldiff_lp_cp_=0;
      Task=0;
			//返回结束的状态
		  server_nh->setSucceeded();
		}
		// if(moveStage==9&&abs(cp.a*180/M_PI-change_target_)<3)
		// {
		// 	//change_target_=0;
		// 	tar++;
		// 	return_to3_=true;
		// 	befor_lp.a=0;
		// 	befor_anel_=0;
		// 	befor_firstanglediff_=0;
		// 	befor_angeldiff_lp_cp_=0;
		// 	moveStage=3;  //需要重置lp befor_lp 
		// }

		// if (moveStage == 1) //旋转到第一目标垂直方向
		// {
		// 	movespeed = 0;
		// 	liftpeed = 0;
		// 	float speed = 0;
		// 	float different = firstAngelDiff * 180 / M_PI;
		// 	if (abs(different) > 15)
		// 	{
		// 		speed = 0.3;
		// 	}
		// 	else if(abs(different) > 10)
		// 	{
		// 		speed = 0.15;
		// 	}
		// 	else /*if(abs(different) > 5)*/
		// 	{
		// 		speed = 0.06;
		// 	}
		// 	//speed=0.1;
		// 	if (different < 0)
		// 	{
		// 		angelspeed = speed;
		// 	}
		// 	else
		// 	{
		// 		angelspeed = -speed;
		// 	}
		// }
		// if(moveStage==2)
		// {
		// 	angelspeed=0;
		// 	movespeed=0.05;
		// 	liftpeed=0;
		// }
		//the nest stage add
		if(moveStage==3)  //origin 1 change to 3
		{
			movespeed=0;
			float speed=0;
			ToPi(cp.a);
			ToPi(lp.a);
			float angle = (cp.a - lp.a) * 180 / M_PI;

			if (abs(angle) > 15)
			{
				speed=0.3;
			}
			else if(abs(angle)>10)
			{
				speed=0.15;
			}
			else
			{
				speed=0.1;
			}
			
			//speed=0.1;
			// To2Pi(cp.a);
			// To2Pi(lp.a);
			float angel2 =cp.a*180 / M_PI- lp.a *180 / M_PI;
			if(angle<0&&abs(angel2)<180)
			{
				angelspeed=speed;
			}
			else
			{
				angelspeed=-speed;
			}
			
			 
		}
		else if(moveStage==4)//前进加微调
		{
			angelspeed=0;
			movespeed=0.25;
			float speed=0;
			//float whoda=cos(cp.a)-cos(atan2(Dy,Dx));
			if (abs(secondanglediff) > 15)
			{
				speed=0.06;
			}
			else if(abs(secondanglediff) > 10)
			{
				speed=0.1;
			}
			else if(abs(secondanglediff) > 5)
			{
				speed=0.06;
			}
			//speed=0.1;
			float X=0.01;
			float C=2;
			float A=atan2(Dy,Dx);   //x是角度调整的系数
			float B=lp.a;
			X=disstance*sin(A-B)*C;
			// if(turn_over_>0)
			// {
			// 	X=-Dy*C;
			// }
			// else
			// {
			// 	X=-Dx*C;
			// }
			std::cout<<"X==="<<X<<std::endl;
			if(secondanglediff<0)
			{
				angelspeed=speed;
			}
			else
			{
				angelspeed=-speed;
			}
			speed=speed+X;
			
		}
		else if(moveStage==5)
		{
			angelspeed=0;
			movespeed=0.1;
			if(disstancepro<0.35)
				movespeed=0.05;
			float speed=0;
			ToPi(cp.a);
			ToPi(lp.a);
			float angle = (cp.a - lp.a) * 180 / M_PI;
			if (abs(angle) > 15)
			{
				speed=0.15;
			}
			else if(abs(angle) > 10)
			{
				speed=0.12;
			}
			else if(abs(angle) > 5)
			{
				speed=0.1;
			}
			else
			{
				speed=0.05;
			}
			
			//speed=0.1;
			if(angle<0&&abs(angle)<180)
			{
				angelspeed=speed;
			}
			else
			{
				angelspeed=-speed;
			}
		}
		
		else if(moveStage==6)
		{
			angelspeed=0;
			movespeed=0;
			if(Task==1)
      {
        lift_speed_=500;
      }
      else if(Task==2)
      {
        lift_speed_=-500;
      }
      
			SetMoveCommand( movespeed, angelspeed, liftpeed );
			sleep(20);
			moveStage=7;  //error
			return;
		}
		else if(moveStage==7)
		{
			angelspeed=0.1;
			movespeed=-0.2;
			liftpeed=0.0;
			ToPi(cp.a);
			ToPi(lp.a);
			float speed=0;
			float angle = (cp.a - lp.a) * 180 / M_PI;
			
			//speed=0.1;
			if(angle<0)
			{
				angelspeed=speed;
			}
			else
			{
				angelspeed=-speed;
			}
		}
		else if(moveStage==8)
		{
			movespeed=-0.2;
			float speed=0;
			ToPi(cp.a);
			ToPi(lp.a);
			float angle = (cp.a - lp.a) * 180 / M_PI;
			if (abs(angle) > 15)
			{
				speed=0.06;
			}
			else
			{
				speed=0.06;
			}
			//speed=0.1;
			if(angle<0)
			{
				angelspeed=speed;
			}
			else
			{
				angelspeed=-speed;
			}
		}
		SetMoveCommand( movespeed, angelspeed, liftpeed );
	//	befor_firstanglediff_=firstAngelDiff; //1 t0 2
		befor_angeldiff_lp_cp_=angeldiff; //3 to 4
}
inline void PillarLine::ShelfPose(std::vector<Pose> &lps)
{
	std::vector<Pose> shelf_poses;
	for(int i=0;i<pollar_lines_.size()-1;i++)
	{
		cv::Point2f pollar1 = pollar_lines_[i].cen_;
		cv::Point2f pollar2 = pollar_lines_[i + 1].cen_;
		//std::cout<<"pollar1 and pollar2="<<pollar1.x<<" "<<pollar1.y<<"----- "<<pollar2.x<<" "<<pollar2.y<<std::endl;
		// double dis=sqrt(pow(pollar1.x-pollar2.x,2)+pow(pollar1.y-pollar2.y,2)) ;
		// std::cout<<"the again dist="<<dis<<std::endl;
		// if (abs(dis-shelf_with)<0.15) 	//两点是货架的两点,仅仅按照宽度来匹配
		// {
			
			float dxj = pollar1.x - pollar2.x;
			float dyj = pollar1.y - pollar2.y;
			float jDir = atan2(dyj, dxj);

			cv::Point2f tPos = 0.5f * (pollar1 + pollar2);
			Pose relPose;
			relPose.x = tPos.x;
			relPose.y = tPos.y;
			//std::cout<<"tar x y="<<tPos.x<<" "<<tPos.y<<std::endl;
			relPose.a =jDir-M_PI/2;
      To2Pi(relPose.a);

			// float offset = 0.0;
			// relPose.x -= offset * cos(relPose.a);
			// relPose.y -= offset * sin(relPose.a);

			lps.push_back(relPose);
		// }
	}
}
inline std::vector<PillarLine> DetectPillar(Pose lidarPose,std::vector<LaserPoint> polarLaserPoints)
{
	std::vector<Pillar> pillars;

	bool firstRound = true;
	std::vector<cv::Point2f> seg;
	std::vector<LaserPoint> lps;
	for( std::vector<LaserPoint>::iterator it = polarLaserPoints.begin(); it != polarLaserPoints.end(); it++ )
	{
		LaserPoint lp = *it;
		if(lp.range <= 0)
			continue;

		cv::Point2f lpPlanar;
		LaserHitPoint(lidarPose, lp, lpPlanar); //转换成坐标 seq 坐标集合  lps雷达原始数据集合
		if(firstRound)
		{
			firstRound = false;
			seg.push_back(lpPlanar);
			lps.push_back(lp);
		}
		else
		{
			cv::Point2f prevP = seg.back();
			float dist = PointDist(prevP, lpPlanar);
			if(dist < 0.3)
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
				// for(std::vector<cv::Point2f>::iterator pit = pillar.planar_points_.begin(); pit != pillar.planar_points_.end(); pit++)
				// 	{
				// 		std::cout << "[" << pit->x << ", " << pit->y << "]\t";
				// 	}
				// std::cout<<std::endl;
				if( IsPillar( pillar ))
				{
					float distance = sqrt(pow(pillar.cen_.x , 2) + pow(pillar.cen_.y, 2));
					if(distance<2.0)
					{
							pillars.push_back(pillar);
					for(std::vector<cv::Point2f>::iterator pit = pillar.planar_points_.begin(); pit != pillar.planar_points_.end(); pit++)
					{
						std::cout << "[" << pit->x << ", " << pit->y << "]\t";
					}
					std::cout << "\nTotal " << pillar.planar_points_.size()<< "\n";
					}
				}
			}
		}
	}

	std::vector<PillarLine> jarLines;
	if( pillars.size() > 1 )
	{
		for( uint i = 0; i < pillars.size()-1; i++ )
		{
			Pillar jar1 = pillars[i];
			for( uint j = i+1; j < pillars.size(); j++ )
			{
				Pillar jar2 = pillars[j];
				float dist = PointDist(jar1.cen_, jar2.cen_);
				if(dist >0.88&& dist < 0.98)   //两个柱子间的距离，1m
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
void getOdom(const nav_msgs::Odometry &odom) //得到里程计信息
{
      car_x = odom.pose.pose.position.x;
      car_y = odom.pose.pose.position.y;
      tf::Quaternion quat;
      tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
      double roll, pitch, yaw;//定义存储r\p\y的容器
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
      car_a=yaw;
}
void getLaser(const sensor_msgs::LaserScan &laser) //得到雷达信息
{
      LaserPoint laserpoint;
      polar_laser_points_.clear();
      for(int i=0;i<laser.ranges.size();i++)
      {
        double theta=laser.range_min+laser.angle_increment*i;
        laserpoint.angle=theta;
        laserpoint.range=laser.ranges[i];
        polar_laser_points_.push_back(laserpoint);
      }
      Pose cp(car_x,car_y,car_a);
      std::vector<PillarLine> pillarlines = DetectPillar(cp, polar_laser_points_); //检测
      std::vector<Pose> lps;
      Pose lp;
      std::cout << "lines size=" << pillarlines.size() << std::endl;
      if (pillarlines.size() > 0)
      {
        for (int i = 0; i < pillarlines.size(); i++)
        {
          pillarlines[i].ShelfPose(lps);
        }
      }

      //  for(int i=0;i<lps.size();i++)
      // {
      //     std::cout<<"lp"<<i<<"=[ "<<lps[i].x<<","<<lps[i].y<<","<<lps[i].a<<" ]"<<" ";
      // }
      if (lps.size() > 0)
      {
        lp = lps[0];
        for (int i = 0; i < lps.size() - 1; i++)
        {
          for (int j = 0; j < lps.size() - i - 1; j++)
          {
            ToPi(lps[j].a);
            ToPi(lps[j + 1].a);
            if (abs(lps[j].a) > abs(lps[j + 1].a)) //probrom
            {
              Pose swap = lps[j + 1];
              lps[j + 1] = lps[j];
              lps[j] = swap;
            }
          }
        }
      }

      for (int i = 0; i < lps.size(); i++)
      {
        std::cout << "lp" << i << "=[ " << lps[i].x << "," << lps[i].y << "," << lps[i].a * 180 / M_PI << " ]"
                  << " ";
      }
      if (lps.size() == 1)
        lp = lps[0];
      if (lps.size() >= 2) //筛选最合适的边
      {
        float diss1 = lps[0].x * lps[0].x + lps[0].y * lps[0].y;
        float diss2 = lps[1].x * lps[1].x + lps[1].y * lps[1].y;
        std::cout << "angel=" << abs((lps[0].a - lps[1].a) * 180 / M_PI) << std::endl;
        if (abs((lps[0].a - lps[1].a) * 180 / M_PI) < 10)
        {
          if (diss1 > diss2)
          {
            lp = lps[1];
          }
          else
          {
            lp = lps[0];
          }
        }
        else
        {
          lp = lps[0];
        }
      }

      std::cout << std::endl;
      std::cout << "lps size=" << lps.size() << std::endl;
      std::cout << "select lp=" << lp.x << " " << lp.y << " " << lp.a << std::endl;
      std::cout << "befor_lp_=" << befor_lp_.a << " moveastage=" << moveStage << std::endl;
      float disstance;
      if (lp.a != 0)
      {
        float distance = sqrt(pow(lp.x, 2) + pow(lp.y, 2));
        if (distance < 2.0 && distance > 0.1 && (moveStage == 1 || moveStage == 3 || moveStage == 2 || moveStage == 4 || moveStage == 7))
        {
          destpose_ = lp;
          ToPi(destpose_.a);
          float A = cp.a;
          float X = lp.x + 0.15;

          destpose_.x = X * cos(A) - lp.y * sin(A) + cp.x; //转换加位移
          destpose_.y = X * sin(A) + lp.y * cos(A) + cp.y;
          destpose_.a += cp.a - 5 * M_PI / 180;
          float distance_eve_lp = sqrt(pow(befor_lp_.x - destpose_.x, 2) + pow(befor_lp_.y - destpose_.y, 2));
          if (befor_lp_.a == 0 && abs((destpose_.a - cp.a) * 180 / M_PI) < 30)
          {
            befor_lp_ = destpose_;
          }
          else if (distance_eve_lp < 0.2)
          {
            befor_lp_ = destpose_;
            disstance = sqrt(X * X + lp.y * lp.y);
            if(abs(befor_lp_.a)<M_PI/4) //手动左右平移3厘米
            {
                befor_lp_.y+=0.03*sin(befor_lp_.a-M_PI/2);
                befor_lp_.x+=0.03*cos(befor_lp_.a-M_PI/2);
            }
          }
        }
      }
      // std::cout << destpose_.x *cos(cp.a)-destpose_.y*sin(cp.a)+cp.x<< " " << destpose_.x *sin(cp.a)+destpose_.y*cos(cp.a)+cp.y << " " << Mod2Pi(destpose_.a+cp.a) << "\n";
      // std::cout<<"lp pose="<<lp.x<<" "<<lp.y<<" "<<lp.a*180/M_PI<<std::endl;
      std::cout << "car and des=" << cp.x << " " << cp.y << " " << cp.a * 180 / M_PI << " to " << befor_lp_.x << " " << befor_lp_.y << " " << befor_lp_.a * 180 / M_PI << std::endl;
      DrawLaserMapAndPillar("Laser", polar_laser_points_, cp, lps, befor_lp_ /*, cmdSeq*/);

      //Pose liftPose[jarLines.size()];
      // Pose car = ekf.GetRobotPose();
      //Pose lp=                      //选择出角度相近的架子做目标
      MotionPlan_shelf(cp, befor_lp_, moveStage, befor_lp_, disstance); //控制接入
      std::cout << "\n";
}
void execute(const service_shelf::DoDishesGoalConstPtr& goal, Server* as) 
{
  // Do lots of awesome groundbreaking robot stuff here
  Task=goal->lift_mode;
  //as->setSucceeded();
}
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "do_dishes_server");
  ros::NodeHandle n;
  n.subscribe("/odom",10,getOdom);
  n.subscribe("/scan",10,getLaser);
 
  vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
  vel_int16 = n.advertise<std_msgs::Int16>("/left_ctrl",10);
  Server server(n, "pick_goods", boost::bind(&execute, _1, &server), false);
  server_nh=&server;
  server.start();

  ros::Rate loop_rate(10);
   while (ros::ok())
  {
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = (double)cmd_speed_;
    vel_cmd.angular.z = (double)cmd_steer_;

    std_msgs::Int16 msg;
    msg.data = lift_speed_;
    if(Task==1||Task==2)
    {
      vel_int16.publish(msg);
      vel_pub.publish(vel_cmd);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  
 
  return 0;
}