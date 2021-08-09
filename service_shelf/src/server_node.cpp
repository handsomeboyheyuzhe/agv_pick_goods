 
#include <service_shelf/DoDishesAction.h> 
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "tf/transform_datatypes.h"
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int16.h"
#include "../include/jar.h"
//Twist.h是机器运动速度消息包的格式定义文件
ros::Publisher vel_pub;
ros::Publisher vel_int16;
typedef actionlib::SimpleActionServer<service_shelf::DoDishesAction> Server;
Server *server_nh;
static int Task=0,moveStage=3;
static double car_x,car_y,car_a;
static bool get_lidar_=false;
static float cmd_speed_=0,cmd_steer_=0,cmd_lift_=0,lift_speed_=0;
boost::shared_mutex radardata_mutex_,send_data_mutex_,odomdata_mutex_;
std::vector<LaserPoint> polar_laser_points_;
Pose destpose_,befor_lp_,second_lp_;
std::vector<Pose> first_pose_,second_pose_;
float befor_second_diss_ = 0.0,befor_first_diss_ = 0.0,befor_second_angle_diff_ = 0.0,befor_firstanglediff_=0.0;

inline void SetMoveCommand(float speed, float steer, double lift)
{
  boost::shared_lock<boost::shared_mutex> guard(send_data_mutex_);
  cmd_speed_ = speed/2.0;
  cmd_steer_ = steer;
  cmd_lift_ = lift;
  //std::cout << "~~~~~~~~~~~ Move Cmd [" << speed << ", " << steer << ", " << (int)lift << "]\n";
}
inline void MotionPlan_shelf(Pose cp,Pose &lp,Pose second_lp,int &moveStage)
{
	ToPi(cp.a);
		ToPi(lp.a);
		float movespeed = 0.0;
		float liftpeed = 0.0;
		float angelspeed=0.0;
		float Dx=lp.x-cp.x;
		float Dy=lp.y-cp.y;
		float Dx2=second_lp.x-cp.x;
		float Dy2=second_lp.y-cp.y;
		float disstance=sqrt(Dy*Dy+Dx*Dx);
		float disstance2=sqrt(Dy2*Dy2+Dx2*Dx2);
		
		float anglediff=cp.a*180/M_PI-lp.a*180/M_PI;
		float anglediff2=cp.a*180/M_PI-atan2(Dy2,Dx2)*180/M_PI;
		std::cout<<"movestage="<<moveStage<<std::endl;
		std::cout<<"the first tar ange and dis="<<anglediff<<" "<<disstance<<std::endl;
		std::cout<<"the second tar ange and dis="<<anglediff2<<" "<<disstance2<<std::endl;
		
		if(moveStage==1&&anglediff2*befor_second_angle_diff_<0&&abs(anglediff2)<3)
		{
			moveStage=2;
		}
		if(moveStage==2&&disstance2>befor_second_diss_&&abs(disstance2)<0.07)
		{
			moveStage=3;
		}
		if(moveStage==3&&anglediff*befor_firstanglediff_<0&&abs(anglediff)<3)
		{
			moveStage=4;
		}
		if(moveStage==4&&abs(disstance)<0.1)
		{
			moveStage=5;
		}
		if(moveStage==6&&disstance>1.2)
		{
			lp.a=0;
			moveStage = 0;
			befor_second_diss_ = 0.0;
			befor_first_diss_ = 0.0;
			befor_second_angle_diff_ = 0.0;
			befor_firstanglediff_=0.0;
		}
		
		if (moveStage == 1) //turn to the second
		{
			
			float speed = 0;
			if (abs(anglediff2) > 15)
			{
				speed = 0.3;
			}
			else if(abs(anglediff2) > 10)
			{
				speed = 0.2;
			}
			else if(abs(anglediff2) > 5)
			{
				speed = 0.15;
			}
			else
			{
				speed = 0.1;
			}
			
			

			if (anglediff2 < 0)
			{
				angelspeed = speed;
			}
			else
			{
				angelspeed = -speed;
			}
		}
		if(moveStage==2)
		{
			movespeed=0.1;
			float speed = 0.1;
			// if (abs(anglediff2) > 15)
			// {
			// 	speed = 0.3;
			// }
			// else if(abs(anglediff2) > 10)
			// {
			// 	speed = 0.2;
			// }
			// else if(abs(anglediff2) > 5)
			// {
			// 	speed = 0.15;
			// }
			// else
			// {
			// 	speed = 0.1;
			// }
			
			
			if (anglediff2 < 0)
			{
				angelspeed = speed;
			}
				else
			{
				angelspeed = -speed;
			}
		}
		if(moveStage==3)  //turn to the first
		{
			float speed=0;

			if (abs(anglediff) > 15)
			{
				speed=0.3;
			}
			else if(abs(anglediff)>10)
			{
				speed=0.2;
			}
			else if(abs(anglediff)>5)
			{
				speed=0.13;
			}
			else
			{
				speed=0.05;
			}
			
			if(anglediff<0)
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
			movespeed=0.1;
			float speed=0.05;
			
			if(anglediff<0)
			{
				angelspeed=speed;
			}
			else
			{
				angelspeed=-speed;
			}
			
		}
		else if(moveStage==5)
		{
			// liftpeed=lift_speed_;
			// lift_speed_=-lift_speed_;
			// SetMoveCommand( movespeed, angelspeed, liftpeed );
			// SleepInMilliSeconds(20000);
			// moveStage=6;  //error
		}
		else if(moveStage==6)
		{
			movespeed=-0.1;
		}
		SetMoveCommand( movespeed, angelspeed, liftpeed );
		befor_second_diss_=disstance2;
		befor_first_diss_=disstance;
		befor_firstanglediff_=anglediff;
		befor_second_angle_diff_=anglediff2; 
}


void getOdom(const nav_msgs::Odometry &odom) //得到里程计信息
{
	  boost::shared_lock<boost::shared_mutex> guard(odomdata_mutex_);
      car_x = odom.pose.pose.position.x;
      car_y = odom.pose.pose.position.y;
      tf::Quaternion quat;
      tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
      double roll, pitch, yaw;//定义存储r\p\y的容器
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
      car_a=yaw;
	  //std::cout<<"get odom data=="<<car_x<<" "<<car_y<<" "<<car_a<<std::endl;
}
void getLaser(const sensor_msgs::LaserScan &laser) //得到雷达信息
{
	  boost::shared_lock<boost::shared_mutex> guard(radardata_mutex_);
      LaserPoint laserpoint;
      polar_laser_points_.clear();
      for(int i=0;i<laser.ranges.size();i++)
      {
        double theta=laser.range_min+laser.angle_increment*i;
		float the=(float)theta;
		ToPi(the);
		the=(the+174.0/180*M_PI);
		ToPi(the);
		
        laserpoint.angle=the;
        laserpoint.range=laser.ranges[i];
		//if(abs(the)<M_PI/2&&the<0)
        laserpoint.range=laser.ranges[i];
        	polar_laser_points_.push_back(laserpoint);
		std::cout<<"init laser="<<the*180/3.14<<" "<<laser.ranges[i]<<" "<<laser.intensities[i]<<std::endl;
      }
	  get_lidar_=true;
	  std::cout<<"init laser= get new radar data"<<std::endl;
		
}
Pose greaterpose(std::vector<Pose> poses,Pose cp)
{
    Pose pose(0.0,0.0,0.0);
    if (poses.size() > 0)
    {
        for (int i = 0; i < poses.size() - 1; i++)
        {
            for (int j = 0; j < poses.size() - i - 1; j++)
            {
                ToPi(poses[j].a);
                ToPi(poses[j + 1].a);
                if (abs(poses[j].a * 180 / M_PI-cp.a*180/M_PI) > abs(poses[j + 1].a * 180 / M_PI-cp.a*180/M_PI)) //probrom
                {
                    Pose swap = poses[j + 1];
                    poses[j + 1] = poses[j];
                    poses[j] = swap;
                }
            }
        }
        pose=poses[0];
    }
	 std::cout<<" pose=="<<pose.x<<" "<<pose.y<<" "<<pose.a<<std::endl;
	if (pose.a != 0)
	{
		float distance = sqrt(pow(pose.x, 2) + pow(pose.y, 2));
		std::cout << "dis=" << distance << std::endl;
		if (distance < 3.0 && distance > 0.1)
		{
			destpose_ = pose;
			ToPi(destpose_.a);
			float A = cp.a;
			float X = pose.x+0.25 ;
			float Dx = befor_lp_.x - cp.x;  //里程计坐标
			float Dy = befor_lp_.y - cp.y;
			float Y = pose.y;
			Pose secondlp;
			destpose_.x = X * cos(A) - Y * sin(A) + cp.x; //转换加位移
			destpose_.y = X * sin(A) + Y * cos(A) + cp.y;
			destpose_.a += cp.a;

			secondlp.x =destpose_.x-1.0*cos(destpose_.a) ; //转换加位移
			secondlp.y = destpose_.y-1.0*sin(destpose_.a);
			secondlp.a += destpose_.a;

			float distance_eve_lp = sqrt(pow(befor_lp_.x - destpose_.x, 2) + pow(befor_lp_.y - destpose_.y, 2));
			if (befor_lp_.a == 0 && abs((destpose_.a - cp.a) * 180 / M_PI) < 45)
			{
				befor_lp_ = destpose_;
				second_lp_ = secondlp;
				first_pose_.push_back(befor_lp_);
				second_pose_.push_back(secondlp);
			}
			else if (distance_eve_lp < 0.5)
			{
				first_pose_.insert(first_pose_.begin(),destpose_);
				if(first_pose_.size()>2)
					first_pose_.pop_back();
				second_pose_.insert(second_pose_.begin(),secondlp);
				if(second_pose_.size()>2)
					second_pose_.pop_back();
				befor_lp_ = AveragePose(first_pose_);
				second_lp_ = AveragePose(second_pose_);
			}
		}
	}
	return pose;
}
Pose GetRelativeCarPose(Pose initpose,Pose carpose)
{
	Pose relativepose;
	float dx=carpose.x-initpose.x;
	float dy=carpose.y-initpose.y;
	float a=atan2(carpose.y-initpose.y,carpose.x-initpose.x)-initpose.a;
	float diss=sqrtf(dx*dx+dy*dy);
	relativepose.x=diss*cos(a);
	relativepose.y=diss*sin(a);
	relativepose.a=carpose.a-initpose.a;
	return relativepose;
}
void execute(const service_shelf::DoDishesGoalConstPtr &goal, Server *as)
{
	// Do lots of awesome groundbreaking robot stuff here
	std::cout<<"the execute before task"<<std::endl;
	Task = goal->lift_mode;
	moveStage=1;
	std::cout<<"the execute before while"<<std::endl;
	Pose initCarPose(car_x, car_y, car_a);
	initCarPose.a = (float)initCarPose.a;
	while (true)
	{
		sleep(0.01);
		 if(get_lidar_)
		{
			Pose cp(0.0,0.0,0.0);
			{
				boost::shared_lock<boost::shared_mutex> guard(odomdata_mutex_);
				Pose car(car_x, car_y, car_a);
				car.a = (float)car_a;
				cp = car;
			}
			cp=GetRelativeCarPose(initCarPose,cp);
			std::vector<Pose> shelfposes;
			{
			boost::shared_lock<boost::shared_mutex> guard(radardata_mutex_);
		    shelfposes =DetectJar(cp, polar_laser_points_); //检测 
			}
			
            Pose lp=greaterpose(shelfposes,cp);

			DrawLaserMapAndPillar("Laser", polar_laser_points_, cp, befor_lp_ ,second_lp_/*, cmdSeq*/);
			MotionPlan_shelf(cp, befor_lp_,second_lp_, moveStage);
			
			
			
		}
		get_lidar_ = false;
		
		
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "do_dishes_server");
  ros::NodeHandle n;
  ros::Subscriber sub_odom = n.subscribe("/odom",10,getOdom);
  ros::Subscriber sub_laser = n.subscribe("/scan",10,getLaser);
 
  vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
  vel_int16 = n.advertise<std_msgs::Int16>("/lift_ctrl",10);
  Server server(n, "pick_goods", boost::bind(&execute, _1, &server), false);
  server_nh=&server;
  server.start();

  ros::Rate loop_rate(10);
  
   while (ros::ok())
  {
    geometry_msgs::Twist vel_cmd;

	{
		boost::shared_lock<boost::shared_mutex> guard(send_data_mutex_);
		vel_cmd.linear.x = (double)cmd_speed_;
		vel_cmd.angular.z = (double)cmd_steer_;
		
	}
     if(Task==1||Task==2)
     {
      vel_pub.publish(vel_cmd);
	  
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  	//  ros::spin();
  
 
  return 0;
}