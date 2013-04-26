#ifndef _mocap_align_hpp
#define _mocap_align_hpp

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace mocap_align
{
	class MocapAlign : public nodelet::Nodelet
	{
	public:
		MocapAlign( );
		~MocapAlign( );
		virtual void onInit( );
		void UAVOdomCB( const nav_msgs::OdometryPtr &msg );
		void PendulumOdomCB( const nav_msgs::OdometryPtr &msg );
	private:
		void Iterate( );
		void spin( );
		void spinOnce( );

		// ROS Interface
		ros::NodeHandle nh;
		ros::NodeHandle nh_priv;
		ros::Rate r;
		tf::TransformListener li;
		tf::TransformBroadcaster br;

		// Parameters
		std::string frame_base;
		std::string frame_a;
		std::string frame_b;

		boost::thread spin_thread;
	};
}

#endif /* _mocap_align_hpp */
