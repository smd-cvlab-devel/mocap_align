#include "mocap_align/mocap_align.hpp"

#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include <iomanip>

PLUGINLIB_DECLARE_CLASS(mocap_align, MocapAlign, mocap_align::MocapAlign, nodelet::Nodelet)

namespace mocap_align
{
	MocapAlign::MocapAlign( )
		: r( 100 )
		, frame_base( "map" )
		, frame_a( "frame_a" )
		, frame_b( "frame_b" )
	{
	}

	MocapAlign::~MocapAlign( )
	{
		spin_thread.interrupt( );
		if( spin_thread.joinable( ) )
			spin_thread.join( );
	}

	void MocapAlign::onInit( )
	{
		int rate;

		nh = getNodeHandle( );
		nh_priv = getPrivateNodeHandle( );

		nh_priv.param( "frame_base", frame_base, (std::string)"map" );
		nh_priv.param( "frame_a", frame_a, (std::string)"frame_a" );
		nh_priv.param( "frame_b", frame_b, (std::string)"frame_b" );
		nh_priv.param( "rate", rate, 100 );

		r = ros::Rate( rate );

		spin_thread = boost::thread( &MocapAlign::spin, this );
	}

	void MocapAlign::spin( )
	{
		while( 1 )
		{
			boost::this_thread::interruption_point( );
			spinOnce( );
			r.sleep( );
		}
	}

	void MocapAlign::spinOnce( )
	{
		static tf::StampedTransform tf_a;
		static tf::StampedTransform tf_b;

		try
		{
			li.lookupTransform( frame_base, frame_a, ros::Time(0), tf_a );
			li.lookupTransform( frame_base, frame_b, ros::Time(0), tf_b );
		}
		catch( tf::TransformException ex )
		{
			ROS_INFO( "Missed a transform...chances are that we are still OK" );
			return;
		}
		if( tf_a.getOrigin( ).x( ) != tf_a.getOrigin( ).x( ) ||
			tf_b.getOrigin( ).x( ) != tf_b.getOrigin( ).x( ) )
		{
			ROS_WARN( "NaN DETECTED" );
			return;
		}

		// Rotate us to be aligned with the uav
		const tf::Quaternion delta_quat = tf::createQuaternionFromRPY( 0, 0, tf::getYaw( tf_a.getRotation( ) ) - tf::getYaw( tf_b.getRotation( ) ) );
		const tf::Quaternion quat_b_aligned = tf_b.getRotation( ) * delta_quat;

		// Broadcast the aligned tf
		tf::StampedTransform tf_b_aligned( tf_b );
		if( tf_b_aligned.getOrigin( ).x( ) != tf_b_aligned.getOrigin( ).x( ) )
		{
			ROS_WARN( "NaN PRODUCED" );
			return;
		}
		tf_b_aligned.setRotation( quat_b_aligned );
		tf_b_aligned.child_frame_id_ = tf_b_aligned.child_frame_id_ + "_aligned";
		br.sendTransform( tf_b_aligned );
	}
}
