#include "pointcloud_concatenate/pointcloud_concatenate.hpp"

// Constructor
PointcloudConcatenate::PointcloudConcatenate(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
	nh_ = nh; // Set nodehandle
	node_name_ = ros::this_node::getName();

	// Initialise variables / parameters to class variables
	handleParams();

	// Initialization tf2 listener
	tfBuffer.reset(new tf2_ros::Buffer);
	tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

	// Initialise publishers and subscribers
	// Queues size of 1 to only keep the most recent message
	sub_cloud_front = nh_.subscribe("cloud_front", 1, &PointcloudConcatenate::subCallbackCloudFront, this);
	sub_cloud_back = nh_.subscribe("cloud_back", 1, &PointcloudConcatenate::subCallbackCloudBack, this);
	sub_cloud_left = nh_.subscribe("cloud_left", 1, &PointcloudConcatenate::subCallbackCloudLeft, this);
	sub_cloud_right = nh_.subscribe("cloud_right", 1, &PointcloudConcatenate::subCallbackCloudRight, this);
	sub_cloud_top = nh_.subscribe("cloud_top", 1, &PointcloudConcatenate::subCallbackCloudTop, this);
	pub_cloud_out = nh_.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);
}

// Destructor
PointcloudConcatenate::~PointcloudConcatenate()
{
	// Free up allocated memory
	ROS_INFO("Destructing PointcloudConcatenate...");
	// delete pointer_name;
}

void PointcloudConcatenate::subCallbackCloudFront(sensor_msgs::PointCloud2 msg)
{
	cloud_front = msg;
	cloud_front_received = true;
	cloud_front_received_recent = true;
}

void PointcloudConcatenate::subCallbackCloudBack(sensor_msgs::PointCloud2 msg)
{
	cloud_back = msg;
	cloud_back_received = true;
	cloud_back_received_recent = true;
}

void PointcloudConcatenate::subCallbackCloudLeft(sensor_msgs::PointCloud2 msg)
{
	cloud_left = msg;
	cloud_left_received = true;
	cloud_left_received_recent = true;
}

void PointcloudConcatenate::subCallbackCloudRight(sensor_msgs::PointCloud2 msg)
{
	cloud_right = msg;
	cloud_right_received = true;
	cloud_right_received_recent = true;
}

void PointcloudConcatenate::subCallbackCloudTop(sensor_msgs::PointCloud2 msg)
{
	cloud_top = msg;
	cloud_top_received = true;
	cloud_top_received_recent = true;
}

void PointcloudConcatenate::handleParams()
{
	// Handle parameters

	// Set parameters
	ROS_INFO("Loading parameters...");
	std::string param_name;

	// Target frame
	std::string parse_str;
	param_name = node_name_ + "/target_frame";
	ros::param::get(param_name, parse_str);
	if (!parse_str.length() > 0)
	{
		param_frame_target_ = "base_link";
		ROSPARAM_WARN(param_name, param_frame_target_);
	}
	param_frame_target_ = parse_str;

	// Number of pointclouds
	param_name = node_name_ + "/clouds";
	if (!ros::param::get(param_name, param_clouds_))
	{
		param_clouds_ = 2;
		ROSPARAM_WARN(param_name, param_clouds_);
	}

	// Frequency to update/publish
	param_name = node_name_ + "/hz";
	if (!ros::param::get(param_name, param_hz_))
	{
		param_hz_ = 10;
		ROSPARAM_WARN(param_name, param_hz_);
	}

	ROS_INFO("Parameters loaded.");
}

double PointcloudConcatenate::getHz()
{
	return param_hz_;
}

void PointcloudConcatenate::update()
{
	// Is run periodically and handles calling the different methods

	if (pub_cloud_out.getNumSubscribers() > 0 && param_clouds_ >= 1)
	{
		// Initialise pointclouds
		sensor_msgs::PointCloud2 cloud_to_concat;
		cloud_out = cloud_to_concat; // Clear the output pointcloud

		// Track success of transforms
		bool success = true;

		// Sleep if no pointclouds have been received yet
		if ((!cloud_front_received) && (!cloud_back_received) && (!cloud_left_received) && (!cloud_right_received) && (!cloud_top_received))
		{
			ROS_WARN("No pointclouds received yet. Waiting 1 second...");

			// Set end time
			ros::Time end = ros::Time::now();
			end.sec += 1;
			// Sleep
			ros::Time::sleepUntil(end);

			return;
		}

		ros::Time pc_top_time, pc_left_time, pc_right_time, pc_front_time, pc_back_time;

		// Concatenate the first pointcloud (top)
		if (param_clouds_ >= 1 && success && cloud_top_received)
		{
			// Warn if cloud was not received since last update
			if (!cloud_top_received_recent)
			{
				ROS_WARN("Cloud TOP was not received since last update, reusing last received message...");
			}
			cloud_top_received_recent = false;

			// Transform pointcloud to the target frame
			success = pcl_ros::transformPointCloud(param_frame_target_, cloud_top, cloud_to_concat, *tfBuffer);
			if (!success)
			{
				ROS_WARN("Transforming cloud TOP from %s to %s failed!", cloud_top.header.frame_id.c_str(), param_frame_target_.c_str());
			}

			// Concatenate the pointcloud
			if (success)
			{
				pcl::concatenatePointCloud(cloud_out, cloud_to_concat, cloud_out);
			}
			pc_top_time = cloud_top.header.stamp;
		}

		// Concatenate the second pointcloud (left)
		if (param_clouds_ >= 2 && success && cloud_left_received)
		{
			// Warn if cloud was not received since last update
			if (!cloud_left_received_recent)
			{
				ROS_WARN("Cloud LEFT was not received since last update, reusing last received message...");
			}
			cloud_left_received_recent = false;

			// Transform pointcloud to the target frame
			success = pcl_ros::transformPointCloud(param_frame_target_, cloud_left, cloud_to_concat, *tfBuffer);
			if (!success)
			{
				ROS_WARN("Transforming cloud LEFT from %s to %s failed!", cloud_left.header.frame_id.c_str(), param_frame_target_.c_str());
			}

			// Concatenate the pointcloud
			if (success)
			{
				pcl::concatenatePointCloud(cloud_out, cloud_to_concat, cloud_out);
			}
			pc_left_time = cloud_left.header.stamp;
		}

		// Concatenate the third pointcloud (right)
		if (param_clouds_ >= 3 && success && cloud_right_received)
		{
			// Warn if cloud was not received since last update
			if (!cloud_right_received_recent)
			{
				ROS_WARN("Cloud RIGHT was not received since last update, reusing last received message...");
			}
			cloud_right_received_recent = false;

			// Transform pointcloud to the target frame
			success = pcl_ros::transformPointCloud(param_frame_target_, cloud_right, cloud_to_concat, *tfBuffer);
			if (!success)
			{
				ROS_WARN("Transforming cloud RIGHT from %s to %s failed!", cloud_right.header.frame_id.c_str(), param_frame_target_.c_str());
			}

			// Concatenate the pointcloud
			if (success)
			{
				pcl::concatenatePointCloud(cloud_out, cloud_to_concat, cloud_out);
			}
			pc_right_time = cloud_right.header.stamp;
		}

		// Concatenate the fourth pointcloud (front)
		if (param_clouds_ >= 4 && success && cloud_front_received)
		{
			// Warn if cloud was not received since last update
			if (!cloud_front_received_recent)
			{
				ROS_WARN("Cloud FRONT was not received since last update, reusing last received message...");
			}
			cloud_front_received_recent = false;

			// Transform pointcloud to the target frame
			// Here we just assign the pointcloud directly to the output to ensure the secondary
			// data is inherited correctly.
			success = pcl_ros::transformPointCloud(param_frame_target_, cloud_front, cloud_out, *tfBuffer);
			if (!success)
			{
				ROS_WARN("Transforming cloud FRONT from %s to %s failed!", cloud_front.header.frame_id.c_str(), param_frame_target_.c_str());
			}
			pc_front_time = cloud_front.header.stamp;
		}

		// Concatenate the fifth pointcloud (back)
		if (param_clouds_ >= 5 && success && cloud_back_received)
		{
			// Warn if cloud was not received since last update
			if (!cloud_back_received_recent)
			{
				ROS_WARN("Cloud BACK was not received since last update, reusing last received message...");
			}
			cloud_back_received_recent = false;

			// Transform pointcloud to the target frame
			success = pcl_ros::transformPointCloud(param_frame_target_, cloud_back, cloud_to_concat, *tfBuffer);
			if (!success)
			{
				ROS_WARN("Transforming cloud BACK from %s to %s failed!", cloud_back.header.frame_id.c_str(), param_frame_target_.c_str());
			}

			// Concatenate the pointcloud
			if (success)
			{
				pcl::concatenatePointCloud(cloud_out, cloud_to_concat, cloud_out);
			}
			pc_back_time = cloud_back.header.stamp;
		}

		ROS_INFO("TIMES: TOP: %d.%d, LEFT: %d.%d, RIGHT: %d.%d, FRONT: %d.%d, BACK: %d.%d",
				 pc_top_time.sec, pc_top_time.nsec,
				 pc_left_time.sec, pc_left_time.nsec,
				 pc_right_time.sec, pc_right_time.nsec,
				 pc_front_time.sec, pc_front_time.nsec,
				 pc_back_time.sec, pc_back_time.nsec);

		std::vector<uint32_t> nsec_values = {pc_top_time.nsec, pc_left_time.nsec, pc_right_time.nsec /*, pc_front_time.nsec, pc_back_time.nsec*/};
		uint32_t max_nsec = *std::max_element(nsec_values.begin(), nsec_values.end());
		uint32_t min_nsec = *std::min_element(nsec_values.begin(), nsec_values.end());
		uint32_t max_diff_nsec = max_nsec - min_nsec;

		ROS_INFO("Max difference in nsec: %d", max_diff_nsec);

		// Publish the concatenated pointcloud
		if (success)
		{
			publishPointcloud(cloud_out);
		}
	}
}

void PointcloudConcatenate::publishPointcloud(sensor_msgs::PointCloud2 cloud)
{
	// Publishes the combined pointcloud

	// Update the timestamp
	cloud.header.stamp = ros::Time::now();
	// Publish
	pub_cloud_out.publish(cloud);
}