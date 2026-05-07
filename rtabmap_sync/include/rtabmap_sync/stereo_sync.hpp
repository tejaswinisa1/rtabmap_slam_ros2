

#include <rtabmap_sync/visibility.h>
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/subscriber.hpp>

#include "rtabmap_msgs/msg/rgbd_image.hpp"
#include "rtabmap_sync/SyncDiagnostic.h"

namespace rtabmap_sync
{

class StereoSync : public rclcpp::Node
{
public:
	RTABMAP_SYNC_PUBLIC
	explicit StereoSync(const rclcpp::NodeOptions & options);

	virtual ~StereoSync();

	void callback(
			  const sensor_msgs::msg::Image::ConstSharedPtr imageLeft,
			  const sensor_msgs::msg::Image::ConstSharedPtr imageRight,
			  const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoLeft,
			  const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoRight);
private:
	double compressedRate_;
	double approxSyncMaxInterval_;
	rclcpp::Time lastCompressedPublished_;

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbdImagePub_;
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbdImageCompressedPub_;

	image_transport::SubscriberFilter imageLeftSub_;
	image_transport::SubscriberFilter imageRightSub_;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoLeftSub_;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoRightSub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;

	std::unique_ptr<SyncDiagnostic> syncDiagnostic_;
};

}

