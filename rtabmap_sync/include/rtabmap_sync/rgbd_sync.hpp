

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

class RGBDSync : public rclcpp::Node
{
public:
	RTABMAP_SYNC_PUBLIC
	explicit RGBDSync(const rclcpp::NodeOptions & options);

	virtual ~RGBDSync();

	void callback(
			const sensor_msgs::msg::Image::ConstSharedPtr image,
			const sensor_msgs::msg::Image::ConstSharedPtr depth,
			const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo);

private:
	double depthScale_;
	int decimation_;
	double compressedRate_;
	double approxSyncMaxInterval_;

	rclcpp::Time lastCompressedPublished_;

	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbdImagePub_;
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbdImageCompressedPub_;

	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoSub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MyApproxSyncDepthPolicy;
	message_filters::Synchronizer<MyApproxSyncDepthPolicy> * approxSyncDepth_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MyExactSyncDepthPolicy;
	message_filters::Synchronizer<MyExactSyncDepthPolicy> * exactSyncDepth_;

	std::unique_ptr<SyncDiagnostic> syncDiagnostic_;
};

}

