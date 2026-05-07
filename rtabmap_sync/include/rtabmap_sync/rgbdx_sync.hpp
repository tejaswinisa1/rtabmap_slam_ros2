

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
#include "rtabmap_msgs/msg/rgbd_images.hpp"
#include "rtabmap_sync/CommonDataSubscriber.h"
#include "rtabmap_sync/SyncDiagnostic.h"

namespace rtabmap_sync
{

class RGBDXSync : public rclcpp::Node
{
public:
	RTABMAP_SYNC_PUBLIC
	explicit RGBDXSync(const rclcpp::NodeOptions & options);

	virtual ~RGBDXSync();

	void callback(
			const sensor_msgs::msg::Image::ConstSharedPtr image,
			const sensor_msgs::msg::Image::ConstSharedPtr depth,
			const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo);

private:
	DATA_SYNCS2(rgbd2, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS3(rgbd3, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS4(rgbd4, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS5(rgbd5, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS6(rgbd6, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS7(rgbd7, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)
	DATA_SYNCS8(rgbd8, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage, rtabmap_msgs::msg::RGBDImage)

private:
	rclcpp::Publisher<rtabmap_msgs::msg::RGBDImages>::SharedPtr rgbdImagesPub_;

	std::vector<message_filters::Subscriber<rtabmap_msgs::msg::RGBDImage>*> rgbdSubs_;

	std::unique_ptr<SyncDiagnostic> syncDiagnostic_;
};

}

