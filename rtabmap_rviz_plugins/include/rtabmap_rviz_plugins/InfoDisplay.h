

#ifndef INFO_DISPLAY_H
#define INFO_DISPLAY_H

#include <memory>
#include <set>
#include <string>
#include <vector>
#include <utility>

#include <rtabmap_rviz_plugins/visibility.h>
#include <rtabmap_msgs/msg/info.hpp>

#include <rviz_common/display.hpp>
#include "rviz_common/message_filter_display.hpp"
#include <rtabmap/core/Transform.h>

namespace rtabmap_rviz_plugins
{

class RTABMAP_RVIZ_PLUGINS_PUBLIC InfoDisplay: public rviz_common::MessageFilterDisplay<rtabmap_msgs::msg::Info>
{
Q_OBJECT
public:
	InfoDisplay();
	virtual ~InfoDisplay();

	virtual void reset();
	virtual void update( float wall_dt, float ros_dt );

protected:
	/** @brief Do initialization. Overridden from MessageFilterDisplay. */
	virtual void onInitialize();

	/** @brief Process a single message.  Overridden from MessageFilterDisplay. */
	virtual void processMessage( const rtabmap_msgs::msg::Info::ConstSharedPtr cloud );

private:
	QString info_;
	int globalCount_;
	int localCount_;
	std::map<std::string, float> statistics_;
	rtabmap::Transform loopTransform_;
	std::mutex info_mutex_;
};

} // namespace rtabmap_rviz_plugins

#endif
