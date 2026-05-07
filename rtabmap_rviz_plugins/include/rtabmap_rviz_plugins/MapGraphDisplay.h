

#ifndef MAP_GRAPH_DISPLAY_H
#define MAP_GRAPH_DISPLAY_H

#include <rtabmap_rviz_plugins/visibility.h>
#include <rtabmap_msgs/msg/map_graph.hpp>

#include <rviz_common/message_filter_display.hpp>

namespace Ogre
{
class ManualObject;
}

namespace rviz_common
{

namespace properties
{

class ColorProperty;
class FloatProperty;

}  // namespace properties

}  // namespace rviz_common

namespace rtabmap_rviz_plugins
{

/**
 * \class MapGraphDisplay
 * \brief Displays the graph of rtabmap::MapGraph message
 */
class RTABMAP_RVIZ_PLUGINS_PUBLIC MapGraphDisplay: public rviz_common::MessageFilterDisplay<rtabmap_msgs::msg::MapGraph>
{
Q_OBJECT
public:
  MapGraphDisplay();
  virtual ~MapGraphDisplay();

  /** @brief Overridden from Display. */
  virtual void reset();

protected:
  /** @brief Overridden from Display. */
  virtual void onInitialize();

  /** @brief Overridden from MessageFilterDisplay. */
  void processMessage( const rtabmap_msgs::msg::MapGraph::ConstSharedPtr msg );

private:
  void destroyObjects();

  std::vector<Ogre::ManualObject*> manual_objects_;

  rviz_common::properties::ColorProperty* color_neighbor_property_;
  rviz_common::properties::ColorProperty* color_neighbor_merged_property_;
  rviz_common::properties::ColorProperty* color_global_property_;
  rviz_common::properties::ColorProperty* color_local_property_;
  rviz_common::properties::ColorProperty* color_landmark_property_;
  rviz_common::properties::ColorProperty* color_user_property_;
  rviz_common::properties::ColorProperty* color_virtual_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
};

} // namespace rtabmap_rviz_plugins

#endif /* MAP_GRAPH_DISPLAY_H */

