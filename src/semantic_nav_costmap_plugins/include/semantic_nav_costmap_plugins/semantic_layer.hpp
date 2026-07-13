#ifndef SEMANTIC_NAV_COSTMAP_PLUGINS__SEMANTIC_LAYER_HPP_
#define SEMANTIC_NAV_COSTMAP_PLUGINS__SEMANTIC_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "tf2_ros/buffer.h"

#include "semantic_nav_interfaces/msg/semantic_object_array.hpp"

namespace semantic_nav_costmap_plugins
{

/**
 * One object the layer is currently keeping alive.
 *
 * This struct is the whole reason the layer exists as a stateful component. The
 * detector is deliberately memoryless - it only ever says "this is what I see in
 * this frame". Persistence, ageing and forgetting all live here, in one place.
 */
struct TrackedObject
{
  std::string class_id;
  double x;                 // in the costmap's global frame
  double y;
  double cost_radius;       // metres; how far the keep-out zone extends
  double confidence;        // confidence at the moment it was last observed
  rclcpp::Time last_seen;   // used to age the object out
};

/**
 * A Nav2 costmap layer that paints keep-out zones around semantically meaningful
 * objects (people, animals) detected by the camera.
 *
 * WHY THIS IS C++ AND NOT PYTHON
 *   Costmap layers are loaded by pluginlib as shared libraries directly into the
 *   Nav2 process. This is not a style preference - a Python layer is not
 *   loadable at all.
 *
 * WHAT THIS ADDS OVER THE LIDAR
 *   The lidar already sees a person as an anonymous cylinder and the obstacle
 *   layer already marks it. What the lidar cannot do is treat that cylinder
 *   differently *because it is a person*. This layer supplies the class-conditional
 *   behaviour: a larger keep-out radius, because people move unpredictably and the
 *   cost of hitting one is not comparable to brushing a wall.
 *
 * DERIVES FROM CostmapLayer, NOT Layer
 *   CostmapLayer gives this layer its own grid, which is then merged into the
 *   master costmap. That matters because it lets us combine with updateWithMax:
 *   we can only ever *raise* a cell's cost, never lower it. If we wrote directly
 *   into the master grid we could accidentally erase a real lidar obstacle that
 *   happens to sit inside a person's radius - a silent, dangerous bug.
 */
class SemanticLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SemanticLayer() = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;

  // Tells Nav2 that a "clear costmap" recovery is allowed to wipe this layer.
  // We return true: if the robot is genuinely stuck, a stale person that is no
  // longer there must not be able to trap it forever. The per-object decay below
  // is the primary mechanism; this is the escape hatch of last resort.
  bool isClearable() override {return true;}

private:
  void objectsCallback(const semantic_nav_interfaces::msg::SemanticObjectArray::SharedPtr msg);

  /**
   * Age every tracked object and drop the ones that have faded out.
   *
   * This is the decay the whole design hinges on. An object's confidence falls off
   * with time since it was last seen; once it falls below min_confidence_ it is
   * forgotten. Fresh detections reset the clock.
   *
   * Note what this buys us: when the detector publishes an EMPTY array (it sees
   * nothing), nothing gets refreshed, so everything decays naturally. That is why
   * the detector must publish empty arrays rather than staying silent - silence
   * would be indistinguishable from a crashed detector, and we would have no
   * safe way to tell "the person left" from "the camera died".
   */
  void decayObjects(const rclcpp::Time & now);

  rclcpp::Subscription<semantic_nav_interfaces::msg::SemanticObjectArray>::SharedPtr sub_;

  // The callback runs on an executor thread while updateBounds/updateCosts run on
  // the costmap's own thread. Every access to objects_ must be guarded.
  std::mutex mutex_;
  std::vector<TrackedObject> objects_;

  // Parameters
  std::string topic_;
  double decay_time_;          // seconds for confidence to fall to zero
  double min_confidence_;      // below this, an object is forgotten
  double association_distance_;  // metres; same-class detections closer than this
                                 // are treated as the same object
  double lethal_core_radius_;  // metres; inside this, cells are hard-blocked
  bool enabled_param_;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("SemanticLayer")};

  // Bounds of what we painted last cycle. We must re-touch these next cycle even if
  // the object is gone, otherwise the cells it occupied are never revisited and its
  // cost stays burned into the master grid forever.
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool has_last_bounds_{false};
};

}  // namespace semantic_nav_costmap_plugins

#endif  // SEMANTIC_NAV_COSTMAP_PLUGINS__SEMANTIC_LAYER_HPP_