#include "semantic_nav_costmap_plugins/semantic_layer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace semantic_nav_costmap_plugins
{

void SemanticLayer::onInitialize()
{
  // Layer gives us node_ as a weak_ptr. Nav2 owns the node's lifetime, not us, so
  // we lock it to get a usable handle and fail loudly if it has already gone away.
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"SemanticLayer: failed to lock node"};
  }
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  // Parameters are namespaced under the layer's own name (e.g. "semantic_layer.topic"),
  // which is what lets the same plugin be loaded twice with different settings.
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/semantic_objects")));
  declareParameter("decay_time", rclcpp::ParameterValue(3.0));
  declareParameter("min_confidence", rclcpp::ParameterValue(0.2));
  declareParameter("association_distance", rclcpp::ParameterValue(0.75));
  declareParameter("lethal_core_radius", rclcpp::ParameterValue(0.35));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".decay_time", decay_time_);
  node->get_parameter(name_ + ".min_confidence", min_confidence_);
  node->get_parameter(name_ + ".association_distance", association_distance_);
  node->get_parameter(name_ + ".lethal_core_radius", lethal_core_radius_);

  // NO_INFORMATION, not FREE_SPACE. This is the single most important line in the
  // file. updateWithMax() skips any cell we leave as NO_INFORMATION, so an unpainted
  // cell means "this layer has no opinion here". If the default were FREE_SPACE we
  // would be actively asserting that every cell we did not touch is drivable, and
  // when merged we would fight with the lidar layer over empty space.
  default_value_ = NO_INFORMATION;

  // Allocate our own grid to match the master costmap's geometry, then blank it.
  matchSize();
  resetMaps();

  // Reliable QoS, matching the detector's publisher. A best-effort subscriber paired
  // with a reliable publisher (or vice versa) simply never receives anything, with no
  // error - the same silent failure mode we hit on the camera topics.
  sub_ = node->create_subscription<semantic_nav_interfaces::msg::SemanticObjectArray>(
    topic_, rclcpp::QoS(10).reliable(),
    std::bind(&SemanticLayer::objectsCallback, this, std::placeholders::_1));

  current_ = true;
  RCLCPP_INFO(logger_, "SemanticLayer initialised, subscribed to %s", topic_.c_str());
}

void SemanticLayer::objectsCallback(
  const semantic_nav_interfaces::msg::SemanticObjectArray::SharedPtr msg)
{
  const std::string global_frame = layered_costmap_->getGlobalFrameID();

  std::lock_guard<std::mutex> lock(mutex_);
  const rclcpp::Time now = clock_->now();

  for (const auto & obj : msg->objects) {
    double x = obj.position.x;
    double y = obj.position.y;

    // The detector publishes in odom. The costmap's global frame is usually odom too
    // (local costmap) but will be map once AMCL/SLAM is added, so we cannot assume
    // they are the same - we transform when they differ.
    if (msg->header.frame_id != global_frame) {
      geometry_msgs::msg::PointStamped in, out;
      in.header = msg->header;
      in.point = obj.position;
      try {
        // TimePointZero == "latest available transform", not the message's stamp.
        // Same trade-off as in the detector: under a low real-time factor the sensor
        // stamps run ahead of TF and an exact-stamp lookup fails every single cycle.
        // The cost is a small pose error while the robot moves; measured at ~10 cm,
        // which disappears inside a 2 m keep-out radius.
        in.header.stamp = rclcpp::Time(0, 0, clock_->get_clock_type());
        out = tf_->transform(in, global_frame, tf2::durationFromSec(0.1));
        x = out.point.x;
        y = out.point.y;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "SemanticLayer: dropping object, TF %s -> %s failed: %s",
          msg->header.frame_id.c_str(), global_frame.c_str(), ex.what());
        continue;
      }
    }

    // --- Association -------------------------------------------------------
    // The detector is stateless and sends track_id = -1, so identity has to be
    // recovered here. Nearest same-class object within association_distance_ is
    // assumed to be the same thing we saw last frame.
    //
    // Known weakness: two people standing closer together than this threshold will
    // merge into one tracked object. The clean fix is to switch the detector to
    // ultralytics' model.track(persist=True), which supplies a real track_id; the
    // message already carries the field for exactly that reason. Then this block
    // becomes a lookup by id instead of a distance search.
    TrackedObject * match = nullptr;
    double best_dist = association_distance_;
    for (auto & known : objects_) {
      if (known.class_id != obj.class_id) {
        continue;
      }
      const double d = std::hypot(known.x - x, known.y - y);
      if (d < best_dist) {
        best_dist = d;
        match = &known;
      }
    }

    if (match) {
      // Refresh: a re-observation resets the decay clock and overwrites the pose.
      match->x = x;
      match->y = y;
      match->cost_radius = obj.cost_radius;
      match->confidence = obj.confidence;
      match->last_seen = now;
    } else {
      objects_.push_back(
        TrackedObject{obj.class_id, x, y, obj.cost_radius, obj.confidence, now});
    }
  }

  // Deliberately no "clear everything not in this message". An object leaving the
  // camera's field of view is not the same as it ceasing to exist - the robot turns
  // its head constantly. Objects are only ever removed by ageing out, never by a
  // single frame failing to mention them.
}

void SemanticLayer::decayObjects(const rclcpp::Time & now)
{
  objects_.erase(
    std::remove_if(
      objects_.begin(), objects_.end(),
      [&](const TrackedObject & obj) {
        const double age = (now - obj.last_seen).seconds();
        // Linear falloff to zero over decay_time_. Linear rather than exponential
        // because it gives a hard, predictable forgetting horizon: after decay_time_
        // seconds without a sighting the object is definitely gone. An exponential
        // tail would leave faint ghosts around indefinitely.
        const double factor = 1.0 - (age / decay_time_);
        return (obj.confidence * factor) < min_confidence_;
      }),
    objects_.end());
}

void SemanticLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  decayObjects(clock_->now());

  // Re-touch whatever we painted last cycle, even if the object that caused it is
  // now gone. updateCosts() is only ever called for the bounded window, so a cell we
  // stop reporting is a cell that never gets recomputed - the old cost would stay
  // burned into the master grid and the robot would refuse to enter a space where
  // someone merely used to be. This one block is what makes the decay actually visible.
  if (has_last_bounds_) {
    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);
  }

  bool any = false;
  double bx_min = 0.0, by_min = 0.0, bx_max = 0.0, by_max = 0.0;

  for (const auto & obj : objects_) {
    const double r = obj.cost_radius;
    const double ox_min = obj.x - r;
    const double ox_max = obj.x + r;
    const double oy_min = obj.y - r;
    const double oy_max = obj.y + r;

    if (!any) {
      bx_min = ox_min; bx_max = ox_max;
      by_min = oy_min; by_max = oy_max;
      any = true;
    } else {
      bx_min = std::min(bx_min, ox_min);
      bx_max = std::max(bx_max, ox_max);
      by_min = std::min(by_min, oy_min);
      by_max = std::max(by_max, oy_max);
    }

    // touch() widens the window Nav2 will ask us to recompute.
    *min_x = std::min(*min_x, ox_min);
    *min_y = std::min(*min_y, oy_min);
    *max_x = std::max(*max_x, ox_max);
    *max_y = std::max(*max_y, oy_max);
  }

  if (any) {
    last_min_x_ = bx_min; last_min_y_ = by_min;
    last_max_x_ = bx_max; last_max_y_ = by_max;
    has_last_bounds_ = true;
  } else {
    has_last_bounds_ = false;
  }
}

void SemanticLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const rclcpp::Time now = clock_->now();

  // Wipe our own grid before repainting. Without this, a cell painted three seconds
  // ago would still hold its old value: nothing else ever clears it, and updateWithMax
  // can only raise the master's cost, never lower it. The decay would compute
  // correctly and change nothing on screen.
  resetMaps();

  const double res = master_grid.getResolution();

  for (const auto & obj : objects_) {
    const double age = (now - obj.last_seen).seconds();
    const double effective_conf =
      obj.confidence * std::max(0.0, 1.0 - (age / decay_time_));

    if (effective_conf < min_confidence_) {
      continue;  // decayObjects will collect it on the next bounds pass
    }

    const double r = obj.cost_radius;

    // Convert the object's bounding box into cell indices, then walk the cells.
    // Iterating a box and rejecting by distance is simpler and quite fast enough at
    // these radii; a circle-drawing algorithm would be premature optimisation.
    unsigned int mx_min, my_min, mx_max, my_max;
    if (!master_grid.worldToMap(obj.x - r, obj.y - r, mx_min, my_min) ||
      !master_grid.worldToMap(obj.x + r, obj.y + r, mx_max, my_max))
    {
      // The object is (partly) off the edge of the rolling costmap window. Skipping
      // is correct: the robot cannot collide with something outside the map it plans in.
      continue;
    }

    // Clamp to the window Nav2 asked us to update. Writing outside it is undefined -
    // those cells are not ours this cycle.
    const int i0 = std::max(static_cast<int>(mx_min), min_i);
    const int i1 = std::min(static_cast<int>(mx_max), max_i - 1);
    const int j0 = std::max(static_cast<int>(my_min), min_j);
    const int j1 = std::min(static_cast<int>(my_max), max_j - 1);

    for (int j = j0; j <= j1; ++j) {
      for (int i = i0; i <= i1; ++i) {
        double wx, wy;
        master_grid.mapToWorld(
          static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);

        const double d = std::hypot(wx - obj.x, wy - obj.y);
        if (d > r) {
          continue;  // outside the circle inscribed in the box
        }

        unsigned char cost;
        if (d <= lethal_core_radius_) {
          // The body itself. Hard-blocked regardless of confidence: if we believe
          // there is a person here at all, driving through them is never acceptable.
          cost = LETHAL_OBSTACLE;
        } else {
          // Social buffer. Cost falls off linearly from the body out to cost_radius,
          // and is scaled by how much we still trust the observation. The result is a
          // gradient the planner will slide around rather than a wall it must stop at.
          //
          // Capped below LETHAL on purpose: this zone is a strong preference, not a
          // physical obstacle. Making it lethal would let a single low-confidence
          // detection seal a corridor and strand the robot - the exact failure mode
          // that the decay is here to prevent.
          const double falloff = (r - d) / (r - lethal_core_radius_);
          cost = static_cast<unsigned char>(
            falloff * effective_conf * static_cast<double>(INSCRIBED_INFLATED_OBSTACLE));
        }

        if (cost == FREE_SPACE) {
          continue;  // leave the cell as NO_INFORMATION rather than claiming it is free
        }

        // Two people's radii can overlap. Keep the higher cost - the same "never lower
        // an existing risk" rule we apply when merging into the master grid.
        const unsigned int index = getIndex(
          static_cast<unsigned int>(i), static_cast<unsigned int>(j));
        if (costmap_[index] == NO_INFORMATION || costmap_[index] < cost) {
          costmap_[index] = cost;
        }
      }
    }
  }

  (void)res;  // resolution is implicit in mapToWorld; kept for readability of the math

  // Merge with max, never overwrite. The lidar layer's obstacles and the static map
  // must survive us: our grid can only ever raise a cell's cost. Overwriting would
  // let a person's soft social buffer erase a hard wall standing behind them.
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void SemanticLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  objects_.clear();
  has_last_bounds_ = false;
  resetMaps();
  current_ = true;
}

}  // namespace semantic_nav_costmap_plugins

// Registers the class with pluginlib. Without this macro (and the matching XML) Nav2
// can load the shared library but cannot find the class inside it, and fails at
// runtime with an unhelpful "does not exist" error.
PLUGINLIB_EXPORT_CLASS(
  semantic_nav_costmap_plugins::SemanticLayer, nav2_costmap_2d::Layer)