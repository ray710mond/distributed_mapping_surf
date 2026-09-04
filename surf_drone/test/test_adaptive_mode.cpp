#include <gtest/gtest.h>

#include "surf_drone/adaptive_mode.hpp"
#include "surf_multirobot_msgs/msg/voxel_delta.hpp"

namespace
{

TEST(AdaptiveMode, UsesHoldTimesAndHysteresis)
{
  surf_drone::AdaptiveModeController::Config config;
  config.full_min_mbps = 10.0;
  config.delta_min_mbps = 3.0;
  config.dynamic_min_mbps = 1.0;
  config.up_hysteresis_mbps = 1.0;
  config.down_hold_seconds = 2.0;
  config.up_hold_seconds = 4.0;
  config.minimum_dwell_seconds = 0.0;
  config.ewma_alpha = 1.0;
  surf_drone::AdaptiveModeController controller(config);

  EXPECT_EQ(controller.update(0.0, 5.0, 0U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_VOXEL_DELTAS);
  EXPECT_EQ(controller.update(1.0, 0.5, 0U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_VOXEL_DELTAS);
  EXPECT_EQ(controller.update(3.1, 0.5, 0U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_METADATA_ONLY);
  EXPECT_EQ(controller.update(4.0, 12.0, 0U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_METADATA_ONLY);
  EXPECT_EQ(controller.update(8.1, 12.0, 0U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_FULL);
}

TEST(AdaptiveMode, NeverUpgradesWhileTheLinkQueueIsCongested)
{
  surf_drone::AdaptiveModeController::Config config;
  config.full_min_mbps = 10.0;
  config.delta_min_mbps = 3.0;
  config.dynamic_min_mbps = 1.0;
  config.up_hysteresis_mbps = 0.0;
  config.down_hold_seconds = 0.0;
  config.up_hold_seconds = 0.0;
  config.minimum_dwell_seconds = 0.0;
  config.ewma_alpha = 1.0;
  config.congested_queue_depth = 2U;
  surf_drone::AdaptiveModeController controller(config);

  EXPECT_EQ(controller.update(0.0, 0.1, 2U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_VOXEL_DELTAS);
  EXPECT_EQ(controller.update(0.1, 0.1, 2U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_METADATA_ONLY);
  EXPECT_EQ(controller.update(0.2, 20.0, 2U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_METADATA_ONLY);
}

}  // namespace
