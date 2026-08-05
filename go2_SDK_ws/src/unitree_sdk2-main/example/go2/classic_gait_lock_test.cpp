#include <gtest/gtest.h>

#include "classic_gait_lock.h"

TEST(ClassicGaitLock, DoesNotEnforceOutsideNavigation)
{
  ClassicGaitLock lock(2.0);

  lock.updateNavigationState("IDLE");

  EXPECT_FALSE(lock.isLocked());
  EXPECT_FALSE(lock.isEnforcementDue(10.0));
}

TEST(ClassicGaitLock, EnforcesImmediatelyAndPeriodicallyDuringNavigation)
{
  ClassicGaitLock lock(2.0);

  lock.updateNavigationState("PATH_ALIGNING");
  EXPECT_TRUE(lock.isLocked());
  EXPECT_TRUE(lock.isEnforcementDue(10.0));

  lock.recordEnforcement(10.0);
  EXPECT_FALSE(lock.isEnforcementDue(11.9));
  EXPECT_TRUE(lock.isEnforcementDue(12.0));

  lock.recordEnforcement(12.0);
  lock.updateNavigationState("ACTIVE");
  EXPECT_FALSE(lock.isEnforcementDue(12.0));
}

TEST(ClassicGaitLock, ReleasesWhenNavigationFinishes)
{
  ClassicGaitLock lock(2.0);
  lock.updateNavigationState("GOAL_ALIGNING");
  lock.recordEnforcement(10.0);

  lock.updateNavigationState("SUCCEEDED");

  EXPECT_FALSE(lock.isLocked());
  EXPECT_FALSE(lock.isEnforcementDue(20.0));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
