#include <gtest/gtest.h>

#include <vector>

#include "classic_gait_initializer.h"

TEST(ClassicGaitInitializer, ReturnsImmediatelyWhenFirstAttemptSucceeds)
{
  int calls = 0;
  int waits = 0;

  const int result = initializeClassicGait(
      [&calls]() {
        ++calls;
        return 0;
      },
      [&waits]() { ++waits; },
      3);

  EXPECT_EQ(0, result);
  EXPECT_EQ(1, calls);
  EXPECT_EQ(0, waits);
}

TEST(ClassicGaitInitializer, RetriesUntilClassicGaitSucceeds)
{
  const std::vector<int> results{-1, -1, 0};
  std::size_t calls = 0;
  int waits = 0;

  const int result = initializeClassicGait(
      [&results, &calls]() { return results.at(calls++); },
      [&waits]() { ++waits; },
      3);

  EXPECT_EQ(0, result);
  EXPECT_EQ(3u, calls);
  EXPECT_EQ(2, waits);
}

TEST(ClassicGaitInitializer, StopsAfterMaximumAttempts)
{
  int calls = 0;
  int waits = 0;

  const int result = initializeClassicGait(
      [&calls]() {
        ++calls;
        return -1;
      },
      [&waits]() { ++waits; },
      3);

  EXPECT_EQ(-1, result);
  EXPECT_EQ(3, calls);
  EXPECT_EQ(2, waits);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
