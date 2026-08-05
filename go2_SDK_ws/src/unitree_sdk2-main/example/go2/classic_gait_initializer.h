#ifndef UNITREE_SDK2_CLASSIC_GAIT_INITIALIZER_H
#define UNITREE_SDK2_CLASSIC_GAIT_INITIALIZER_H

template <typename SetClassicGait, typename WaitBetweenAttempts>
int initializeClassicGait(SetClassicGait set_classic_gait,
                          WaitBetweenAttempts wait_between_attempts,
                          int max_attempts)
{
  int result = -1;
  for (int attempt = 0; attempt < max_attempts; ++attempt)
  {
    result = set_classic_gait();
    if (result == 0)
      return result;

    if (attempt + 1 < max_attempts)
      wait_between_attempts();
  }
  return result;
}

#endif
