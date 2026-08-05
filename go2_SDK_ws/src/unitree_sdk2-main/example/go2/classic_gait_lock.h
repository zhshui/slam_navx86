#ifndef UNITREE_SDK2_CLASSIC_GAIT_LOCK_H
#define UNITREE_SDK2_CLASSIC_GAIT_LOCK_H

#include <limits>
#include <string>

class ClassicGaitLock
{
public:
  explicit ClassicGaitLock(double enforcement_interval)
      : enforcement_interval_(enforcement_interval),
        locked_(false),
        last_enforcement_(-std::numeric_limits<double>::infinity())
  {
  }

  void setEnforcementInterval(double enforcement_interval)
  {
    if (enforcement_interval > 0.0)
      enforcement_interval_ = enforcement_interval;
  }

  bool updateNavigationState(const std::string& state)
  {
    const bool should_lock =
        state == "PATH_ALIGNING" ||
        state == "ACTIVE" ||
        state == "GOAL_ALIGNING";
    const bool changed = should_lock != locked_;
    locked_ = should_lock;
    if (changed && locked_)
      last_enforcement_ = -std::numeric_limits<double>::infinity();
    return changed;
  }

  bool isLocked() const
  {
    return locked_;
  }

  bool isEnforcementDue(double now) const
  {
    return locked_ && now - last_enforcement_ >= enforcement_interval_;
  }

  void recordEnforcement(double now)
  {
    last_enforcement_ = now;
  }

private:
  double enforcement_interval_;
  bool locked_;
  double last_enforcement_;
};

#endif
