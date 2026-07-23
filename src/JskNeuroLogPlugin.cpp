#include "jsk_robot_log_manager/JskNeuroLogPlugin.hpp"

extern "C"
{
  AbstractPlugin* makePlugin(void)
  {
    return new JskNeuroLogPlugin();
  }
}
