// Copyright 2019 Project March.
#include <march_shared_msgs/Error.h>

struct ErrorCounter
{
  ErrorCounter() : count(0)
  {
  }

  void cb(const march_shared_msgs::Error&)
  {
    ++count;
  }

  size_t count;
};