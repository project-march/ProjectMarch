#pragma once

#include <xsens/xsintarray.h>

#include <sstream>

int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate);
