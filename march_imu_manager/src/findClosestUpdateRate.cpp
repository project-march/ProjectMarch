#include "findClosestUpdateRate.h"

int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate)
{
    if (supportedUpdateRates.empty())
    {
        return 0;
    }

    if (supportedUpdateRates.size() == 1)
    {
        return supportedUpdateRates[0];
    }

    int uRateDist = -1;
    int closestUpdateRate = -1;
    for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
    {
        const int currDist = std::abs(*itUpRate - desiredUpdateRate);

        if ((uRateDist == -1) || (currDist < uRateDist))
        {
            uRateDist = currDist;
            closestUpdateRate = *itUpRate;
        }
    }
    return closestUpdateRate;
}
