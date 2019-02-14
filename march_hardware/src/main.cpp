#include "march_hardware/March4.h"
#include "march_hardware/Joint.h"

int main(int argc, char** argv)
{
    MARCH4 march4 = MARCH4();

    printf("%f\n", march4.getJoint("TEMP IMC").getTemperature());
    printf("%f\n", march4.getJoint("TEMP IMC").getAngle());


    printf("%f\n", march4.getJoint("TEMP").getTemperature());
    printf("%f\n", march4.getJoint("TEMP").getAngle());
    printf("%f\n", march4.getJoint("IMC").getTemperature());
    printf("%f\n", march4.getJoint("IMC").getAngle());
    printf("%f\n", march4.getJoint("NONE").getTemperature());
    printf("%f\n", march4.getJoint("NONE").getAngle());
}
