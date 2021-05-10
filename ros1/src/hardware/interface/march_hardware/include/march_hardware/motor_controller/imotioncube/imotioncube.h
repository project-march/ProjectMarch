// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_IMOTIONCUBE_H
#define MARCH_HARDWARE_IMOTIONCUBE_H
#include "imotioncube_state.h"
#include "imotioncube_target_state.h"
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/ethercat/imotioncube_pdo_map.h"
#include "march_hardware/ethercat/pdo_types.h"
#include "march_hardware/ethercat/sdo_interface.h"
#include "march_hardware/ethercat/slave.h"
#include "march_hardware/motor_controller/actuation_mode.h"
#include "march_hardware/motor_controller/motor_controller.h"
#include <march_hardware/motor_controller/motor_controller_state.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace march {
class IMotionCube : public MotorController
/* You will often find that the documentation references to the IMC Manual. This
 * Manual can be found at
 * https://technosoftmotion.com/wp-content/uploads/P091.025.iMOTIONCUBE.CAN_.CAT_.UM_-1.pdf
 * Or if this link no longer works a copy can be found on the NAS:
 * 'march/2020-2021/03 -
 * Technical/Motorcontrollers/IMotionCube/P091.064.EtherCAT.iPOS.UM.pdf'
 */
{
public:
    /**
     * Constructs an IMotionCube with an incremental and absolute encoder.
     *
     * @param slave slave of the IMotionCube
     * @param absolute_encoder pointer to absolute encoder, required so cannot
     * be nullptr
     * @param incremental_encoder pointer to incremental encoder, required so
     * cannot be nullptr
     * @param actuation_mode actuation mode in which the IMotionCube must
     * operate
     * @throws std::invalid_argument When an absolute or incremental encoder is
     * nullptr.
     */
    IMotionCube(const Slave& slave,
        std::unique_ptr<AbsoluteEncoder> absolute_encoder,
        std::unique_ptr<IncrementalEncoder> incremental_encoder,
        ActuationMode actuation_mode);
    IMotionCube(const Slave& slave,
        std::unique_ptr<AbsoluteEncoder> absolute_encoder,
        std::unique_ptr<IncrementalEncoder> incremental_encoder,
        std::string& sw_stream, ActuationMode actuation_mode);

    ~IMotionCube() noexcept override = default;

    // Override functions for actuating the IMotionCube
    void prepareActuation() override;
    void actuateRadians(float target_position) override;
    void actuateTorque(float target_torque) override;

    // Transform the ActuationMode to a number that is understood by the
    // IMotionCube
    int getActuationModeNumber() const override;

    // Get a full description of the state of the IMotionCube
    std::unique_ptr<MotorControllerState> getState() override;

    // Getters for specific information about the state of the motor and the
    // IMotionCube
    float getTorque() override;
    float getMotorCurrent() override;
    float getMotorControllerVoltage() override;
    float getMotorVoltage() override;

    constexpr static float MAX_TARGET_DIFFERENCE = 0.393;
    // This value is slightly larger than the current limit of the
    // linear joints defined in the URDF.
    const static int16_t MAX_TARGET_TORQUE = 23500;

    // Constant used for converting a fixed point 16.16 bit number to a float,
    // which is done by dividing by 2^16
    static constexpr float FIXED_POINT_TO_FLOAT_CONVERSION = 1 << 16;

    // iMOTIONCUBE setting (slow loop sample period)
    static constexpr float TIME_PER_VELOCITY_SAMPLE = 0.004;

protected:
    // Override protected functions from Slave class
    bool initSdo(SdoSlaveInterface& sdo, int cycle_time) override;
    void reset(SdoSlaveInterface& sdo) override;

    // Override protected functions from MotorController class
    float getAbsolutePositionUnchecked() override;
    float getIncrementalPositionUnchecked() override;
    float getAbsoluteVelocityUnchecked() override;
    float getIncrementalVelocityUnchecked() override;

private:
    // Actuate position in Internal Units
    void actuateIU(int32_t target_iu);

    // Set the IMotionCube in a certain state
    void goToTargetState(const IMotionCubeTargetState& target_state);
    void setControlWord(uint16_t control_word);

    // Getters for information about the state of the IMotionCube
    int32_t getAbsolutePositionIU();
    int32_t getIncrementalPositionIU();
    float getAbsoluteVelocityIU();
    float getIncrementalVelocityIU();
    uint16_t getStatusWord();
    uint16_t getMotionError();
    uint16_t getDetailedError();
    uint16_t getSecondDetailedError();

    void mapMisoPDOs(SdoSlaveInterface& sdo);
    void mapMosiPDOs(SdoSlaveInterface& sdo);
    /**
     * Initializes all iMC by checking the setup on the drive and writing
     * necessary SDO registers.
     * @param sdo SDO interface to write to
     * @param cycle_time the cycle time of the EtherCAT
     * @return 1 if reset is necessary, otherwise it returns 0
     */
    bool writeInitialSettings(SdoSlaveInterface& sdo, int cycle_time);
    /**
     * Calculates checksum on .sw file passed in string format in sw_string_ by
     * simple summation until next empty line. Start_address and end_address are
     * filled in the method and used for downloading the .sw file to the drive.
     * @param start_address the found start address of the setup in the .sw file
     * @param end_address the found end address of the setup in the .sw file
     * @return the computed checksum in the .sw file
     */
    uint16_t computeSWCheckSum(uint16_t& start_address, uint16_t& end_address);
    /**
     * Compares the checksum of the .sw file and the setup on the drive. If both
     * are equal 1 is returned. This method makes use of the
     * computeSWCheckSum(int&, int&) method. The start and end addresses are
     * used in conjunction with the registers 0x2069 and 0x206A (described in
     * the CoE manual fro Technosoft(2019) in par. 16.2.5 and 16.2.6) to
     * determine the checksum on the drive.
     * @return true or 1 if the setup is verified and therefore correct,
     * otherwise returns 0
     */
    bool verifySetup(SdoSlaveInterface& sdo);
    /**
     * Downloads the setup on the .sw file onto the drive using 32bit SDO write
     * functions. The general process is specified in chapter 16.4 of the CoE
     * manual from Technosoft(2019).
     */
    void downloadSetupToDrive(SdoSlaveInterface& sdo);

    // Use of smart pointers are necessary here to make dependency injection
    // possible and thus allow for mocking the encoders. A unique pointer is
    // chosen since the IMotionCube should be the owner and the encoders
    // do not need to be passed around.
    std::string sw_string_;

    std::unordered_map<IMCObjectName, uint8_t> miso_byte_offsets_;
    std::unordered_map<IMCObjectName, uint8_t> mosi_byte_offsets_;
};

} // namespace march
#endif // MARCH_HARDWARE_IMOTIONCUBE_H