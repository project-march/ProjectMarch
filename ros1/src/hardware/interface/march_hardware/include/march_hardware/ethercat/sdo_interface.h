#ifndef MARCH_HARDWARE_SDO_INTERFACE_H
#define MARCH_HARDWARE_SDO_INTERFACE_H
#include <cstdint>
#include <memory>
#include <utility>

namespace march {
/**
 * An interface to read and write Service Data Objects (SDOs).
 */
class SdoInterface {
public:
    /**
     * Writes an SDO to given location.
     *
     * @tparam T type of the SDO to write
     * @param slave index of the slave
     * @param index index of the register
     * @param sub sub index inside the register
     * @param value value to write
     * @return received working counter from the read operation. Returns 0 when
     * a failure occurred, positive otherwise.
     */
    template <typename T>
    int write(uint16_t slave, uint16_t index, uint8_t sub, T value)
    {
        return this->write(slave, index, sub, sizeof(T), &value);
    }

    /**
     * Reads an SDO from given location.
     *
     * @tparam T type to read SDO to
     * @param slave index of the slave
     * @param index index of the register
     * @param sub sub index inside the register
     * @param val_size reference to the size of the value. Will output the size
     * of the read value.
     * @param value reference to value to read to
     * @return received working counter from the read operation. Returns 0 when
     * a failure occurred, positive otherwise.
     */
    template <typename T>
    int read(uint16_t slave, uint16_t index, uint8_t sub, int& val_size,
        T& value) const
    {
        return this->read(slave, index, sub, val_size, &value);
    }

protected:
    virtual int write(uint16_t slave, uint16_t index, uint8_t sub,
        std::size_t size, void* value)
        = 0;

    virtual int read(uint16_t slave, uint16_t index, uint8_t sub, int& val_size,
        void* value) const = 0;
};

/**
 * Shared pointer to an SdoInterface.
 */
using SdoInterfacePtr = std::shared_ptr<SdoInterface>;

/**
 * An interface to read and write Service Data Objects (SDOs) for a given slave.
 */
class SdoSlaveInterface {
public:
    SdoSlaveInterface(uint16_t slave_index, SdoInterfacePtr sdo)
        : slave_index_(slave_index)
        , sdo_(sdo)
    {
    }

    template <typename T> int write(uint16_t index, uint8_t sub, T value)
    {
        return this->sdo_->write(this->slave_index_, index, sub, value);
    }

    template <typename T>
    int read(uint16_t index, uint8_t sub, int& val_size, T& value) const
    {
        return this->sdo_->read(
            this->slave_index_, index, sub, val_size, value);
    }

private:
    const uint16_t slave_index_;
    SdoInterfacePtr sdo_;
};

/**
 * An implementation of the SdoInterface using SOEM.
 */
class SdoInterfaceImpl : public SdoInterface {
public:
    /**
     * Creates a new shared SdoInterfaceImpl.
     * @return Generic PdoInterface shared ptr
     */
    static SdoInterfacePtr create()
    {
        return std::make_shared<SdoInterfaceImpl>();
    }

protected:
    int write(uint16_t slave, uint16_t index, uint8_t sub, std::size_t size,
        void* value) override;

    int read(uint16_t slave, uint16_t index, uint8_t sub, int& val_size,
        void* value) const override;
};
} // namespace march
#endif // MARCH_HARDWARE_SDO_INTERFACE_H
