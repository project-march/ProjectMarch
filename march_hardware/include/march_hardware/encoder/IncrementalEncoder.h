// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_INCREMENTAL_ENCODER_H
#define MARCH_HARDWARE_INCREMENTAL_ENCODER_H
#include "march_hardware/encoder/Encoder.h"

#include <ostream>

namespace march
{
class IncrementalEncoder : public Encoder
{
public:
  IncrementalEncoder(size_t number_of_bits, double transmission);

  double toRad(int32_t iu) const override;
  double getRadPerBit() const override;
  double getTransmission() const;

  /** @brief Override comparison operator */
  friend bool operator==(const IncrementalEncoder& lhs, const IncrementalEncoder& rhs)
  {
    return lhs.getSlaveIndex() == rhs.getSlaveIndex() && lhs.getTotalPositions() == rhs.getTotalPositions() &&
           lhs.transmission_ == rhs.transmission_;
  }
  /** @brief Override stream operator for clean printing */
  friend std::ostream& operator<<(std::ostream& os, const IncrementalEncoder& encoder)
  {
    return os << "slaveIndex: " << encoder.getSlaveIndex() << ", "
              << "totalPositions: " << encoder.getTotalPositions() << ", "
              << "transmission: " << encoder.transmission_;
  }

private:
  const double transmission_;
};
}  // namespace march

#endif  // MARCH_HARDWARE_INCREMENTAL_ENCODER_H
