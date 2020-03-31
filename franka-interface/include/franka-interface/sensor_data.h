#ifndef FRANKA_INTERFACE_SENSOR_DATA_H_
#define FRANKA_INTERFACE_SENSOR_DATA_H_

#include <franka-interface-common/definitions.h>

class SensorData {
 public:
  explicit SensorData(SharedBufferTypePtr v) : values_{v} {};

  /**
   * Parse data from memory.
   */
  virtual void parse_data() = 0;

  /**
   * Initialize any other hand.
   */
  virtual void initialize_data() = 0;

  /**
   * Should we terminate the current skill.
   */
  virtual bool update_data() = 0;

 protected:
  SharedBufferTypePtr values_=0;
};

#endif  // FRANKA_INTERFACE_SENSOR_DATA_H_