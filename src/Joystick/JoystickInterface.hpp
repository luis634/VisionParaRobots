#ifndef _LOGITECF710_H_
#define _LOGITECF710_H_
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>
#include "joystick.hpp"

/**
 * @brief This class provides an intuitive interface for a joystick
 * controller.
 * By: Alberto Jahuey Moncada A01039835@itesm.mx
 */

class JoystickInterface {
 public:
  /**
   * @brief Construct a new Joystick object
   *
   * @param port
   * @param updateRateMs
   */
  JoystickInterface(const unsigned int port = 0,
                    const unsigned long updateRateMicroS = 10);

  /**
   * @brief Whether or not the Logitech controller is connected
   * initialization
   *
   * @return true
   * @return false
   */
  bool isConnected();

  /**
   * @brief Destroy the Logitech F 1 7 0 object
   *
   */
  ~JoystickInterface();

  /**
   * @brief Get the value, mapped from -1.0 to 1.0, of the requested axis as of
   * the last update.
   * @param axis
   * @return double
   */
  float getAxisValue(uint8_t axis);
  /**
   * @brief Get the state of the requested button, returns true if the button is
   * pressed, false otherwise
   *
   * @param button
   * @return true
   * @return false
   */
  bool getButtonState(uint8_t button);

 private:
  /**
   * @brief Update joystick values
   *
   */
  void updateJoystickValues();
  bool running, joystickFound;

  Joystick js;
  std::thread updateThread;
  std::mutex mut;
  const unsigned long updateRateMs;

  int portNumber;

  std::map<uint8_t, float> axes;

  std::map<uint8_t, bool> buttons;
};

#endif