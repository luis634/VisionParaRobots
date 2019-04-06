#include "JoystickInterface.hpp"

JoystickInterface::JoystickInterface(const unsigned int port,
                                     const unsigned long updateRateMicroS)
    : js(port), updateRateMs(updateRateMs) {
  portNumber = port;
  joystickFound = js.isFound();
  if (joystickFound) {
    running = true;
    updateThread = std::thread(&JoystickInterface::updateJoystickValues, this);
  }
}

bool JoystickInterface::isConnected() {
  DIR* dir;
  struct dirent* ent;
  if ((dir = opendir("/dev/input/")) != NULL) {
    std::stringstream sstm;
    sstm << "js" << portNumber;
    while ((ent = readdir(dir)) != NULL) {
      if (strcmp(ent->d_name, sstm.str().c_str()) == 0) {
        closedir(dir);
        return true;
      }
    }
    closedir(dir);
  }
  return false;
}

float JoystickInterface::getAxisValue(uint8_t axis) {
  double output = axes[axis];
  return output;
}

bool JoystickInterface::getButtonState(uint8_t button) {
  bool output = buttons[button];
  return output;
}

void JoystickInterface::updateJoystickValues() {
  while (running) {
    JoystickEvent event;

    if (js.sample(&event)) {
      if (event.isButton()) {
        buttons[event.number] = event.value;
      } else if (event.isAxis()) {
        axes[event.number] = ((float)event.value / event.MAX_AXES_VALUE);
      }
    }

    if (!isConnected()) {
      for (std::pair<const uint8_t, float>& pair : axes) {
        pair.second = 0.0;
      }
      for (std::pair<const uint8_t, bool>& pair : buttons) {
        pair.second = false;
      }
    }

    std::this_thread::sleep_for(std::chrono::microseconds(updateRateMs));
  }
}

JoystickInterface::~JoystickInterface() {
  running = false;
  if (updateThread.joinable()) {
    updateThread.join();
  }
}
