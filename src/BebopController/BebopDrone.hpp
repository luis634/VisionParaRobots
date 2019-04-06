/*
    Copyright (C) 2014 Parrot SA

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
    * Neither the name of Parrot nor the names
    of its contributors may be used to endorse or promote products
    derived from this software without specific prior written
    permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.
*/

/*
    Code by:    Alberto Jahuey Moncada  A01039835@itesm.mx
                Abiel Fernandez Cantu   A01197654@itesm.mx

    For ITESM, Robot Vision class

*/

//#define THREADED_TIMED_MOVEMENT
#ifndef BEBOP_CONTROLLER_H_
#define BEBOP_CONTROLLER_H_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <string>

#include <errno.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <thread>

extern "C" {
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARSAL/ARSAL.h>

#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/avutil.h"
#include "libavutil/common.h"
#include "libavutil/imgutils.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libswscale/swscale.h"
}

#define TAG "BebopProgram"
using namespace cv;
using namespace std;
using namespace std::chrono;

enum class DroneMoveDirection { Longitudinal, Sideways, Vertical };

class BebopDrone {
 public:
  static BebopDrone &getInstance(const char *droneIP = "192.168.42.1",
                                 const int discoveryPort = 44444) {
    static BebopDrone singletonInstance(droneIP, discoveryPort);
    return singletonInstance;
  }

  ARCONTROLLER_Frame_t getCurrentFrame();
  Mat getFrameAsMat();

  int getYaw();
  int getPitch();
  int getRoll();
  int getBatteryLevel();

  void takeoff();
  void hover();
  void land();
  void emergencyStop();

  void setYaw(int amount);
  void setRoll(int amount);
  void setPitch(int amount);
  void setVerticalSpeed(int amount);
  void setYawRollPitchVSpeed(int yawAmount, int rollAmount, int pitchAmount,
                             int verticalSpeed);

#ifdef THREADED_TIMED_MOVEMENT
  void moveTimed(DroneMoveDirection direction, double seconds,
                 int movementAmount = 50);

  bool isMovingTimed();
#endif

 private:
  BebopDrone(const char *droneIP, const int discoveryPort);
  BebopDrone(BebopDrone const &);
  BebopDrone &operator=(BebopDrone const &);
  ~BebopDrone();

  static int batteryLevel;
  static int yaw, pitch, roll, vertical;

  static const int heightFrame = 480;
  static const int widthFrame = 856;
  AVCodec *codec;
  static AVCodecContext *ctx;
  static AVFrame *frame;
  static AVPacket packet;
  static AVFrame *framebgr;

  const char *BEBOP_IP_ADDRESS;
  const int BEBOP_DISCOVERY_PORT;

  int failed = 0;
  static ARSAL_Sem_t stateSem;
  ARDISCOVERY_Device_t *device = NULL;
  ARCONTROLLER_Device_t *deviceController = NULL;
  static ARCONTROLLER_Frame_t currentFrame;
  static Mat currentFrameMat;

  eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
  eARCONTROLLER_DEVICE_STATE deviceState = ARCONTROLLER_DEVICE_STATE_MAX;

  // called when the state of the device controller has changed
  static void stateChanged(eARCONTROLLER_DEVICE_STATE newState,
                           eARCONTROLLER_ERROR error, void *customData);

  // called when a command has been received from the drone
  static void commandReceived(
      eARCONTROLLER_DICTIONARY_KEY commandKey,
      ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, void *customData);

  static void cmdBatteryStateChangedRcv(
      ARCONTROLLER_Device_t *deviceController,
      ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);

  static void cmdSensorStateListChangedRcv(
      ARCONTROLLER_Device_t *deviceController,
      ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);

  // called when a streaming frame has been received
  static eARCONTROLLER_ERROR didReceiveFrame(ARCONTROLLER_Frame_t *frameDrone,
                                             void *customData);

  static eARCONTROLLER_ERROR decoderConfig(ARCONTROLLER_Stream_Codec_t codec,
                                           void *customData);

  double checkAskedOutput(double uncheckedAmount);

#ifdef THREADED_TIMED_MOVEMENT
  void waitAndHover(double seconds);
  bool currentlyMovingTimed;
  thread waitAndHoverThread;

#endif
};

#endif