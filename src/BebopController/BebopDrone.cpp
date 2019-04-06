/*
    Code by:    Alberto Jahuey Moncada  A01039835@itesm.mx
                Abiel Fernandez Cantu   A01197654@itesm.mx

    For ITESM, Robot Vision class

*/

#include "BebopDrone.hpp"

AVCodecContext* BebopDrone::ctx;
AVFrame* BebopDrone::frame;
AVPacket BebopDrone::packet;
AVFrame* BebopDrone::framebgr;

ARSAL_Sem_t BebopDrone::stateSem;
ARCONTROLLER_Frame_t BebopDrone::currentFrame;
Mat BebopDrone::currentFrameMat;

int BebopDrone::yaw;
int BebopDrone::pitch;
int BebopDrone::roll;
int BebopDrone::vertical;
int BebopDrone::batteryLevel;

BebopDrone::BebopDrone(const char* droneIP, const int discoveryPort)
    : BEBOP_IP_ADDRESS(droneIP), BEBOP_DISCOVERY_PORT(discoveryPort) {
  ARSAL_Sem_Init(&stateSem, 0, 0);

  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- Bebop Program --");

  // create a discovery device
  if (!failed) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- init discovey device ... ");
    eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;

    device = ARDISCOVERY_Device_New(&errorDiscovery);

    if (errorDiscovery == ARDISCOVERY_OK) {
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG,
                  "    - ARDISCOVERY_Device_InitWifi ...");
      // create a Bebop drone discovery device (ARDISCOVERY_PRODUCT_ARDRONE)

      errorDiscovery = ARDISCOVERY_Device_InitWifi(
          device, ARDISCOVERY_PRODUCT_ARDRONE, "bebop", BEBOP_IP_ADDRESS,
          BEBOP_DISCOVERY_PORT);

      if (errorDiscovery != ARDISCOVERY_OK) {
        failed = 1;
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s",
                    ARDISCOVERY_Error_ToString(errorDiscovery));
      }
    } else {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s",
                  ARDISCOVERY_Error_ToString(errorDiscovery));
      failed = 1;
    }
  }

  // create a device controller
  if (!failed) {
    deviceController = ARCONTROLLER_Device_New(device, &error);

    if (error != ARCONTROLLER_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,
                  "Creation of deviceController failed.");
      failed = 1;
    }
  }

  if (!failed) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- delete discovey device ... ");
    ARDISCOVERY_Device_Delete(&device);
  }

  // add the state change callback to be informed when the device controller
  // starts, stops...
  if (!failed) {
    error = ARCONTROLLER_Device_AddStateChangedCallback(
        deviceController, this->stateChanged, deviceController);

    if (error != ARCONTROLLER_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "add State callback failed.");
      failed = 1;
    }
  }

  // add the command received callback to be informed when a command has been
  // received from the device
  if (!failed) {
    error = ARCONTROLLER_Device_AddCommandReceivedCallback(
        deviceController, this->commandReceived, deviceController);

    if (error != ARCONTROLLER_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "add callback failed.");
      failed = 1;
    }
  }

  // add the frame received callback to be informed when a streaming frame has
  // been received from the device
  if (!failed) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- set Video callback ... ");
    error = ARCONTROLLER_Device_SetVideoStreamCallbacks(
        deviceController, this->decoderConfig, this->didReceiveFrame, NULL,
        NULL);

    if (error != ARCONTROLLER_OK) {
      failed = 1;
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error: %s",
                  ARCONTROLLER_Error_ToString(error));
    }
  }

  if (!failed) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Connecting ...");
    error = ARCONTROLLER_Device_Start(deviceController);

    if (error != ARCONTROLLER_OK) {
      failed = 1;
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s",
                  ARCONTROLLER_Error_ToString(error));
    }
  }

  if (!failed) {
    // wait state update update
    ARSAL_Sem_Wait(&(stateSem));

    deviceState = ARCONTROLLER_Device_GetState(deviceController, &error);

    if ((error != ARCONTROLLER_OK) ||
        (deviceState != ARCONTROLLER_DEVICE_STATE_RUNNING)) {
      failed = 1;
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- deviceState :%d", deviceState);
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s",
                  ARCONTROLLER_Error_ToString(error));
    }
  }
  // send the command that tells to the Bebop to begin its streaming
  if (!failed) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- send StreamingVideoEnable ... ");
    error = deviceController->aRDrone3->sendMediaStreamingVideoEnable(
        deviceController->aRDrone3, 1);
    if (error != ARCONTROLLER_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s",
                  ARCONTROLLER_Error_ToString(error));
      failed = 1;
    }
  }

  currentFrameMat = Mat(heightFrame, widthFrame, CV_8UC3);

  avcodec_register_all();
  codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  ctx = avcodec_alloc_context3(codec);
  frame = av_frame_alloc();
  av_init_packet(&packet);

  ctx->codec_id = AV_CODEC_ID_H264;
  ctx->codec_type = AVMEDIA_TYPE_VIDEO;
  ctx->bit_rate = 20000;
  ctx->framerate = AVRational{1, 25};
  ctx->width = widthFrame;
  ctx->height = heightFrame;
  ctx->pix_fmt = AV_PIX_FMT_YUV420P;
  ctx->gop_size = 30;

  codec->capabilities |= CODEC_CAP_FRAME_THREADS;
  av_opt_set(ctx->priv_data, "preset", "ultrafast", 0);
  framebgr = av_frame_alloc();
  int bytes = avpicture_get_size(AV_PIX_FMT_BGR24, widthFrame, heightFrame);
  uint8_t* buffer = (uint8_t*)av_malloc(bytes * sizeof(uint8_t));

  avpicture_fill((AVPicture*)framebgr, buffer, AV_PIX_FMT_BGR24, widthFrame,
                 heightFrame);
  int error = avcodec_open2(ctx, codec, NULL);

  if (error < 0) {
    cout << "ERROR opening codec " << error << endl;
    exit(1);
  }
  
 deviceController->aRDrone3->sendSpeedSettingsMaxRotationSpeed(
      deviceController->aRDrone3, 90);
  deviceController->aRDrone3->sendSpeedSettingsMaxVerticalSpeed(
      deviceController->aRDrone3, 1);
  deviceController->aRDrone3->sendPilotingSettingsMaxTilt(
      deviceController->aRDrone3, 8);
  deviceController->aRDrone3->sendSpeedSettingsMaxPitchRollRotationSpeed(
      deviceController->aRDrone3, 100);
  deviceController->aRDrone3->sendNetworkSettingsWifiSelection(
      deviceController->aRDrone3,  ARCOMMANDS_ARDRONE3_NETWORKSETTINGS_WIFISELECTION_TYPE_MANUAL, ARCOMMANDS_ARDRONE3_NETWORKSETTINGS_WIFISELECTION_BAND_2_4GHZ , 2);
 }

ARCONTROLLER_Frame_t BebopDrone::getCurrentFrame() { return currentFrame; }

Mat BebopDrone::getFrameAsMat() { return currentFrameMat; }

int BebopDrone::getYaw() { return yaw; }

int BebopDrone::getPitch() { return pitch; }

int BebopDrone::getRoll() { return roll; }

int BebopDrone::getBatteryLevel() { return batteryLevel; }

void BebopDrone::takeoff() {
  if (deviceController != NULL) {
    error = deviceController->aRDrone3->sendPilotingTakeOff(
        deviceController->aRDrone3);
    error = deviceController->aRDrone3->setPilotingPCMDFlag(
        deviceController->aRDrone3, 0);
  }
}

void BebopDrone::hover() {
  if (deviceController != NULL) {
    error = deviceController->aRDrone3->setPilotingPCMD(
        deviceController->aRDrone3, 0, 0, 0, 0, 0, 0);
    yaw = 0;
    pitch = 0;
    roll = 0;
  }
}

void BebopDrone::land() {
  if (deviceController != NULL) {
    error = deviceController->aRDrone3->sendPilotingLanding(
        deviceController->aRDrone3);
  }
}

void BebopDrone::emergencyStop() {
  if (deviceController != NULL) {
    error = deviceController->aRDrone3->sendPilotingEmergency(
        deviceController->aRDrone3);
  }
}

void BebopDrone::setYaw(int amount) { setYawRollPitchVSpeed(amount, 0, 0, 0); }

void BebopDrone::setRoll(int amount) { setYawRollPitchVSpeed(0, amount, 0, 0); }

void BebopDrone::setPitch(int amount) {
  setYawRollPitchVSpeed(0, 0, amount, 0);
}

void BebopDrone::setVerticalSpeed(int amount) {
  setYawRollPitchVSpeed(0, 0, 0, amount);
}

void BebopDrone::setYawRollPitchVSpeed(int yawAmount, int rollAmount,
                                       int pitchAmount, int verticalSpeed) {
  yaw = checkAskedOutput(yawAmount);
  roll = checkAskedOutput(rollAmount);
  pitch = checkAskedOutput(pitchAmount);
  vertical = checkAskedOutput(verticalSpeed);

  // cout << yaw << " " << roll << " " << pitch << endl;

  if (deviceController != NULL) {
    error = deviceController->aRDrone3->setPilotingPCMDYaw(
        deviceController->aRDrone3, yaw);

    error = deviceController->aRDrone3->setPilotingPCMDRoll(
        deviceController->aRDrone3, roll);

    error = deviceController->aRDrone3->setPilotingPCMDPitch(
        deviceController->aRDrone3, pitch);

    error = deviceController->aRDrone3->setPilotingPCMDGaz(
        deviceController->aRDrone3, vertical);

    error = deviceController->aRDrone3->setPilotingPCMDFlag(
        deviceController->aRDrone3, 1);
  }
}

#ifdef THREADED_TIMED_MOVEMENT

void BebopDrone::moveTimed(DroneMoveDirection direction, double seconds,
                           int movementAmount) {
  if (!currentlyMovingTimed) {
    currentlyMovingTimed = true;
    if (waitAndHoverThread.joinable()) {
      waitAndHoverThread.join();
    }

    switch (direction) {
      case DroneMoveDirection::Sideways:
        setYawRollPitchVSpeed(0.0, movementAmount, 0.0, 0.0);
        break;

      case DroneMoveDirection::Longitudinal:
        setYawRollPitchVSpeed(0.0, 0.0, movementAmount, 0.0);
        break;

      default:
        break;
    }
    waitAndHoverThread = thread(&BebopDrone::waitAndHover, this, seconds);
  }
}

bool BebopDrone::isMovingTimed() { return currentlyMovingTimed; }

#endif

// called when the state of the device controller has changed
void BebopDrone::stateChanged(eARCONTROLLER_DEVICE_STATE newState,
                              eARCONTROLLER_ERROR error, void* customData) {
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - stateChanged newState: %d .....",
              newState);

  switch (newState) {
    case ARCONTROLLER_DEVICE_STATE_STOPPED:
      ARSAL_Sem_Post(&(stateSem));
      break;

    case ARCONTROLLER_DEVICE_STATE_RUNNING:
      ARSAL_Sem_Post(&(stateSem));
      break;

    default:
      break;
  }
}
void BebopDrone::commandReceived(
    eARCONTROLLER_DICTIONARY_KEY commandKey,
    ARCONTROLLER_DICTIONARY_ELEMENT_t* elementDictionary, void* customData) {
  ARCONTROLLER_Device_t* deviceController = (ARCONTROLLER_Device_t*)customData;

  if (deviceController == NULL) return;

  // if the command received is a battery state changed
  switch (commandKey) {
    case ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED:
      cmdBatteryStateChangedRcv(deviceController, elementDictionary);
      break;
    case ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED:
      cmdSensorStateListChangedRcv(deviceController, elementDictionary);
      break;
    default:
      break;
  }
}

void BebopDrone::cmdBatteryStateChangedRcv(
    ARCONTROLLER_Device_t* deviceController,
    ARCONTROLLER_DICTIONARY_ELEMENT_t* elementDictionary) {
  ARCONTROLLER_DICTIONARY_ARG_t* arg = NULL;
  ARCONTROLLER_DICTIONARY_ELEMENT_t* singleElement = NULL;

  if (elementDictionary == NULL) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "elements is NULL");
    return;
  }

  // get the command received in the device controller
  HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY,
                singleElement);

  if (singleElement == NULL) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "singleElement is NULL");
    return;
  }

  // get the value
  HASH_FIND_STR(
      singleElement->arguments,
      ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT,
      arg);

  if (arg == NULL) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg is NULL");
    return;
  }
  /// TODO IMPLEMENT THIS
  // // update UI
  // batteryStateChanged(arg->value.U8);
  batteryLevel = arg->value.U8;
}

void BebopDrone::cmdSensorStateListChangedRcv(
    ARCONTROLLER_Device_t* deviceController,
    ARCONTROLLER_DICTIONARY_ELEMENT_t* elementDictionary) {
  ARCONTROLLER_DICTIONARY_ARG_t* arg = NULL;
  ARCONTROLLER_DICTIONARY_ELEMENT_t* dictElement = NULL;
  ARCONTROLLER_DICTIONARY_ELEMENT_t* dictTmp = NULL;

  eARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME
      sensorName =
          ARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME_MAX;
  int sensorState = 0;

  if (elementDictionary == NULL) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "elements is NULL");
    return;
  }

  // get the command received in the device controller
  HASH_ITER(hh, elementDictionary, dictElement, dictTmp) {
    // get the Name
    HASH_FIND_STR(
        dictElement->arguments,
        ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME,
        arg);
    if (arg != NULL) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg sensorName is NULL");
      continue;
    }

    sensorName =
        (eARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME)
            arg->value.I32;

    // get the state
    HASH_FIND_STR(
        dictElement->arguments,
        ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORSTATE,
        arg);
    if (arg == NULL) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg sensorState is NULL");
      continue;
    }

    sensorState = arg->value.U8;
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "sensorName %d ; sensorState: %d",
                sensorName, sensorState);
  }
}

// called when a streaming frame has been received
eARCONTROLLER_ERROR BebopDrone::didReceiveFrame(
    ARCONTROLLER_Frame_t* frameDrone, void* customData) {
  try {
    if (frame != NULL) {
      currentFrame = *frameDrone;
    }

    packet.data = frameDrone->data;
    packet.size = frameDrone->used;

    avcodec_send_packet(ctx, &packet);
    int len = avcodec_receive_frame(ctx, frame);

    if (len == 0 && (frame->data != NULL && frame->linesize != NULL)) {
      SwsContext* convert_ctx = sws_getContext(
          widthFrame, heightFrame, AV_PIX_FMT_YUV420P, widthFrame, heightFrame,
          AV_PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL, NULL);
      sws_scale(convert_ctx, frame->data, frame->linesize, 0, heightFrame,
                framebgr->data, framebgr->linesize);
      currentFrameMat = Mat(heightFrame, widthFrame, CV_8UC3, framebgr->data[0],
                            framebgr->linesize[0]);
    }
  } catch (Exception e) {
    cout << e.what() << endl;
  }

  return ARCONTROLLER_OK;
}

eARCONTROLLER_ERROR BebopDrone::decoderConfig(ARCONTROLLER_Stream_Codec_t codec,
                                              void* customData) {
  printf("CODEC= %i\n", codec.type);
  ARCONTROLLER_Stream_CodecH264_t h264Codec = codec.parameters.h264parameters;

  int got_picture;

  packet.data = h264Codec.spsBuffer;
  packet.size = h264Codec.spsSize;
  avcodec_decode_video2(ctx, frame, &got_picture, &packet);
  packet.data = h264Codec.ppsBuffer;
  packet.size = h264Codec.ppsSize;
  avcodec_decode_video2(ctx, frame, &got_picture, &packet);

  return ARCONTROLLER_OK;
}

double BebopDrone::checkAskedOutput(double uncheckedAmount) {
  double amount = uncheckedAmount > 100 ? 100 : uncheckedAmount;
  amount = uncheckedAmount < -100 ? -100 : uncheckedAmount;
  return amount;
}

#ifdef THREADED_TIMED_MOVEMENT
void BebopDrone::waitAndHover(double seconds) {
  high_resolution_clock::time_point startTime = high_resolution_clock::now();
  high_resolution_clock::time_point currentTime = high_resolution_clock::now();
  duration<double, std::milli> time_span = currentTime - startTime;
  while (time_span.count() < fabs(seconds) * 1000) {
    time_span = currentTime - startTime;
    currentTime = high_resolution_clock::now();

    setYawRollPitch(yaw, roll, pitch);
  }

  hover();
  currentlyMovingTimed = false;
}
#endif

BebopDrone::~BebopDrone() {
  land();
  // we are here because of a disconnection or user has quit IHM, so safely
  // delete everything
  if (deviceController != NULL) {
    deviceState = ARCONTROLLER_Device_GetState(deviceController, &error);
    if ((error == ARCONTROLLER_OK) &&
        (deviceState != ARCONTROLLER_DEVICE_STATE_STOPPED)) {
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Disconnecting ...");

      error = ARCONTROLLER_Device_Stop(deviceController);

      if (error == ARCONTROLLER_OK) {
        // wait state update update
        ARSAL_Sem_Wait(&(stateSem));
      }
    }

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "ARCONTROLLER_Device_Delete ...");
    ARCONTROLLER_Device_Delete(&deviceController);
  }

  ARSAL_Sem_Destroy(&(stateSem));

  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- END --");

  avcodec_free_context(&ctx);
  avcodec_close(ctx);
  av_frame_free(&frame);
  av_frame_free(&framebgr);

  av_packet_unref(&packet);

#ifdef THREADED_TIMED_MOVEMENT
  if (waitAndHoverThread.joinable()) {
    waitAndHoverThread.join();
  }
#endif
}
