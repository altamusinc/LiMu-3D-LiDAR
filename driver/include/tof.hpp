#ifndef __TOF_H__
#define __TOF_H__

#include "frame.hpp"
#include "camera_info.hpp"
#include <boost/thread.hpp>

struct IPAddress {
  uint8_t ip0;
  uint8_t ip1;
  uint8_t ip2;
  uint8_t ip3;

  IPAddress(uint8_t a0, uint8_t a1, uint8_t a2, uint8_t a3) : ip0(a0), ip1(a1), ip2(a2), ip3(a3)
  {

  }
};

class ToF {
public:
  static ToF* tof320(const char* host, const char* port);

  virtual void stopStream() = 0;  
  virtual void streamDCS() = 0;
  virtual void streamGrayscale() = 0;
  virtual void streamDistance() = 0;
  virtual void streamDistanceAmplitude() = 0;
  virtual void setOffset(int16_t offset) = 0;
  virtual void setMinAmplitude(uint16_t minAmplitude) = 0;
  virtual void setBinning(const bool vertical, const bool horizontal) = 0;
  virtual void setRoi(const uint16_t x0, const uint16_t y0, const uint16_t x1, const uint16_t y1) = 0;
  virtual void setIntegrationTime(uint16_t, uint16_t, uint16_t, uint16_t) = 0;
  virtual void setHDRMode(uint8_t mode) = 0;
  virtual void setModulation(const uint8_t index, const uint8_t channel) = 0;
  virtual void setFilter(const bool medianFilter, const bool averageFilter, const uint16_t temporalFactor, const uint16_t temporalThreshold, const uint16_t edgeThreshold,
            const uint16_t temporalEdgeThresholdLow, const uint16_t temporalEdgeThresholdHigh, const uint16_t interferenceDetectionLimit, const bool interferenceDetectionUseLastValue) = 0;

  virtual void setPrecompute(bool precompute) = 0;

  virtual void setIPAddress(IPAddress ip, IPAddress netmask, IPAddress gateway) = 0;

  virtual void subscribeFrame(std::function<void (std::shared_ptr<Frame>)>) = 0;
  virtual void subscribeCameraInfo(std::function<void (std::shared_ptr<CameraInfo>)>) = 0;

  virtual void setLensType(int lensType) = 0;
  virtual void setLensCenter(int cx, int cy) = 0;

  virtual int getWidth() = 0;
  virtual int getHeight() = 0;
};

#endif
