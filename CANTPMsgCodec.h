#pragma once

#include "Arduino.h"
#include <vector>
#include "CANTPDef.h"
#include "HardwareCAN.h"
using namespace std;

class CANTPMsgCodec {  // 数据包编解码器
protected:
    // 接收缓存
    uint16_t rxCount;
    uint16_t rxDataSize;
    vector<uint8_t> rxData;
    // 发送缓存
    uint16_t txCounter;     // 发送计数器
    uint16_t txDataSize;    // 数据长度
    uint8_t *txData;        // 数据地址

    CANTPConfig canConf;
    uint16_t mtuSize;   // MTU大小
    uint8_t deviceID;   // 设备ID

public:
    CANTPMsgCodec();
    inline void setMTUSize(uint16_t mtu) { mtuSize = canConf.setMtuSize(mtu); }
    inline uint16_t getMTUSize(uint16_t mtu) const { return mtuSize; }
    inline void setCANType(uint8_t canType){ canConf.setCANType(canType); }
    inline uint8_t getCANType() const { return canConf.getCANType(); }
    void setDeviceID(uint8_t devID) { deviceID = devID; };
    uint8_t getDeviceID() const { return deviceID; }
    void readRXMessage(CANMessage &rxMsg);
    bool getNextTXMessage(CANMessage &txMsg);
    void startTransmission(uint8_t *data, uint16_t size);

    virtual void onReceived() = 0;

};
