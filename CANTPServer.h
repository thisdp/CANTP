#pragma once

#include "BeaoUtils.h"
#include "CANTPDef.h"
#include "CANTPMsgCodec.h"
#include "CANTPUniqueID.h"
#include "CANTPDevice.h"
#include <iostream>
#include <string.h>
#include <unordered_map>
#include <Arduino.h>
using namespace std;

class CANTPServer;
class CANTPServerDevice : public CANTPMsgCodec {
private:
    char deviceName[32];
    CANTPUniqueID uniqueID;
    CANTPConfig canConf;

    uint8_t uniqueIDFragmentAppendLength;
    CANTPConnState connection;
    CANTPServer* server;
    MSTimer clientSyncTimer;

public:
    CANTPServerDevice(CANTPServer *attachedServer);
    
    // Copy assignment operator and copy constructor
    CANTPServerDevice& operator=(const CANTPServerDevice& other);
    CANTPServerDevice(const CANTPServerDevice& other);

    void begin();
    inline void setCANTPConfig(CANTPConfig aCANConf) { canConf = aCANConf; }
    inline CANTPConfig getCANTPConfig() { return canConf; }
    inline void setConnectionState(CANTPConnState newState) { connection = newState; }
    inline CANTPConnState getConnectionState() { return connection; }
    inline void addUniqueIDFragment(uint8_t uidFrag) { uniqueID.setRaw(uniqueIDFragmentAppendLength++, uidFrag); }
    inline CANTPUniqueID &getUniqueID() { return uniqueID; }
    inline void setUniqueID(CANTPUniqueID &uid) { uniqueID = uid; }
    inline void resetClientSyncTimer() { clientSyncTimer.start(); }
    inline void setDeviceName(const char* devName) {
        size_t len = strlen(devName);
        if (len >= sizeof(deviceName)) len = sizeof(deviceName) - 1;
        strncpy(deviceName, devName, len);
        deviceName[len] = '\0';
    }
    inline char *getDeviceName() { return deviceName; }
    inline bool isDesynced() { return clientSyncTimer.checkTimedOut(); }

    void onReceived();

    // 发送相关方法可以在这里声明，但因为没有具体的实现代码，我们暂不处理它们
};
/******************************服务端****************************/
class CANTPServer {
private:
    bool isNewDeviceConnecting;
    uint32_t newDeviceConnectingTick;
    std::unordered_map<uint8_t, CANTPServerDevice> deviceList;

    #define ConnectingDevice deviceList.at(0)
    
    CANTPServerDevice *currentDevice;
    uint8_t staticDevicesCount;
    HardwareCAN *hwCAN;
    CANMessage rxMsg;
    CANMessage txMsg;
    MSTimer globalHeartBeatTimer;

    CANTPConfig canConf;

public:
    uint32_t receiveTimes;
    uint32_t remoteReceiveTimes;
    uint32_t dataReceiveTimes;
    uint32_t head;
    uint32_t chead;

    CANTPServer(HardwareCAN& canPhy, uint8_t staticDevices);

    void begin(uint8_t maxMtuSize = 8, uint8_t canType = CANType::CAN20B);
    void loadDeviceList();
    void loadDevice(uint8_t devID);
    void saveDeviceList();
    void saveDevice(uint8_t devID);
    void onDeviceConnected(CANTPServerDevice &dev);
    void onDeviceDisconnected(CANTPServerDevice &dev);
    uint8_t searchProperSpace(uint8_t expectedID = 0);
    inline bool isNewDeviceConnectTimedOut() { return isNewDeviceConnecting && (millis() - newDeviceConnectingTick >= 2000); }
    inline void resetNewDeviceConnectTick() { newDeviceConnectingTick = millis(); }
    void onMessageReceived();
    void update();
    bool verifyDeviceOnline(uint8_t devID);
};