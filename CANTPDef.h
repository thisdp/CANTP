// CANTP协议定义
/*
如果设备为 断开连接 状态，则等待一段时间后主动发起 连接请求指令，并且进入 正在连接 状态
如果服务端发现总线上有 连接请求，往总线上发送 允许连接 数据包， 并且附带服务器的CANConfig，
随后进入UniqueID仲裁，设备向服务端发送1字节的UniqueID长度、1字节随机生成的UniqueID、然后是N字节的UniqueID （STM32中UniqueID为12个字节，在ESP32中UniqueID是6字节的MAC地址）
在服务端开始处理连接请求途中，不会响应其他 连接请求指令
未通过仲裁的设备将会恢复 断开连接 状态，通过仲裁的设备，会向服务端发送 最大传输单元 和 CAN总线类型（CAN2.0B/CANFD），CANFD的速率协商会在建立连接之后处理
然后会向服务端发送一个 期待使用的CANID，
如果这个CANID不为0，则服务端会在列表中查找这个ID是否被占用，如果该ID被占用，则会向设备发送回一个随机分配的CANID，设备可以选择接受/不接收这个ID，不接受ID则会发送 断开连接指令，接受的话就会使用这个ID并且发送 连接指令
如果这个CANID为0，则服务器会先根据UniqueID在保存的设备列表里查找对应的CANID，如果未检索到相应的CANID则动态分配CANID
如果一切顺利，则建立连接，并发送 连接完成
*/
#pragma once

#include <Arduino.h>
#include <vector>
#include <cstring>
#include "BeaoUtils.h"

// 定义CAN总线标准或CAN FD
#define USECAN CANSTD
#if USECAN == CANSTD
#define CANFrameMaxLength 8
#elif USECAN == CANFD
#define CANFrameMaxLength 64
#endif

#define CANDataFrame 0
#define CANRemoteFrame 1

// 计算数据部分的最大长度
#define CANFrameDataLength ((uint8_t)(CANFrameMaxLength - sizeof(uint16_t)))
#define CANFrameDataLengthMin ((uint8_t)(8 - sizeof(uint16_t)))

enum CANTPDataPackType : uint8_t {
    CANTP_CONNECTION_LOST = 0,
    CANTP_CONNECTION_DONE = 1,  //8. *1
    CANTP_ASSIGNED_ID = 2,      //6. *1
    CANTP_CONNECT_ALLOW = 3,    //2. *1
    CANTP_SHORT_DATA = 4,
    CANTP_DATA_HEAD = 5,
    CANTP_DATA = 6,
};

enum CANTPRemotePackType : uint8_t {
    CANTP_HEARTBEAT_SERVER = 0,
    CANTP_HEARTBEAT_CLIENT = 1,
    CANTP_DISCONNECT = 2,      //7. *1
    CANTP_CONNECT = 3,         //7. *1
    CANTP_EXPECTING_CAN_ID = 4,//5. *1
    CANTP_CONFIGURE = 5,       //4. *1
    CANTP_UID_ARBITRATION = 6, //3. *N+2 , 12+2 STM32;
    CANTP_CONNECT_REQUEST = 7, //1. *1
};

enum class CANTPConnState : uint8_t {
    DISCONNECTED,
    CONNECTED,
    ASKINGCONNECT,
    CONNECTING,
    ARBITRATING_UID,
    CONFIGURING,
    EXPECTING_CAN_ID,
};

namespace CANType {
    constexpr uint8_t CAN20B = 0;
    constexpr uint8_t CANFD = 1;
    constexpr uint8_t CANXL = 2;
};

enum CANFuncCode {  

};

#pragma pack(push)
#pragma pack(1)
class CANTPConfig {
public:
    union{
        struct{
            uint8_t mtuCode : 4;
            uint8_t canType : 4;
        };
        uint8_t config;
    };
    CANTPConfig(uint8_t argMtuCode, uint8_t argCanType);
    CANTPConfig();
    void setConfig(uint8_t argMtuCode, uint8_t argCanType);
    uint8_t getConfig() const;
    void setCANType(uint8_t argCanType);
    uint8_t getCANType() const;
    void setMtuCode(uint8_t argMtuCode);
    uint8_t getMtuCode() const;
    uint16_t setMtuSize(uint16_t argMtuSize);
    uint16_t getMtuSize() const;
};

class CANTPFrameID {
public:
    union{
        struct{
            uint32_t extIdentifier  : 18;
            uint32_t identifier     : 8;
            uint32_t packType       : 3;
            uint32_t reserved       : 3;
        };
        struct{
            uint32_t extID      : 18;
            uint32_t stdID      : 11;
            uint32_t rsvID      : 3;
        };
        uint32_t frameIdentifier;
    };

    CANTPFrameID();
    CANTPFrameID(uint8_t id, uint8_t pType, uint32_t extID = 0);
    void setIdentifier(uint32_t standardID, uint32_t extensiveID);
    uint32_t getStdIdentifier();
    uint32_t getExtIdentifier();
    uint32_t getIdentifier();
    inline void setPackType(uint8_t pType) { packType = pType; }
    inline void setDeviceID(uint8_t id) { identifier = id; }
};

class CANTPFrameData {
public:
    uint8_t totalFrames;
    uint8_t frameNumber;
    uint8_t data[CANFrameDataLength];
};

class CANTPFrame {
public:
    CANTPFrame(CANTPFrameID fID, CANTPFrameData fData, uint8_t length);
    CANTPFrameID id;
    CANTPFrameData data;
    uint8_t dataLength;
};


/********************CANTP帧格式********************/
/*
完整传输: Head帧 > Data帧1 > Data帧2 > ... > Data帧N
Head帧:
    2Bytes 功能标识符
    2Bytes 起始地址
    2Bytes 数据总长度
    2Bytes 数据包总数
Data帧:
    2Bytes 数据包序号
    (mtuSize-2)Bytes 数据
*/
class CANTPShortPack {
private:
    uint8_t data[CANFrameMaxLength - 1];
public:
    void setData(uint8_t *pData, uint8_t len);
    uint8_t *getData();
};

class CANTPPackHead {
private:
    uint16_t totalLength;
    uint16_t packCount;
public:
    CANTPPackHead(uint16_t dataLength = 0, uint16_t packCount = 0);
    void setTotalLength(uint16_t length);
    uint16_t getTotalLength() const;
    void setPackCount(uint16_t packCount);
    uint16_t getPackCount() const;
};

class CANTPPackData {
private:
    uint16_t packIndex;
    uint8_t data[CANFrameDataLength];
public:
    CANTPPackData(uint16_t packIndex = 0, uint8_t *data = nullptr);
    void setPackIndex(uint16_t packIndex);
    uint16_t getPackIndex();
    void setData(uint8_t *data, uint16_t size);
    uint8_t *getData();
};

#pragma pack(pop)

extern Logger globalCANTPLogger;
bool setCANTPDebugStream(Stream &stream);
