#pragma once

#include "BeaoUtils.h"
#include "Arduino.h"

#if defined(ESP32)
#include "driver/gpio.h"
#include "driver/twai.h"
#include "hal/twai_hal.h"
#else
#endif

//CAN Classic/FD的Length Code
extern uint8_t CANLengthCode[16];
//CANXL的Length Code为线性的, 从0开始到2047表示1到2048
/*CAN消息*/
#pragma pack(push)
#pragma pack(1)
class CANMessageBuffer {
public:
    uint16_t bufferLength;
    uint16_t usingLength;
    uint8_t *data;

    CANMessageBuffer();
    ~CANMessageBuffer();
    
    void resize(uint16_t size);
    void requestSpace(uint16_t size);
    void setData(uint8_t* argData, uint16_t argLength);
    uint8_t* getData();
    void clear();
    uint16_t getLength();
};

class CANMessage {
public:
#if defined(ESP32)
    uint32_t isExtendPack : 1;     //IDE
    uint32_t isRemotePack : 1;     //RTR
    uint32_t reserved1 : 30;

    uint32_t identifier : 11;    //STDID
    uint32_t extIdentifier : 18;    //EXTID
    uint32_t reserved2 : 3;

    uint8_t dataLength;             //DLC

    constexpr static size_t headerSize = sizeof(uint32_t)*2+sizeof(uint8_t);
#else
    uint32_t identifier;    //STDID
    uint32_t extIdentifier; //EXTID
    uint32_t isExtendPack;  //IDE
    uint32_t isRemotePack;  //RTR
    uint32_t dataLength;    //DLC
    
    constexpr static size_t headerSize = sizeof(uint32_t)*5;
#endif
    CANMessageBuffer data;
    uint8_t txStateHandle;

    CANMessage() : dataLength(0) {}
    uint16_t lengthCodeToLength(uint8_t dlc, uint16_t maxSize = 64);
    uint16_t lengthToLengthCode(uint16_t length, uint16_t maxSize = 64);
#if defined(ESP32)
    inline void readHeadFrom(twai_message_t* readHead) { memcpy(this, readHead, headerSize); }
    inline void writeHeadTo(twai_message_t* writeHead) { memcpy(writeHead, this,headerSize); }
    inline void readFrom(twai_message_t *twaiMsg) {
        readHeadFrom(twaiMsg);
        readDataFrom(twaiMsg->data,lengthCodeToLength(twaiMsg->data_length_code));
    }
    inline void writeTo(twai_message_t* twaiMsg) {
        writeHeadTo(twaiMsg);
        writeDataTo(twaiMsg->data,lengthCodeToLength(twaiMsg->data_length_code));
    }

    inline void setExtend(bool isExtend) { isExtendPack = isExtend; }
    inline void setRemote(bool isRemote) { isRemotePack = isRemote; }
#else
    inline void readHeadFrom(CAN_TxHeaderTypeDef* readHead) { memcpy(this, readHead, headerSize); }
    inline void readHeadFrom(CAN_RxHeaderTypeDef* readHead) { memcpy(this, readHead, headerSize); }
    inline void writeHeadTo(CAN_TxHeaderTypeDef* writeHead) { memcpy(writeHead, this, headerSize); }
    inline void writeHeadTo(CAN_RxHeaderTypeDef* writeHead) { memcpy(writeHead, this, headerSize); }
    inline void readFrom(CAN_TxHeaderTypeDef *msg, uint8_t *msgData) {
        readHeadFrom(msg);
        readDataFrom(msgData,lengthCodeToLength(msg->DLC));
    }
    inline void readFrom(CAN_RxHeaderTypeDef *msg, uint8_t *msgData) {
        readHeadFrom(msg);
        readDataFrom(msgData,lengthCodeToLength(msg->DLC));
    }
    inline void writeTo(CAN_TxHeaderTypeDef *msg, uint8_t *msgData) {
        writeHeadTo(msg);
        writeDataTo(msgData,lengthCodeToLength(msg->DLC));
    }
    inline void writeTo(CAN_RxHeaderTypeDef *msg, uint8_t *msgData) {
        writeHeadTo(msg);
        writeDataTo(msgData,lengthCodeToLength(msg->DLC));
    }
    inline void setExtend(bool isExtend) { isExtendPack = isExtend?CAN_ID_EXT:CAN_ID_STD; }
    inline void setRemote(bool isRemote) { isRemotePack = isRemote?CAN_RTR_REMOTE:CAN_RTR_DATA; }
#endif
    inline void readDataFrom(uint8_t* readData, uint16_t size) { data.setData(readData, size); }
    inline void writeDataTo(uint8_t* writeData, uint16_t size) { memcpy(writeData, data.getData(), min(size,data.getLength())); }
    inline uint8_t *getData() { return data.getData(); }
    inline void clear() { memset(this, 0, sizeof(CANMessage)); };
    inline void setStdIdentifier(uint32_t stdID) { identifier = stdID; }
    inline void setExtIdentifier(uint32_t extID) { extIdentifier = extID; }
    inline void setDataLength(uint32_t dataLen) { dataLength = lengthToLengthCode(dataLen); data.requestSpace(getDataLength()); }
    inline void setDataLengthCode(uint16_t dlc) { dataLength = dlc; data.requestSpace(getDataLength()); }
    inline bool isExtend() { return isExtendPack != 0; }
    inline bool isRemote() { return isRemotePack != 0; }
    inline uint32_t getIdentifier() { return identifier; }
    inline uint32_t getExtIdentifier() { return extIdentifier; }
    inline uint32_t getCompleteIdentifier() { return ((identifier & 0x7FF) << 18) + (extIdentifier & 0x3FFFF); }
    inline uint8_t getDataLengthCode() { return dataLength; }
    inline uint16_t getDataLength() { return lengthCodeToLength(dataLength); }
    inline CANMessage &operator=(CANMessage& other){
        if (this == &other) return *this; // 防止自我赋值
        memcpy(this, &other, headerSize);
        data.setData(other.data.getData(), other.data.getLength());
        return *this;
    }
};
#pragma pack(pop)

//同一时刻只传输一个数据
class HardwareCAN {
private:
    bool isInited;
    uint16_t maxDataLength;
    uint32_t baudrate;
    uint32_t baudrateData;
#if defined(ESP32)
    // ESP32-specific members if any
    //TWAICAN* can;
#else
    CAN_HandleTypeDef* can;
#endif
    DynamicFIFO<CANMessage> rxMessage;
    DynamicFIFO<CANMessage> txMessage;
    CANMessage txMessageEmergency;
    bool hasTxMessageEmergency;
    CANMessage rxTempMessage;
    CANMessage txTempMessage;

    CANMessage* txMsg;  //正在发送的数据
public:
#if defined(ESP32)
    //HardwareCAN(TWAICAN &argCan);
#else
    HardwareCAN(CAN_HandleTypeDef &argCan);
#endif
    inline uint8_t getPendingTXMessages() {
#if defined(ESP32)
        //return argCan->getQueuedTXMessage();
#else
        return HAL_CAN_GetTxMailboxesFreeLevel(can);
#endif
    }
    inline int available() { return rxMessage.length(); }
    inline int availableForWrite() { return txMessage.emptyLength(); }
    inline bool isTXEmpty() { return txMessage.isEmpty(); }
    inline bool isTXFull() { return txMessage.isFull(); }
    void begin(uint16_t maxLength, uint32_t baud = 500000, uint32_t fdBaud = 500000);
    bool receive(CANMessage &msg);
    bool send(uint32_t identifier, uint8_t* data, uint8_t length, bool isRemote = false, bool isExtend = false, uint32_t extIdentifier = 0);
    bool send(CANMessage &msg);
    bool abortSend();
    bool emergencySend(CANMessage &msg);
    bool hasEmergencyMessage();
    //bool isBusBlocked();    //总线阻塞
    void doReceive();
    void doSend();
    void update();
};