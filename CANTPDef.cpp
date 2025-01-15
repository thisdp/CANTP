#include "CANTPDef.h"
Logger globalCANTPLogger;

// 定义最小和最大MTU大小
constexpr uint16_t MIN_MTU = 8;
constexpr uint16_t MAX_MTU = 2048;
bool setCANTPDebugStream(Stream &stream){
    globalCANTPLogger.setStream(stream);
    return true;
}

CANTPConfig::CANTPConfig(uint8_t argMtuCode, uint8_t argCanType) : mtuCode(argMtuCode), canType(argCanType) {}
CANTPConfig::CANTPConfig() : mtuCode(0), canType(0) {}

void CANTPConfig::setConfig(uint8_t argMtuCode, uint8_t argCanType) {
    setMtuCode(argMtuCode);
    setCANType(argCanType);
}

uint8_t CANTPConfig::getConfig() const {
    return config;
}

void CANTPConfig::setCANType(uint8_t argCanType){
    switch(argCanType){
    case CANType::CANFD:
    case CANType::CANXL:
        canType = argCanType;
    break;
    default:
        canType = CANType::CAN20B;
    break;
    }
}

uint8_t CANTPConfig::getCANType() const{
    return canType;
}

uint8_t CANTPConfig::getMtuCode() const{
    return mtuCode;
}
void CANTPConfig::setMtuCode(uint8_t argMtuCode){
    mtuCode = argMtuCode;
}

uint16_t CANTPConfig::getMtuSize() const{
    uint16_t mtuSize = 1 << (mtuCode + 3);
    if(mtuSize >= 2048) return 2048;
    return mtuSize;
}

uint16_t CANTPConfig::setMtuSize(uint16_t size){
    if (size <= MIN_MTU) {
        mtuCode = 0; // 假设mtuCode为0对应MTU=8
        return MIN_MTU;
    }
    if (size >= MAX_MTU) {
        mtuCode = 9; // 假设mtuCode为9对应MTU=2048
        return MAX_MTU;
    }
    // 查找比size大的最近的2的n次幂的值
    for (uint8_t i = 3; i < 12; ++i) { // 从2^3(8)到2^11(2048)
        uint16_t powerOfTwo = 1 << i;
        if (size <= powerOfTwo) {
            mtuCode = i - 3; // 计算mtuCode, 从0开始
            return powerOfTwo;
        }
    }
    // 如果输入在合法范围内但没有匹配到，返回最大值（理论上不应该到达这里）
    mtuCode = 9;
    return MAX_MTU;
}

CANTPFrameID::CANTPFrameID() {}

CANTPFrameID::CANTPFrameID(uint8_t pType, uint8_t id, uint32_t extID) 
    : packType(pType), identifier(id), extIdentifier(extID), reserved(0) {}

void CANTPFrameID::setIdentifier(uint32_t standardID, uint32_t extensiveID) {
    stdID = standardID;
    extID = extensiveID;
}

uint32_t CANTPFrameID::getStdIdentifier(){
    return stdID;
}
uint32_t CANTPFrameID::getExtIdentifier(){
    return extID;
}
uint32_t CANTPFrameID::getIdentifier(){
    return frameIdentifier;
}

CANTPFrame::CANTPFrame(CANTPFrameID fID, CANTPFrameData fData, uint8_t length) : id(fID), data(fData), dataLength(length) {}

void CANTPShortPack::setData(uint8_t *pData, uint8_t len) {
    memcpy(data, pData, len);
}

uint8_t* CANTPShortPack::getData() {
    return data;
}

CANTPPackHead::CANTPPackHead(uint16_t dataLength, uint16_t packCount) 
    : totalLength(dataLength), packCount(packCount) {}

void CANTPPackHead::setTotalLength(uint16_t length) {
    this->totalLength = length;
}

uint16_t CANTPPackHead::getTotalLength() const {
    return totalLength;
}

void CANTPPackHead::setPackCount(uint16_t packCount) {
    this->packCount = packCount;
}

uint16_t CANTPPackHead::getPackCount() const {
    return packCount;
}

CANTPPackData::CANTPPackData(uint16_t packIndex, uint8_t *data) 
    : packIndex(packIndex) {
    if(data != nullptr) memcpy(this->data, data, CANFrameDataLength);
}

void CANTPPackData::setPackIndex(uint16_t packIndex) {
    this->packIndex = packIndex;
}

uint16_t CANTPPackData::getPackIndex() {
    return this->packIndex;
}

void CANTPPackData::setData(uint8_t *data, uint16_t size){
    memcpy(this->data, data, size);
}

uint8_t* CANTPPackData::getData(){
    return data;
}