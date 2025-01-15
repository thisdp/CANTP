#pragma once

/*CANTP的UniqueID组成
1Byte:  UniqueID总长度N
1Byte:  0~255随机数
NBytes: UniqueID数据
*/
#include <string.h>
using namespace std;

class CANTPUniqueID {
public:
    CANTPUniqueID(uint8_t len = 0) {
        memset(this, 0, sizeof(CANTPUniqueID));
        uniqueIDLength = len; //UniqueID数据总长度
    }
    CANTPUniqueID(uint8_t* uniqueID, uint8_t len){
        uniqueIDLength = len;
        setData(uniqueID);
        uniqueIDRandom = 0;
    }
    inline void setLength(uint8_t len) { uniqueIDLength = len;}
    inline uint8_t getLength() { return uniqueIDLength; }
    inline void setData(uint8_t* uID) { memcpy(uniqueIDData, uID, uniqueIDLength-2); }
    inline uint8_t *getData() { return uniqueIDData; }
    inline uint8_t getDataLength() { return (uniqueIDLength < 2)? 0 : uniqueIDLength - 2; }
    inline void setRaw(uint8_t index, uint8_t raw) { uniqueID[index] = raw; }
    inline uint8_t getRaw(uint8_t index) { return (index < uniqueIDLength) ? uniqueID[index] : 0; }
    inline void clear() { memset(this, 0, sizeof(CANTPUniqueID)); }
    inline uint8_t getRandomID() { return uniqueIDRandom; }
    inline void setRandomID(uint8_t randID) { uniqueIDRandom = randID; }
    bool operator==(const CANTPUniqueID& other) const { // Random ID仅用作CAN总线仲裁，不用作UniqueID比较
        return uniqueIDLength == other.uniqueIDLength && (memcmp(uniqueIDData, other.uniqueIDData, uniqueIDLength-2) == 0);
    }
    bool operator!=(const CANTPUniqueID& other) const { return !(*this == other); }
private:
    union {
        struct {
            uint8_t uniqueIDLength;     // 总长度
            uint8_t uniqueIDRandom;     // 随机数
            uint8_t uniqueIDData[30];   // UniqueID数据内容
        };
        uint8_t uniqueID[32]; // 存储UniqueID，最大支持32字节
    };
    //STM32为12位
};
