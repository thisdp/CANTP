#pragma once

#include "Arduino.h"
/***CANTP设备类型 */
namespace CANTP_DeviceType {
    constexpr uint16_t Undefined = 0;
    constexpr uint16_t Input = 10;
};

/***内存 */
#define mAddr(x) ((uint8_t*)(&(x)))
class CANTPMemory{
public:
    uint16_t size;
    uint8_t *mem;
    CANTPMemory() : mem(0), size(0){}
    uint8_t *begin(uint16_t mSize){
        if(mem != nullptr) delete[] mem;
        if(mSize != 0){
            mem = new uint8_t[mSize];
        }
        return mem;
    }
    uint16_t getBlockAddress(uint8_t *m){ return (uint16_t)(m - this->mem); }
    uint8_t getUINT8(uint16_t address){ return *(uint8_t*)(mem + address); }
    uint16_t getUINT16(uint16_t address){ return *(uint16_t*)(mem + address); }
    uint32_t getUINT32(uint16_t address){ return *(uint32_t*)(mem + address); }
    int8_t getINT8(uint16_t address){ return *(int8_t*)(mem + address); }
    int16_t getINT16(uint16_t address){ return *(int16_t*)(mem + address); }
    int32_t getINT32(uint16_t address){ return *(int32_t*)(mem + address); }
    float getFloat(uint16_t address){ return *(float*)(mem + address); }
    double getDouble(uint16_t address){ return *(double*)(mem + address); }
    uint8_t *getBuffer(uint16_t address){ return mem+address; }
    void setUINT8(uint16_t address, uint8_t data){ *(uint8_t*)(mem + address) = data; }
    void setUINT16(uint16_t address, uint16_t data){ *(uint16_t*)(mem + address) = data; }
    void setUINT32(uint16_t address, uint32_t data){ *(uint32_t*)(mem + address) = data; }
    void setINT8(uint16_t address, int8_t data){ *(int8_t*)(mem + address) = data; }
    void setINT16(uint16_t address, int16_t data){ *(int16_t*)(mem + address) = data; }
    void setINT32(uint16_t address, int32_t data){ *(int32_t*)(mem + address) = data; }
    void setFloat(uint16_t address, float data){ *(float*)(mem + address) = data; }
    void setDouble(uint16_t address, double data){ *(double*)(mem + address) = data; }
    void setBuffer(uint16_t address, uint8_t *data, uint16_t size){ memcpy(mem + address, data, size); }
    ~CANTPMemory(){ if(mem != nullptr) delete[] mem; }
};

class CANTPDevice : public CANTPMemory{
public:
//设备基础数据
    uint16_t deviceType;
    CANTPDevice() : deviceType(CANTP_DeviceType::Undefined), CANTPMemory(){}
    void setDeviceType(uint16_t devType){
        deviceType = devType;
    }
    uint16_t getDeviceType(){
        return deviceType;
    }

};