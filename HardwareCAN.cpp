#include "HardwareCAN.h"

uint8_t CANLengthCode[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
/***********************************CAN数据缓存区 */
CANMessageBuffer::CANMessageBuffer()
    : data(nullptr), bufferLength(0), usingLength(0) {}

CANMessageBuffer::~CANMessageBuffer() {
    if (data) delete[] data;
}

void CANMessageBuffer::resize(uint16_t size) {
    if (data) delete[] data;
    data = new uint8_t[size];
    bufferLength = size;
    usingLength = 0;
}

void CANMessageBuffer::requestSpace(uint16_t size) {
    if (data == nullptr || bufferLength != size)
        resize(size);
}

void CANMessageBuffer::setData(uint8_t* argData, uint16_t argLength) {
    requestSpace(argLength);
    memcpy(data, argData, argLength);
    usingLength = argLength;
}

uint8_t* CANMessageBuffer::getData() {
    return data;
}

void CANMessageBuffer::clear() {
    if (data) memset(data, 0, bufferLength);
    usingLength = 0;
}

uint16_t CANMessageBuffer::getLength() {
    if (!data) return 0;
    if (usingLength > bufferLength) return bufferLength;
    return usingLength;
}
/************************CAN Message实现 */
uint16_t CANMessage::lengthCodeToLength(uint8_t dlc, uint16_t maxSize){
    if(dlc > sizeof(CANLengthCode)/sizeof(CANLengthCode[0])) return maxSize;
    return CANLengthCode[dlc];
}
uint16_t CANMessage::lengthToLengthCode(uint16_t length, uint16_t maxSize){
    if(length <= 8) return length;
    //if(length <= 32) return (((length-8)+3)/4)+8;
    if(length <= 32) return (length-5)/4+8;
    if(length <= 48) return 48;
    return maxSize;
}
/************************Hardware CAN实现 */
// 构造函数实现
#if defined(ESP32)
//HardwareCAN::HardwareCAN(TWAICAN &argCan) : can(&argCan), rxMessage(32), txMessage(32), isInited(false) {}
#else
HardwareCAN::HardwareCAN(CAN_HandleTypeDef &argCan) : can(&argCan), rxMessage(32), txMessage(32), isInited(false), hasTxMessageEmergency(false), txMsg(0) {}
#endif

void HardwareCAN::begin(uint16_t maxLength, uint32_t baud, uint32_t baudData){
    isInited = true;
    rxTempMessage.data.requestSpace(maxLength);
    txTempMessage.data.requestSpace(maxLength);
    txMessageEmergency.data.requestSpace(maxLength);
    baudrate = baud;
    baudrateData = baudData;
}

bool HardwareCAN::receive(CANMessage &msg) {
    if (rxMessage.isEmpty()) return false;
    rxMessage.dequeue(msg);
    return true;
}

bool HardwareCAN::send(uint32_t identifier, uint8_t* data, uint8_t length, bool isRemote, bool isExtend, uint32_t extIdentifier) {
    if (!availableForWrite() || length > maxDataLength) return false;   //不允许超出最大
    txTempMessage.setStdIdentifier(identifier);
    txTempMessage.setExtIdentifier(extIdentifier);
    txTempMessage.setExtend(isExtend);
    txTempMessage.setRemote(isRemote);
    txTempMessage.setDataLength(length);
    txTempMessage.readDataFrom(data, length);
    txMessage.enqueue(txTempMessage);
    return true;
}

bool HardwareCAN::send(CANMessage &msg) {
    if (!availableForWrite()) return false;
    msg.txStateHandle = 0;  //待传送
    if (!txMessage.enqueue(msg)) return false;
    return true;
}

bool HardwareCAN::emergencySend(CANMessage &msg) {
    if (hasEmergencyMessage()) return false;
    hasTxMessageEmergency = true;
    txMessageEmergency = msg;
    return true;
}

bool HardwareCAN::hasEmergencyMessage(){
    return hasTxMessageEmergency;
}

/*bool HardwareCAN::isBusBlocked(){
}*/

bool HardwareCAN::abortSend() {    //取消发送
#if defined(ESP32)
// ESP32-specific implementation if any
#else
    HAL_CAN_AbortTxRequest(can,CAN_TX_MAILBOX0);
    HAL_CAN_AbortTxRequest(can,CAN_TX_MAILBOX1);
    HAL_CAN_AbortTxRequest(can,CAN_TX_MAILBOX2);
    txMsg->txStateHandle = 0;
#endif
}

void HardwareCAN::doReceive() {
#if defined(ESP32)
// ESP32-specific implementation if any
#else
    if (HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0) > 0) { // 检查是否有消息
        if (!rxMessage.isFull()) {
            CAN_RxHeaderTypeDef rxHead;
            HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &rxHead, rxTempMessage.getData());
            rxTempMessage.readHeadFrom(&rxHead);
            rxMessage.enqueue(rxTempMessage);
        }
    }
#endif
}
void HardwareCAN::doSend() {
#if defined(ESP32)
// ESP32-specific implementation if any
#else
    if(txMsg){  //如果有数据正在发送
        if(HAL_CAN_GetTxMailboxesFreeLevel(can) != 3) return; // 如果有数据还在传输
        switch(txMsg->txStateHandle){
        case 0: //等待发送
            txMsg->txStateHandle = 1;   //等待传送完成
            CAN_TxHeaderTypeDef txHead;
            memset(&txHead, 0, sizeof(txHead));
            txMsg->writeHeadTo(&txHead);
            uint32_t txUsingMailBox;
            HAL_CAN_AddTxMessage(can, &txHead, txMsg->getData(), &txUsingMailBox);  //开始传送
        break;
        case 1:
            txMsg->txStateHandle = 2;
            if(txMsg == &txMessageEmergency){   //如果是紧急包
                hasTxMessageEmergency = false;  //复位
            }else{  //常规包
                txMessage.dequeue();    //出列
            }
            txMsg = 0;
        break;
        }
    }else{ 
        if(hasTxMessageEmergency){ // 如果有紧急消息需要发送
            txMsg = &txMessageEmergency;
            txMsg->txStateHandle = 0;
        }else if(!txMessage.isEmpty()){ // 如果有常规消息需要发送
            if(txMessage.peek(txMsg,0)){
                txMsg->txStateHandle = 0;
            }
        }
    }
#endif
}

void HardwareCAN::update() {
    if(!isInited) return;
    doReceive();
    doSend();
}
