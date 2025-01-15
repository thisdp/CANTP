#include "CANTPMsgCodec.h"
CANTPMsgCodec::CANTPMsgCodec() {
    txData = nullptr;
    txDataSize = 0;
    txCounter = 0;
    rxData.reserve(16);  // 预留16个字节的空间
}
void CANTPMsgCodec::readRXMessage(CANMessage &rxMsg) {
    if (rxMsg.isRemote()) return;   // 只解析数据帧
    uint32_t canIdentifier = rxMsg.getCompleteIdentifier();
    uint8_t packType = ((CANTPFrameID*)&canIdentifier)->packType;
    uint8_t packIdentifier = ((CANTPFrameID*)&canIdentifier)->identifier;
    switch (packType) {   // 数据包解码
    case CANTP_SHORT_DATA:
        onReceived();
        break;
    case CANTP_DATA_HEAD: {
        if (rxMsg.getDataLength() != sizeof(CANTPPackHead)) return; // 长数据包头长度错误
        CANTPPackHead *head = (CANTPPackHead*)rxMsg.getData();    // 强制转换
        rxDataSize = head->getTotalLength();    // 数据总长度
        rxCount = head->getPackCount();         // 数据包数量
        rxData.clear();
        break;
    }
    case CANTP_DATA: {
        if (rxMsg.getDataLength() != sizeof(CANTPPackData)) return; // 长数据包头长度错误
        CANTPPackData *data = (CANTPPackData*)rxMsg.getData();    // 强制转换
        for (uint8_t i = 0; i < rxMsg.getDataLength(); i++) { // 遍历数据
            rxData.push_back(data->getData()[i]);
        }
        if (data->getPackIndex() == rxCount - 1) {
            onReceived();
        }
        break;
    }
    default:
        break;
    }
}

bool CANTPMsgCodec::getNextTXMessage(CANMessage &txMsg) {
    if (txDataSize == 0 || txData == nullptr) return false;    // 无效数据
    if (txDataSize <= mtuSize) {  // 使用短数据包传输
        CANTPFrameID id(deviceID, CANTP_SHORT_DATA, 0);
        txMsg.setDataLength(txDataSize);
        txMsg.readDataFrom(txData, txDataSize);
        txMsg.setRemote(false);
        txMsg.setExtend(false);
        txMsg.setStdIdentifier(id.getStdIdentifier());
        txData = nullptr;
        txDataSize = 0;
    } else {  // 使用长数据包传输
        if (txCounter == 0) { // 如果计数器=0，则发送头帧
            CANTPFrameID id(deviceID, CANTP_DATA_HEAD, 0);
            txMsg.setDataLength(sizeof(CANTPPackHead));
            CANTPPackHead *head = (CANTPPackHead*)txMsg.getData();
            head->setPackCount((txDataSize + (mtuSize - 2) - 1) / (mtuSize - 2)); // 向上取整
            head->setTotalLength(txDataSize);
            txMsg.setRemote(false);
            txMsg.setExtend(false);
            txMsg.setStdIdentifier(id.getStdIdentifier());
        } else {
            CANTPFrameID id(deviceID, CANTP_DATA, 0);
            txMsg.setDataLength(mtuSize - 2); // 每个数据帧的实际数据长度为MTU减去两个字节的序列号
            CANTPPackData *data = (CANTPPackData*)txMsg.getData();
            uint16_t packIndex = txCounter - 1;
            uint16_t dataStartIndex = packIndex * (mtuSize - 2);
            uint16_t dataEndIndex = min((int32_t)(dataStartIndex + (mtuSize - 2)), (int32_t)txDataSize);
            data->setPackIndex(packIndex);
            data->setData(&txData[dataStartIndex], dataEndIndex - dataStartIndex);  // 复制数据
            txMsg.setRemote(false);
            txMsg.setExtend(false);
            txMsg.setStdIdentifier(id.getStdIdentifier());
            if (dataEndIndex == txDataSize) { // 完成
                txData = nullptr;
                txDataSize = 0;
                txCounter = 0;
                return false;
            }
        }
        txCounter++;
    }
    return true;
}

void CANTPMsgCodec::startTransmission(uint8_t *data, uint16_t size) {
    if (size == 0 || data == nullptr) return;
    txDataSize = size;
    txData = data;
    txCounter = 0;
}