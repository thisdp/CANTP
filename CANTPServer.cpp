#include "CANTPServer.h"
#include <iostream>
#include <string.h>
#include <Arduino.h>
using namespace std;

/********************服务端********************/
CANTPServer::CANTPServer(HardwareCAN& canPhy, uint8_t staticDevices) : globalHeartBeatTimer(2000) {
    staticDevicesCount = staticDevices;
    hwCAN = &canPhy;
    deviceList.emplace(0, CANTPServerDevice(this)); // 正在连接的设备
    currentDevice = &(deviceList.at(0));
    for (uint8_t i = 1; i <= staticDevicesCount; i++) deviceList.emplace(i, CANTPServerDevice(this));
}

// 初始化方法
void CANTPServer::begin(uint8_t maxMtuSize, uint8_t canType) {
    receiveTimes = 0;
    globalHeartBeatTimer.start();
    canConf.setMtuSize(maxMtuSize); // 8 Bytes
    canConf.setCANType(canType);
}

// 加载设备列表（暂未实现）
void CANTPServer::loadDeviceList() {}

// 加载特定设备（暂未实现）
void CANTPServer::loadDevice(uint8_t devID) {}

// 保存设备列表（暂未实现）
void CANTPServer::saveDeviceList() {}

// 保存特定设备（暂未实现）
void CANTPServer::saveDevice(uint8_t devID) {}

// 当设备连接时调用
void CANTPServer::onDeviceConnected(CANTPServerDevice &dev) {
    uint8_t devID = dev.getDeviceID();
    if (devID == 0) return; // devID不可能为0
    if (deviceList.count(devID) == 0) return;   // 该处不可能没有devID
    deviceList.at(devID) = dev;	// 复制
    deviceList.at(devID).setConnectionState(CANTPConnState::CONNECTED);	// 标记为在线
    if (devID <= staticDevicesCount) saveDevice(devID);
}

// 当设备断开连接时调用
void CANTPServer::onDeviceDisconnected(CANTPServerDevice &dev) {
    uint8_t devID = dev.getDeviceID();
    if (devID == 0) return; // devID不可能为0
    if (deviceList.count(devID) == 0) return;   // 该处不可能没有devID
    deviceList.at(devID).setConnectionState(CANTPConnState::DISCONNECTED);	// 标记为离线
}

// 寻找合适的空间分配给新设备
uint8_t CANTPServer::searchProperSpace(uint8_t expectedID) {
    if (expectedID != 0) {  // 如果指定了ID，则ID优先
        if (deviceList.count(expectedID) == 0) {  // 如果该处没有分配设备
            deviceList.emplace(expectedID, CANTPServerDevice(this));   // 分配ID
            return expectedID;
        }
        if (deviceList.at(expectedID).getConnectionState() == CANTPConnState::DISCONNECTED) {	// 如果设备已断开
            return expectedID;
        } else {	// 如果设备已经连接
            if (deviceList.at(expectedID).getUniqueID() == ConnectingDevice.getUniqueID()) {   // 如果UID一样，则可能是掉线了，重连
                return expectedID;
            } // 如果UID不一样，则可能是地址冲突，重新在动态设备区域内分配ID
        }
    } else {	// 如果未指定ID，则根据 UniqueID，优先搜索 staticDevice
        for (uint8_t i = 1; i <= staticDevicesCount; i++) { // 先扫描静态分配UniqueID一致的设备
            if (deviceList.count(i) == 0) {  // 如果该处没有分配设备
                deviceList.emplace(i, CANTPServerDevice(this));   // 分配ID
                return i;
            }
            if (deviceList.at(i).getUniqueID() == ConnectingDevice.getUniqueID())	// 如果找到已保存的ID
                return i;
        }
    }
    // 上述条件都不满足，则分配动态ID
    for (size_t i = staticDevicesCount + 1; i <= 255; i++) {	// 扫描动态区域内空闲ID
        if (deviceList.count(i) == 0 || deviceList.at(i).getConnectionState() == CANTPConnState::DISCONNECTED) {   // 如果该处没有创建对象或该处设备空闲
            return i;
        }
    }
    return 0;  // 没有空闲的位置
}

// 接收到消息时调用
void CANTPServer::onMessageReceived() {
    receiveTimes++;
    uint32_t canIdentifier = rxMsg.getCompleteIdentifier();
    uint8_t packType = ((CANTPFrameID*)&canIdentifier)->packType;
    uint8_t packIdentifier = ((CANTPFrameID*)&canIdentifier)->identifier;
    head = packType;
    chead = canIdentifier;
    if (rxMsg.isRemote()) {	// 远程帧
        switch (packType) {
        case CANTP_CONNECT_REQUEST:	// 当有设备请求连接
            if (!isNewDeviceConnecting || isNewDeviceConnectTimedOut()) { // 如果没有设备在连接，或上一个连接已经超时
                isNewDeviceConnecting = true;   // 开始处理新连接
                ConnectingDevice.begin();	// 立刻对正在连接的设备初始化
                ConnectingDevice.setConnectionState(CANTPConnState::CONNECTING);
                txMsg.clear();
                CANTPFrameID frameIdentifier(CANTP_CONNECT_ALLOW, canConf.getConfig());
                txMsg.setStdIdentifier(frameIdentifier.getStdIdentifier());
                txMsg.setDataLength(0);
                txMsg.setExtend(false); // 标准帧
                txMsg.setRemote(false); // 数据帧
                hwCAN->send(txMsg); // 发送接受连接回包
                resetNewDeviceConnectTick();
            } // 否则不回复接受设备连接请求
            break;
        case CANTP_UID_ARBITRATION: // 开始通过设备UniqueID仲裁
            if (isNewDeviceConnecting && !isNewDeviceConnectTimedOut()) { // 如果设备正在连接，并且没有超时
                ConnectingDevice.addUniqueIDFragment(packIdentifier);   // 记录Unique片段
                ConnectingDevice.setConnectionState(CANTPConnState::ARBITRATING_UID);
                resetNewDeviceConnectTick();
            }
            break;
        case CANTP_CONFIGURE:		// 接收设备的配置
            if (isNewDeviceConnecting && !isNewDeviceConnectTimedOut()) { // 如果设备正在连接，并且没有超时
                ConnectingDevice.setCANTPConfig(*(CANTPConfig*)&(packIdentifier));
                ConnectingDevice.setConnectionState(CANTPConnState::CONFIGURING);
                resetNewDeviceConnectTick();
            }
            break;
        case CANTP_EXPECTING_CAN_ID:   // 尝试使用期待的CAN ID
            if (isNewDeviceConnecting && !isNewDeviceConnectTimedOut()) { // 如果设备正在连接，并且没有超时
                uint8_t id = searchProperSpace(packIdentifier); // 0表示未分配到地址，如果期待的ID不等于分配到的ID，则表示ID已占用
                ConnectingDevice.setConnectionState(CANTPConnState::EXPECTING_CAN_ID);
                ConnectingDevice.setDeviceID(id);
                txMsg.clear();
                CANTPFrameID frameIdentifier(CANTP_ASSIGNED_ID, id);
                txMsg.setStdIdentifier(frameIdentifier.getStdIdentifier());
                txMsg.setDataLength(0);
                txMsg.setExtend(false); // 标准帧
                txMsg.setRemote(false); // 数据帧
                hwCAN->send(txMsg); // 发送回获取到的CAN ID
            }
            break;
        case CANTP_CONNECT:	   // 设备连接
            if (isNewDeviceConnecting && !isNewDeviceConnectTimedOut()) { // 如果设备正在连接，并且没有超时
                onDeviceConnected(ConnectingDevice);
                isNewDeviceConnecting = false;  // 连接完成
                txMsg.clear();
                CANTPFrameID frameIdentifier(CANTP_CONNECTION_DONE, ConnectingDevice.getDeviceID());
                txMsg.setStdIdentifier(frameIdentifier.getStdIdentifier());
                txMsg.setDataLength(0);
                txMsg.setExtend(false); // 标准帧
                txMsg.setRemote(false); // 数据帧
                hwCAN->send(txMsg); // 向指定设备发送Connect Done
            }
            break;
        case CANTP_DISCONNECT:   // 设备断开连接
            if (isNewDeviceConnecting && !isNewDeviceConnectTimedOut()) { // 如果设备正在连接，并且没有超时
                ConnectingDevice.setConnectionState(CANTPConnState::DISCONNECTED);
                isNewDeviceConnecting = false;  // 连接完成
            }
        case CANTP_HEARTBEAT_CLIENT:	   // 接收到客户端心跳包
            if (deviceList.count(packIdentifier) != 0 && deviceList.at(packIdentifier).getConnectionState() == CANTPConnState::CONNECTED) {	// 仅在线有效
                deviceList.at(packIdentifier).resetClientSyncTimer();
            }
            break;
        }
    } else {	// 数据帧
        switch (packType) {
        case CANTP_SHORT_DATA:  // 短数据包
        case CANTP_DATA_HEAD:   // 数据头
        case CANTP_DATA:	// 数据本体
            if(verifyDeviceOnline(packIdentifier)) deviceList.at(packIdentifier).readRXMessage(rxMsg);    //如果设备在线，则读取数据包
            break;
        }
    }
}

bool CANTPServer::verifyDeviceOnline(uint8_t devID){
    if (deviceList.count(devID) != 0) {
        if (deviceList.at(devID).getConnectionState() == CANTPConnState::CONNECTED) {	// 仅在线有效
            deviceList.at(devID).resetClientSyncTimer();	// 只要收到数据包就复位
            return true;
        } else {	// 如果接收到的数据来自于离线的设备
            txMsg.clear();
            CANTPFrameID frameIdentifier(CANTP_CONNECTION_LOST, devID);
            txMsg.setStdIdentifier(frameIdentifier.getStdIdentifier());
            txMsg.setDataLength(0);
            txMsg.setExtend(false); // 标准帧
            txMsg.setRemote(false); // 数据帧
            hwCAN->send(txMsg); // 向指定设备发送Connect Lost
            return false;
        }
    }
}

// 更新方法
void CANTPServer::update() {
    hwCAN->update();
    while (hwCAN->available()) {
        hwCAN->receive(rxMsg);
        onMessageReceived();
    }
    if (globalHeartBeatTimer.checkTimedOut()) {
        globalHeartBeatTimer.start();
        if(!hwCAN->hasEmergencyMessage()){
            txMsg.clear();
            CANTPFrameID frameIdentifier(CANTP_HEARTBEAT_SERVER, 0);
            txMsg.setStdIdentifier(frameIdentifier.getStdIdentifier());
            txMsg.setDataLength(0);
            txMsg.setExtend(false);
            txMsg.setRemote(true);
            hwCAN->emergencySend(txMsg);
        }
    }
    for (auto& kv : deviceList) {
        if (kv.first != 0) {
            if (kv.second.getConnectionState() == CANTPConnState::CONNECTED) {  // 如果已连接
                if (kv.second.isDesynced()) {   // 如果设备丢失连接过久
                    kv.second.setConnectionState(CANTPConnState::DISCONNECTED); // 标记为断开
                }
            }
        }
    }
}


/********************设备********************/
// 构造函数
CANTPServerDevice::CANTPServerDevice(CANTPServer *attachedServer)
    : server(attachedServer),
      clientSyncTimer(2000) {}

// 复制赋值运算符
CANTPServerDevice& CANTPServerDevice::operator=(const CANTPServerDevice& other) {
    if (this == &other) return *this; // 防止自我赋值
    memcpy(this, &other, sizeof(CANTPServerDevice));
    return *this;
}

// 拷贝构造函数
CANTPServerDevice::CANTPServerDevice(const CANTPServerDevice& other) {
    memcpy(this, &other, sizeof(CANTPServerDevice));
}

// 初始化方法
void CANTPServerDevice::begin() {
    connection = CANTPConnState::DISCONNECTED;
    deviceID = sizeof(CANTPServerDevice);
    canConf.setCANType(CANType::CAN20B);
    canConf.setMtuCode(0); // 8 Bytes
    mtuSize = canConf.getMtuSize();
    uniqueIDFragmentAppendLength = 0;
    memset(deviceName, 0, sizeof(deviceName));
}

void CANTPServerDevice::onReceived(){

}