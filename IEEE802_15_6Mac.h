#ifndef __IMPROVEDWBAN_IEEE802_15_6MAC_H
#define __IMPROVEDWBAN_IEEE802_15_6MAC_H

#include <omnetpp.h>
#include <queue>
#include <map>
#include <vector>
#include "../../messages/WBANPacket_m.h"
#include "../../phy/Radio.h"
#include "../../common/ErrorHandler.h"

using namespace omnetpp;
namespace improvedwban {
/**
 * @enum SelfMessageKind
 * @brief 自消息类型枚举，用于定时器和事件调度
 */
enum SelfMessageKind {
  // ================ 核心生命周期 ================
  SEND_BEACON,                                          // 发送信标帧
  EAP1_START, EAP1_END,                                 // 专用访问阶段1开始/结束
  RAP1_START, RAP1_END,                                 // 随机访问阶段1开始/结束
  TYPE1_POLLED_START, TYPE1_POLLED_END,                 // 类型1轮询访问阶段开始/结束
  EAP2_START, EAP2_END,                                 // 专用访问阶段2开始/结束
  RAP2_START, RAP2_END,                                 // 随机访问阶段2开始/结束
  TYPE2_POLLED_START, TYPE2_POLLED_END,                 // 类型2轮询访问阶段开始/结束
  CAP_START, CAP_END,                                   // 竞争访问阶段开始/结束
  TX_END,                                               // 传输结束

  // ================ CSMA/CA 机制 ================
  BACKOFF_SLOT_TICK,                                    // 退避时隙计时
  CCA_CHECK,                                            // CCA检测
  RETRANSMISSION_TIMEOUT,                               // 重传超时
  
  // ================ 时序管理 ================
  SLOT_END,                                             // 时隙结束
  SENSE_DURATION_END,                                   // 感知持续时间结束
  GUARD_TIME_END,                                       // 保护时间结束
                                             
  // ================ 轮询机制 ================
  POLL_NODE,                                            // 轮询节点
  POLL_RESPONSE_TIMEOUT,                                // Hub等待节点响应的超时
  EXECUTE_NEXT_POLL,                                    // Hub轮询下一个节点的触发器
};

/**
 * @enum SuperframePhase
 * @brief 超帧阶段枚举，定义了超帧中的不同访问阶段
 */
enum SuperframePhase {
    INACTIVE,        // 非活动阶段
    BEACON_PHASE,    // 信标阶段
    EAP1,            // 专用访问阶段1
    RAP1,            // 随机访问阶段1
    TYPE1_POLLED,    // 类型1轮询阶段
    EAP2,            // 专用访问阶段2
    RAP2,            // 随机访问阶段2
    TYPE2_POLLED,    // 类型2轮询阶段
    CAP              // 竞争访问阶段
};

class IEEE802_15_6Mac : public cSimpleModule
{
  public:
    // ================ 基础模块函数 ================
    IEEE802_15_6Mac();                          // 构造函数
    virtual ~IEEE802_15_6Mac();                 // 析构函数
    virtual void initialize() override;         // 初始化函数
    virtual void handleMessage(cMessage *msg) override;  // 消息处理函数
    virtual void finish() override;             // 结束函数
    
    // ================ Setter方法 ================
    virtual void setCurrentPhase(SuperframePhase phase) { currentPhase = phase; } // 设置当前超帧阶段
    virtual void setSuperframeStartTime(simtime_t time) { superframeStartTime = time; } // 设置超帧开始时间
    
    // ================ 核心收发与队列管理函数 ================
    virtual void handleUpperPacket(cMessage *msg);  // 处理上层包
    virtual void handleLowerPacket(cMessage *msg);  // 处理下层包
    virtual void sendUp(improvedwban::WBANPacket *pkt);            // 发送到上层
    virtual void sendDown(improvedwban::WBANPacket *pkt);          // 发送到下层
    virtual void enqueuePacket(improvedwban::WBANDataPacket *pkt);           // 包入队

    // ================ 超帧生命周期管理函数 ================
    virtual void startBeaconPeriod();                      // 启动信标周期
    virtual void startEAP1();                              // 启动EAP1阶段
    virtual void startRAP1();                              // 启动RAP1阶段
    virtual void startEAP2();                              // 启动EAP2阶段
    virtual void startRAP2();                              // 启动RAP2阶段
    virtual void startType1Polled();                       // 启动Type1轮询阶段
    virtual void startType2Polled();                       // 启动Type2轮询阶段
    virtual void startMCAP();                              // 启动MCAP阶段
    
    // ================ CSMA/CA核心机制函数 ================
    virtual void tryInitiateTransmission();                 // 尝试发起传输
    virtual void startBackoffProcess();                     // 启动退避过程
    virtual void processBackoffSlot();                      // 处理退避时隙
    virtual void performCCA();                              // 执行CCA检测
    virtual void handleCCAResult(bool channelIdle);         // 处理CCA结果
    virtual void startCarrierSensing();                    // 开始载波监听
    virtual void handleRetransmission(improvedwban::WBANDataPacket *pkt); // 处理重传
    virtual void sendFromQueue();                           // 从队列发送
    virtual void handleAck(improvedwban::WBANAckPacket *pkt);                // 处理ACK
    virtual bool canSendInCurrentPhase(int priority);         // 检查当前阶段是否可发送
    virtual simtime_t calculateTransmissionDuration(improvedwban::WBANPacket *pkt);  // 计算传输持续时间
    virtual simtime_t getNextSlotBoundary();                // 获取下一个时隙边界
    virtual double getCurrentRSSI();                        // 获取当前RSSI值
    virtual bool isChannelBusy();                           // 检查信道是否忙碌
    
    // ================ 轮询机制函数 ================
    virtual void sendPolledData();                          // 发送轮询数据
    virtual void executePolling();                          // 执行轮询
    
    // ================ 数据包处理函数 ================
    virtual void handleBeacon(improvedwban::WBANBeaconPacket *pkt);      // 处理信标包
    virtual void handleDataPacket(improvedwban::WBANDataPacket *pkt);    // 处理数据包
    virtual void handleManagementPacket(improvedwban::WBANManagementPacket *pkt); // 处理管理包
  
  protected:
    // ================ 核心收发与队列管理私有函数 ================
    void decrementBackoffCounter();                          // 递减退避计数器
    void deferTransmission();                               // 推迟传输

    // ================ 超帧生命周期管理私有函数 ================
    void endCurrentPhase();                                // 结束当前阶段
    void transitionToNextPhase();                          // 转换到下一阶段

    // ================ CSMA/CA 核心机制私有函数 ================
    void initializeCSMAParameters();                       // 初始化CSMA参数
    void resetBackoffCounter();                            // 重置退避计数器

    // ================ 轮询机制私有函数 ================
    void initializePollingList();                         // 初始化轮询列表
    void scheduleNextPoll();                               // 调度下一次轮询
    void handlePollResponse();                             // 处理轮询响应

    // ================ 辅助与工具函数 ================
    void updateStatistics();                               // 更新统计信息
    void updateThroughput();                               // 更新吞吐量统计
    void logPhaseTransition(SuperframePhase newPhase);     // 记录阶段转换

    // ================ 模块参数 ================
    int nodeId;                                         // 节点ID
    bool isHub;                                         // 是否是Hub节点
    int queueSize;                                      // 队列的最大长度
    simtime_t beaconPeriodLength;                       // 信标周期长度
    simtime_t allocationSlotLength;                     // 分配时隙长度  
    int CWminPriority[8];                               // 不同优先级的争用窗口参数
    int CWmaxPriority[8];
    double ccaThreshold;                                // CCA阈值 (dBm)
    simtime_t ccaDuration;                              // CCA检测持续时间
    int maxRetransmissions;                             // 最大重传次数
    double baseRetransmissionTimeout;                   // 基础重传超时时间 - 10ms
    
    // 超帧阶段百分比参数
    double eap1Percentage;                              // EAP1阶段百分比
    double rap1Percentage;                              // RAP1阶段百分比
    double type1PolledPercentage;                       // Type1轮询阶段百分比
    double eap2Percentage;                              // EAP2阶段百分比
    double rap2Percentage;                              // RAP2阶段百分比
    double type2PolledPercentage;                       // Type2轮询阶段百分比
    double capPercentage;                               // CAP阶段百分比
    
    // 超帧阶段持续时间参数
    simtime_t eap1Duration;                             // EAP1阶段持续时间
    simtime_t rap1Duration;                             // RAP1阶段持续时间
    simtime_t eap2Duration;                             // EAP2阶段持续时间
    simtime_t rap2Duration;                             // RAP2阶段持续时间
    simtime_t mcapDuration;                             // MCAP阶段持续时间
    simtime_t type1PolledDuration;                      // Type1轮询阶段持续时间
    simtime_t type2PolledDuration;                      // Type2轮询阶段持续时间
    simtime_t singlePollSlotDuration;                   // 单个轮询时隙长度

    // ================ 状态变量 ================
    int seqNum;                                               // 序列号
    std::queue<improvedwban::WBANDataPacket*> dataQueue;      // 数据队列
    SuperframePhase currentPhase;                             // 跟踪当前超帧阶段
    simtime_t superframeStartTime;                            // 超帧开始时间
    bool isTransmitting;                                      // 当前是否正在传输
    bool isSensing;                                           // 当前是否正在感知信道
    
    // CSMA/CA 状态
    std::map<int, int> retransmissionCount;                   // 跟踪每个数据包的重传次数
    std::map<int, int> currentCW;                             // 当前争用窗口大小
    std::map<int, int> backoffCounter;                        // 退避计数器
    
    // ================ 时间同步相关变量 ================
    simtime_t localClockOffset;                              // 本地时钟与Hub时钟的偏移量
    simtime_t distributedGuardTime;                          // 分布式guard时间
    simtime_t centralizedGuardTime;                          // 集中式guard时间
    
    // IEEE 802.15.6标准定义的时间参数
    simtime_t pSIFS;                                         // 短帧间隔 (75μs)
    simtime_t pMIFS;                                         // 中帧间隔 (20μs)
    simtime_t pExtraIFS;                                     // 额外帧间隔
    simtime_t mClockResolution;                              // 时钟分辨率
    simtime_t mTimeOut;                                      // 超时时间 (30μs)
    
    // IEEE 802.15.6标准定义的协议参数
    static const int MAC_HEADER_BYTES = 10;                  // MAC头部固定大小(字节)
    
    // 同步状态
    bool isSynchronized;                                     // 是否已同步
    simtime_t lastSyncTime;                                  // 上次同步时间
    int syncInterval;                                        // 同步间隔（时隙数）
    
    // 时钟同步与guard时间补偿增强参数
    double hubClockPPM;                                      // Hub时钟精度 (ppm)
    double nodeClockPPM;                                     // 节点时钟精度 (ppm)
    simtime_t nominalSyncInterval;                           // 标称同步间隔 (SIn)
    simtime_t maxSyncInterval;                               // 最大同步间隔 (SIN)
    simtime_t baseGuardTime;                                 // 基础guard时间 (GT0)
    simtime_t extraGuardTime;                                // 额外guard时间 (GTa)
    int syncFailureCount;                                    // 同步失败计数
    int maxSyncFailures;                                     // 最大允许同步失败次数
    
    // ================ 轮询机制状态变量 ================
    // --- Hub侧 ---
    std::vector<int> polledNodeList;                          // 需要被轮询的节点ID列表
    int currentPollingNodeIndex;                              // 当前正在轮询的节点在列表中的索引
    int currentlyPolledNodeId;                                // 当前被轮询的节点ID
    int pollAllocationSlots;                                  // Hub为每个节点分配的时隙数
    int pollSentCount;                                        // Hub发送轮询帧计数
    int polledDataReceivedCount;                              // Hub接收轮询数据计数
    
    // --- Node侧 ---
    bool inPolledAllocation;                                  // 是否在轮询分配中
    simtime_t allocationEndTime;                              // 分配结束时间
    int polledDataSentCount;                                  // Node发送轮询数据计数
    int pollReceivedCount;                                    // Node接收轮询帧计数
    
    // Guard时间补偿相关变量
    improvedwban::WBANDataPacket* pendingPolledPacket;        // 待发送的轮询数据包

    // ================ 定时器 (自消息) ================
    // 超帧阶段定时器
    cMessage *beaconTimer;                                    // 信标定时器
    cMessage *eap1StartTimer;                                 // EAP1开始定时器
    cMessage *eap1EndTimer;                                   // EAP1结束定时器
    cMessage *rap1StartTimer;                                 // RAP1开始定时器
    cMessage *rap1EndTimer;                                   // RAP1结束定时器 
    cMessage *type1PolledStartTimer;                          // TYPE1轮询开始定时器
    cMessage *type1PolledEndTimer;                            // TYPE1轮询结束定时器  
    cMessage *eap2StartTimer;                                 // EAP2开始定时器
    cMessage *eap2EndTimer;                                   // EAP2结束定时器
    cMessage *rap2StartTimer;                                 // RAP2开始定时器
    cMessage *rap2EndTimer;                                   // RAP2结束定时器
    cMessage *type2PolledStartTimer;                          // TYPE2轮询开始定时器
    cMessage *type2PolledEndTimer;                            // TYPE2轮询结束定时器
    cMessage *capStartTimer;                                  // CAP开始定时器
    cMessage *capEndTimer;                                    // CAP结束定时器
    
    // CSMA/CA机制定时器
    cMessage *retransmissionTimer;                            // 重传定时器
    cMessage *backoffTimer;                                   // 退避定时器
    cMessage *ccaTimer;                                       // CCA检测定时器
    
    // 轮询机制定时器
    cMessage *pollResponseTimeoutTimer;                       // 轮询响应超时定时器
    cMessage *executeNextPollTimer;                           // 执行下一次轮询定时器
    cMessage *pollTimer;                                      // 用于调度下一个轮询的定时器
    
    // 时序管理定时器
    cMessage *slotEndTimer;                                   // 时隙结束定时器
    cMessage *senseDurationTimer;                             // 感知持续时间定时器
    
    // ================ 统计信号 ================
    // 基本统计信号
    simsignal_t packetReceivedSignal;
    simsignal_t packetSentSignal;

    simsignal_t packetDroppedSignal;
    simsignal_t endToEndDelaySignal;
    simsignal_t queueLengthSignal;
    simsignal_t channelStateSignal;                           // 信道状态信号
    simsignal_t throughputSignal;                             // 吞吐量信号
    
    // 轮询统计信号
    simsignal_t pollSentSignal;                               // Hub发送轮询帧信号
    simsignal_t pollReceivedSignal;                           // Node接收轮询帧信号
    simsignal_t polledDataSentSignal;                         // Node发送轮询数据信号
    simsignal_t polledDataReceivedSignal;                     // Hub接收轮询数据信号
    
    // ================ 其他变量 ================
    int totalSlots;                                           // 超帧中的总时隙数
    
    // 载波监听统计
    int channelBusyCount;                                     // 信道忙计数
    int channelIdleCount;                                     // 信道空闲计数
    
    // 重传机制
    bool channelIdle;                                         // 信道状态
    
    // 吞吐量统计相关变量
    simtime_t lastThroughputUpdateTime;                       // 上次更新吞吐量的时间
    long totalReceivedBytes;                                  // 总接收字节数
    double currentThroughput;                                 // 当前吞吐量
};
}
#endif
