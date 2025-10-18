// 实现IEEE 802.15.6 协议的Mac层
#include "IEEE802_15_6Mac.h"
namespace improvedwban {
Define_Module(IEEE802_15_6Mac);

//==============================================================================
// 1. OMNeT++ 核心函数 (Core OMNeT++ Functions)
//==============================================================================

// 1.1. 构造函数：初始化所有指针为空
IEEE802_15_6Mac::IEEE802_15_6Mac() {
    beaconTimer = nullptr;
    eap1StartTimer = nullptr;   eap1EndTimer = nullptr;
    rap1StartTimer = nullptr;   rap1EndTimer = nullptr;
    type1PolledStartTimer = nullptr;    type1PolledEndTimer = nullptr;
    eap2StartTimer = nullptr;   eap2EndTimer = nullptr;
    rap2StartTimer = nullptr;   rap2EndTimer = nullptr;
    type2PolledStartTimer = nullptr;   type2PolledEndTimer = nullptr;
    capStartTimer = nullptr;   capEndTimer = nullptr;
    retransmissionTimer = nullptr;
    slotEndTimer = nullptr;
    backoffTimer = nullptr;
    // channelCheckTimer = nullptr;
    ccaTimer = nullptr;
    senseDurationTimer = nullptr;
    pollTimer = nullptr; // 新增
}

// 1.2. 析构函数：清理所有定时器和队列
IEEE802_15_6Mac::~IEEE802_15_6Mac() {
    cancelAndDelete(beaconTimer);
    cancelAndDelete(eap1StartTimer);    cancelAndDelete(eap1EndTimer);
    cancelAndDelete(rap1StartTimer);    cancelAndDelete(rap1EndTimer);
    cancelAndDelete(type1PolledStartTimer); cancelAndDelete(type1PolledEndTimer);
    cancelAndDelete(eap2StartTimer);    cancelAndDelete(eap2EndTimer);
    cancelAndDelete(rap2StartTimer);    cancelAndDelete(rap2EndTimer);
    cancelAndDelete(type2PolledStartTimer);    cancelAndDelete(type2PolledEndTimer);
    cancelAndDelete(capStartTimer);    cancelAndDelete(capEndTimer);
    cancelAndDelete(retransmissionTimer);
    cancelAndDelete(slotEndTimer);
    cancelAndDelete(backoffTimer);
    //cancelAndDelete(channelCheckTimer);
    cancelAndDelete(pollTimer); // 新增
    cancelAndDelete(ccaTimer);
    cancelAndDelete(senseDurationTimer);
    
    // 清空队列中的数据包
    while (!dataQueue.empty()) {
        improvedwban::WBANDataPacket* pkt = dataQueue.front();
        if (pkt) {
            delete pkt;
        }
        dataQueue.pop();
    }
}

// 1.3. 协议初始化函数：仿真开始的入口
void IEEE802_15_6Mac::initialize() {
    // 初始化错误处理器
    ErrorHandler::initialize();
    ErrorHandler::setContinueOnError(true);
    ErrorHandler::setMaxErrorsBeforeStop(1000);  // 设置较大的错误阈值
    
    // 获取参数
    nodeId = par("nodeId");                                         // 赋予节点ID值
    isHub = par("isHub");                                           // 判断是否为Hub节点
    beaconPeriodLength = par("beaconPeriodLength");                 // 赋予超帧周期长度
    allocationSlotLength = par("allocationSlotLength");             // 赋予时隙长度
    totalSlots = (int)(beaconPeriodLength / allocationSlotLength);  // 计算超帧中的总时隙数
    
    // 获取超帧阶段百分比参数
    eap1Percentage = par("eap1Percentage");                         // EAP1阶段百分比
    rap1Percentage = par("rap1Percentage");                         // RAP1阶段百分比
    type1PolledPercentage = par("type1PolledPercentage");           // Type1轮询阶段百分比
    eap2Percentage = par("eap2Percentage");                         // EAP2阶段百分比
    rap2Percentage = par("rap2Percentage");                         // RAP2阶段百分比
    type2PolledPercentage = par("type2PolledPercentage");           // Type2轮询阶段百分比
    capPercentage = par("capPercentage");                           // CAP阶段百分比
    
    // 检查百分比参数总和是否为1.0
    double totalPercentage = eap1Percentage + rap1Percentage + type1PolledPercentage + 
                           eap2Percentage + rap2Percentage + type2PolledPercentage + capPercentage;
    
    if (fabs(totalPercentage - 1.0) > 0.001) {
        EV_ERROR << "[INIT] Node " << nodeId << ": Warning: Superframe phase percentages sum to " 
                << totalPercentage << ", not 1.0. This may cause timing issues." << endl;
    }
    
    // 根据百分比计算各阶段持续时间
    eap1Duration = beaconPeriodLength * eap1Percentage;             // EAP1阶段持续时间
    rap1Duration = beaconPeriodLength * rap1Percentage;             // RAP1阶段持续时间
    type1PolledDuration = beaconPeriodLength * type1PolledPercentage; // Type1轮询阶段持续时间
    eap2Duration = beaconPeriodLength * eap2Percentage;             // EAP2阶段持续时间
    rap2Duration = beaconPeriodLength * rap2Percentage;             // RAP2阶段持续时间
    type2PolledDuration = beaconPeriodLength * type2PolledPercentage; // Type2轮询阶段持续时间
    mcapDuration = beaconPeriodLength * capPercentage;              // MCAP阶段持续时间

    // 初始化不同优先级的CWmin和CWmax
    CWminPriority[0] = par("CWminPriority0");
    CWmaxPriority[0] = par("CWmaxPriority0");
    CWminPriority[1] = par("CWminPriority1");
    CWmaxPriority[1] = par("CWmaxPriority1");
    CWminPriority[2] = par("CWminPriority2");
    CWmaxPriority[2] = par("CWmaxPriority2");
    CWminPriority[3] = par("CWminPriority3");
    CWmaxPriority[3] = par("CWmaxPriority3");
    CWminPriority[4] = par("CWminPriority4");
    CWmaxPriority[4] = par("CWmaxPriority4");
    CWminPriority[5] = par("CWminPriority5");
    CWmaxPriority[5] = par("CWmaxPriority5");
    CWminPriority[6] = par("CWminPriority6");
    CWmaxPriority[6] = par("CWmaxPriority6");
    CWminPriority[7] = par("CWminPriority7");
    CWmaxPriority[7] = par("CWmaxPriority7");

    queueSize = par("queueSize");
    // 载波监听参数初始化
    ccaThreshold = par("ccaThreshold");                             // CCA阈值，通常为 -75 dBm
    ccaDuration = SimTime(par("ccaDuration").doubleValue(), SIMTIME_US);                               // 128 μs
    
    // 重传参数设置
    maxRetransmissions = par("maxRetransmissions");
    baseRetransmissionTimeout = par("baseRetransmissionTimeout");


    // 初始化计数器
    seqNum = 0;

    // 载波监听状态初始化
    isTransmitting = false;
    isSensing = false;
    channelBusyCount = 0;
    channelIdleCount = 0;
    
    // 初始化时间同步相关参数
    localClockOffset = 0;                                      // 初始时钟偏移为0
    distributedGuardTime = 0;                                  // 初始分布式guard时间为0
    centralizedGuardTime = 0;                                  // 初始集中式guard时间为0
    
    // IEEE 802.15.6标准定义的时间参数
    pSIFS = SimTime(75, SIMTIME_US);                           // 短帧间隔 (75μs)
    pMIFS = SimTime(20, SIMTIME_US);                           // 中帧间隔 (20μs)
    pExtraIFS = SimTime(10, SIMTIME_US);                       // 额外帧间隔 (10μs)
    mClockResolution = SimTime(1, SIMTIME_US);                 // 时钟分辨率 (1μs)
    mTimeOut = SimTime(30, SIMTIME_US);                        // 超时时间 (30μs)
    
    // 同步状态初始化
    isSynchronized = isHub;                                    // Hub默认已同步，节点需要同步
    lastSyncTime = simTime();                                  // 上次同步时间初始化为当前时间
    syncInterval = totalSlots;                                 // 同步间隔设置为超帧长度
    
    // 时钟同步与guard时间补偿增强参数初始化
    hubClockPPM = isHub ? 0.0 : par("hubClockPPM").doubleValue();  // Hub时钟精度 (ppm)
    nodeClockPPM = par("nodeClockPPM").doubleValue();           // 节点时钟精度 (ppm)
    nominalSyncInterval = beaconPeriodLength * 8;              // 标称同步间隔 (SIn = 8×超帧长度)
    maxSyncInterval = par("maxSyncInterval").doubleValue();     // 最大同步间隔 (SIN)
    maxSyncFailures = par("maxSyncFailures").intValue();        // 最大允许同步失败次数
    syncFailureCount = 0;                                       // 初始同步失败计数为0
    
    // 计算基础guard时间 (GT0 = pSIFS + pExtraIFS + mClockResolution)
    baseGuardTime = pSIFS + pExtraIFS + mClockResolution;
    extraGuardTime = SimTime(0, SIMTIME_US);                    // 初始无额外guard时间
    
    // 初始化分布式和集中式guard时间
    distributedGuardTime = baseGuardTime;                      // 初始化为基础guard时间
    centralizedGuardTime = baseGuardTime;                      // 初始化为基础guard时间
    
    // Guard时间补偿相关变量初始化
    pendingPolledPacket = nullptr;                             // 初始没有待发送的轮询数据包
    
    currentPhase = INACTIVE; // [FIX] 关键修复：初始化当前阶段！

    // 注册统计信号
    packetSentSignal = registerSignal("packetSent");
    packetReceivedSignal = registerSignal("packetReceived");
    packetDroppedSignal = registerSignal("packetDropped");
    endToEndDelaySignal = registerSignal("endToEndDelay");
    queueLengthSignal = registerSignal("queueLength");
    channelStateSignal = registerSignal("channelState");
    throughputSignal = registerSignal("throughput");
    
    // 注册轮询统计信号
    pollSentSignal = registerSignal("pollSent");
    pollReceivedSignal = registerSignal("pollReceived");
    polledDataSentSignal = registerSignal("polledDataSent");
    polledDataReceivedSignal = registerSignal("polledDataReceived");
    
    // 初始化吞吐量统计相关变量
    lastThroughputUpdateTime = simTime();
    totalReceivedBytes = 0;
    currentThroughput = 0.0;


    // 创建自消息
    beaconTimer = new cMessage("beaconTimer", SEND_BEACON);
    eap1StartTimer = new cMessage("eap1StartTimer", EAP1_START);
    eap1EndTimer = new cMessage("eap1EndTimer", EAP1_END);
    rap1StartTimer = new cMessage("rap1StartTimer", RAP1_START);
    rap1EndTimer = new cMessage("rap1EndTimer", RAP1_END);
    type1PolledStartTimer = new cMessage("type1PolledStartTimer", TYPE1_POLLED_START);
    type1PolledEndTimer = new cMessage("type1PolledEndTimer", TYPE1_POLLED_END);
    eap2StartTimer = new cMessage("eap2StartTimer", EAP2_START);
    eap2EndTimer = new cMessage("eap2EndTimer", EAP2_END);
    rap2StartTimer = new cMessage("rap2StartTimer", RAP2_START);
    rap2EndTimer = new cMessage("rap2EndTimer", RAP2_END);
    type2PolledStartTimer = new cMessage("type2PolledStartTimer", TYPE2_POLLED_START);
    type2PolledEndTimer = new cMessage("type2PolledEndTimer", TYPE2_POLLED_END);
    capStartTimer = new cMessage("capStartTimer", CAP_START);
    capEndTimer = new cMessage("capEndTimer", CAP_END);
    retransmissionTimer = new cMessage("retransmissionTimer", RETRANSMISSION_TIMEOUT);
    // backoffTimer = new cMessage("backoffTimer", BACKOFF_TIMEOUT);
    backoffTimer = new cMessage("backoffTimer", BACKOFF_SLOT_TICK);
    slotEndTimer = new cMessage("slotEndTimer", SLOT_END);

    // 创建载波监听相关定时器
    ccaTimer = new cMessage("ccaTimer", CCA_CHECK);
    senseDurationTimer = new cMessage("senseDurationTimer", SENSE_DURATION_END);
    
    // 轮询机制初始化 ---
    pollTimer = new cMessage("pollTimer", POLL_NODE);
    
    // 初始化轮询相关变量
    currentPollingNodeIndex = 0; // 确保初始化为0
    superframeStartTime = SimTime(-1, SIMTIME_S); // 初始化为无效值

    // --- 填充轮询列表 ---
   int numNodes = getParentModule()->getParentModule()->par("numNodes");
   if (numNodes <= 0) {
       EV_ERROR << "[INIT] Node " << nodeId << ": Invalid number of nodes: " << numNodes << endl;
       numNodes = 0;
   }
   
   for (int i = 1; i <= numNodes; i++) {
       // 从应用层获取 userPriority 参数
       cModule *nodeModule = getParentModule()->getParentModule()->getSubmodule("node", i-1);
       if (nodeModule) {
           cModule *appModule = nodeModule->getSubmodule("app");
           if (appModule && appModule->hasPar("userPriority")) {
               int userPriority = appModule->par("userPriority");
               if(userPriority >= 8 && userPriority <= 15)
                   polledNodeList.push_back(i);
           }
       }
   }
   
   // 初始化轮询计数器
   pollSentCount = 0;
   polledDataSentCount = 0;
   pollReceivedCount = 0;
   polledDataReceivedCount = 0;


   // --- DEBUG ---
   if (isHub) {
       EV << "[POLL_INIT] Hub " << nodeId << " initialized. Polling list has " << polledNodeList.size() << " nodes." << endl;
   }

    // 如果是Hub节点，开始发送信标
    if (isHub) {
        // 检查beaconPeriodLength是否有效
        if (beaconPeriodLength > 0) {
            scheduleAt(simTime() + 0.01, beaconTimer);
        } else {
            EV_ERROR << "[INIT] Hub " << nodeId << ": Invalid beacon period length: " << beaconPeriodLength << ". Not scheduling beacon timer." << endl;
        }
    }
}

// 1.4. 消息处理函数：仿真事件的中央调度器
void IEEE802_15_6Mac::handleMessage(cMessage *msg) {
    // 处理自消息
    if (msg->isSelfMessage()) {
        switch (msg->getKind()) {
            case SEND_BEACON:                                       // beaconTimer中携带的是SEND_BEACON信号量
                startBeaconPeriod();                                // 开始执行超帧周期函数
                break;
            case EAP1_START:
                startEAP1();
                break;
            case EAP1_END:{                                         // EAP1结束，开始RAP1                
                currentPhase = INACTIVE;    
                isSensing = false;
                if(ccaTimer->isScheduled()) cancelEvent(ccaTimer);
                if(backoffTimer->isScheduled()) cancelEvent(backoffTimer);
                if (rap1StartTimer->isScheduled()) cancelEvent(rap1StartTimer);
                scheduleAt(simTime(), rap1StartTimer);
                break;
            }
            case RAP1_START:
                startRAP1();
                break;
            case RAP1_END:{                                        // RAP1结束，开始Type1轮询
                currentPhase = INACTIVE;    
                isSensing = false;
                if(ccaTimer->isScheduled()) cancelEvent(ccaTimer);
                if(backoffTimer->isScheduled()) cancelEvent(backoffTimer);
                if (type1PolledStartTimer->isScheduled()) cancelEvent(type1PolledStartTimer);
                scheduleAt(simTime(), type1PolledStartTimer);
                break;
            }
            case TYPE1_POLLED_START:
                startType1Polled();
                break;
            case TYPE1_POLLED_END:
                // Type1轮询结束，开始EAP2
                {
                    currentPhase = INACTIVE;    
                    isSensing = false;
                    if(ccaTimer->isScheduled()) cancelEvent(ccaTimer);
                    if(backoffTimer->isScheduled()) cancelEvent(backoffTimer);
                    scheduleAt(simTime(), eap2StartTimer);
                    break;
                }
            case EAP2_START:
                startEAP2();
                break;
            case EAP2_END:
                // EAP2结束，开始RAP2
                {
                    currentPhase = INACTIVE;    
                    isSensing = false;
                    if(ccaTimer->isScheduled()) cancelEvent(ccaTimer);
                    if(backoffTimer->isScheduled()) cancelEvent(backoffTimer);
                    scheduleAt(simTime(), rap2StartTimer);
                    break;
                }
            case RAP2_START:
                startRAP2();
                break;
            case RAP2_END:
                {
                    currentPhase = INACTIVE;  
                    isSensing = false;
                    if(ccaTimer->isScheduled()) cancelEvent(ccaTimer);
                    if(backoffTimer->isScheduled()) cancelEvent(backoffTimer);    
                    // RAP2结束，开始Type2轮询
                    scheduleAt(simTime(), type2PolledStartTimer);
                    break;
                }
            case TYPE2_POLLED_START:
                startType2Polled();
                break;
            case TYPE2_POLLED_END:
                {
                    currentPhase = INACTIVE;
                    isSensing = false;
                    if(ccaTimer->isScheduled()) cancelEvent(ccaTimer);
                    if(backoffTimer->isScheduled()) cancelEvent(backoffTimer);
                    // Type2轮询结束，开始CAP
                    scheduleAt(simTime(), capStartTimer);
                    break;
                }
            case CAP_START:
                startMCAP();
                break;
            case CAP_END:
                // CAP结束，超帧周期结束
                {
                    currentPhase = INACTIVE;    //重置状态
                    isSensing = false;
                    if(ccaTimer->isScheduled()) cancelEvent(ccaTimer);
                    if(backoffTimer->isScheduled()) cancelEvent(backoffTimer);
                    if (isHub) {
                        // 准备开始下一个超帧周期（删除，避免时钟漂移）
                        // scheduleAt(simTime() + beaconPeriodLength, beaconTimer); // 修改此处
                    }
                    break;
                }
            case CCA_CHECK:
                {   
                    performCCA();
                    break;
                }
            case TX_END:
                // 传输结束，重置传输状态
                {isTransmitting = false;
                if(nodeId == 0){
                    EV << "Hub节点的传输结束" << endl;
                } else {
                    EV << "节点：" << nodeId << "的传输结束，当前时间为：" << simTime() << endl;
                }
                
                // delete msg;
                tryInitiateTransmission();
                break;}
            case SENSE_DURATION_END:
                {isSensing = false;
                EV << "Node " << nodeId << " sensing duration ended" << endl;
                break;}
            case RETRANSMISSION_TIMEOUT:
                // 重传超时，检查队列中的数据包
                {if (!dataQueue.empty()) {
                    improvedwban::WBANDataPacket* pkt = dataQueue.front();
                    if (pkt) {
                        EV << "Node " << nodeId << ": Retransmission timeout for packet Seq="
                           << pkt->getSequenceNumber() << endl;
                        handleRetransmission(pkt); // <--- 调用重传处理函数
                    } else {
                        EV_ERROR << "[RETRANSMISSION_TIMEOUT] Node " << nodeId << ": NULL packet at front of queue. Removing it." << endl;
                        dataQueue.pop();
                    }
                }
                break;}

            case BACKOFF_SLOT_TICK:                     // 一个时隙结束，计数器退减
                processBackoffSlot();
                break;
            case SLOT_END:
                // 时隙结束
                break;
            case POLL_NODE: 
                EV << "[POLL_EVENT] Node " << nodeId << ": Received self-message POLL_NODE." << endl;
                if (isHub) {
                    // 增加边界检查，防止访问越界
                    if (currentPollingNodeIndex < polledNodeList.size()) {
                        executePolling(); 
                    } else {
                        EV << "[POLL_EVENT] Hub " << nodeId << ": currentPollingNodeIndex (" << currentPollingNodeIndex 
                           << ") >= polledNodeList.size() (" << polledNodeList.size() << "). Cancelling pollTimer." << endl;
                        if (pollTimer->isScheduled()) {
                            cancelEvent(pollTimer);
                        }
                    }
                }
                else {
                    EV << "[POLL_EVENT] Node " << nodeId << " is not a Hub, ignoring POLL_NODE self-message." << endl;
                }
                break;
            case GUARD_TIME_END:
                EV << "[GUARD_TIME] Node " << nodeId << ": Guard time ended, sending pending polled packet." << endl;
                if (pendingPolledPacket != nullptr) {
                    // 检查数据包有效性
                    if (pendingPolledPacket->getByteLength() <= 0) {
                        EV_ERROR << "[GUARD_TIME] Node " << nodeId << ": Invalid pending polled packet with length " 
                                << pendingPolledPacket->getByteLength() << ". Dropping packet." << endl;
                        delete pendingPolledPacket;
                        pendingPolledPacket = nullptr;
                        break;
                    }
                    
                    // 检查当前是否在正确的轮询阶段
                    if (currentPhase != TYPE1_POLLED && currentPhase != TYPE2_POLLED) {
                        EV_ERROR << "[GUARD_TIME] Node " << nodeId << ": Not in polling phase (current phase: " 
                                << currentPhase << "). Cancelling packet transmission." << endl;
                        delete pendingPolledPacket;
                        pendingPolledPacket = nullptr;
                        break;
                    }
                    
                    // 检查同步状态，如果未同步，尝试重新同步
                    if (!isSynchronized) {
                        EV_WARN << "[GUARD_TIME] Node " << nodeId << ": Not synchronized. Attempting to use last known clock offset." << endl;
                        // 使用最后一次已知的时钟偏移量调整发送时间
                        if (localClockOffset != 0) {
                            simtime_t adjustedTime = simTime() + localClockOffset;
                            EV << "[GUARD_TIME] Node " << nodeId << ": Adjusted send time by " << localClockOffset 
                               << " to " << adjustedTime << endl;
                        }
                    }
                    
                    // 设置轮询数据包的队列退出时间
                    pendingPolledPacket->setQueueExitTime(simTime());
                    
                    // 发送之前保存的轮询数据包
                    sendDown(pendingPolledPacket->dup()); // 发送副本，原件保留直到收到ACK
                    EV << "[GUARD_TIME] Node " << nodeId << ": Sent pending polled packet after guard time." << endl;
                    
                    // 为这个轮询的数据包设置ACK超时
                    if (retransmissionTimer->isScheduled()) {
                        EV_WARN << "[GUARD_TIME] Node " << nodeId << " retransmissionTimer is already scheduled. Cancelling it." << endl;
                        cancelEvent(retransmissionTimer);
                    }
                    
                    // 检查baseRetransmissionTimeout有效性
                    if (baseRetransmissionTimeout <= 0) {
                        EV_ERROR << "[GUARD_TIME] Node " << nodeId << " has invalid baseRetransmissionTimeout: " 
                                << baseRetransmissionTimeout << ". Using default 0.01s." << endl;
                        baseRetransmissionTimeout = 0.01; // 默认10ms
                    }
                    
                    // 根据同步状态调整重传超时时间
                    // 如果同步状态不佳，增加重传超时时间以补偿可能的时钟漂移
                    double timeout = baseRetransmissionTimeout;
                    if (!isSynchronized) {
                        // 未同步时，使用更长的超时时间
                        timeout *= 2.0;
                        EV << "[GUARD_TIME] Node " << nodeId << ": Not synchronized, using extended retransmission timeout: " 
                           << timeout << "s (base: " << baseRetransmissionTimeout << "s)" << endl;
                    } else {
                        simtime_t timeSinceLastSync = simTime() - lastSyncTime;
                        if (timeSinceLastSync > nominalSyncInterval) {
                            // 同步间隔过长，适当增加超时时间
                            double extensionFactor = 1.0 + (timeSinceLastSync / nominalSyncInterval - 1.0) * 0.5;
                            timeout *= extensionFactor;
                            EV << "[GUARD_TIME] Node " << nodeId << ": Sync interval extended, using adjusted retransmission timeout: " 
                               << timeout << "s (base: " << baseRetransmissionTimeout << "s, factor: " << extensionFactor << ")" << endl;
                        }
                    }
                    
                    // 设置重传定时器
                    scheduleAt(simTime() + timeout, retransmissionTimer);
                    
                    // 更新轮询发送统计
                    polledDataSentCount++;
                    emit(polledDataSentSignal, polledDataSentCount);
                    
                    // 保留原始数据包在队列中，直到收到ACK
                    pendingPolledPacket = nullptr;
                } else {
                    EV_WARN << "[GUARD_TIME] Node " << nodeId << ": No pending polled packet to send." << endl;
                }
                break;    
            default:
                HANDLE_ERROR_FORMATTED("Unknown self message kind: %d", msg->getKind());
        }
    }
    // 处理从上层收到的消息
    else if (msg->arrivedOn("upperLayerIn")) {
        handleUpperPacket(msg);
    }
    // 处理从下层收到的消息
    else if (msg->arrivedOn("lowerLayerIn")) {
        handleLowerPacket(msg);
    }
    else {
        HANDLE_ERROR("Message arrived on unknown gate");
    }
}

// 1.5. 仿真结束函数
void IEEE802_15_6Mac::finish() {
    // 记录模拟结束时的统计信息
    recordScalar("dataQueueLength", dataQueue.size());
    recordScalar("totalErrors", ErrorHandler::getErrorCount());
    
    // 清理错误处理器
    ErrorHandler::cleanup();
}

//==============================================================================
// 2. 超帧生命周期管理 (Superframe Lifecycle Management)
//==============================================================================


// 2.1. Hub发送信标帧函数
void IEEE802_15_6Mac::startBeaconPeriod() {
    if (isHub) {
        EV << "Hub starting new beacon period at " << simTime() << endl;
        
        // 集中式guard时间补偿计算：GTc = GT0 + SI×(PH+PN)
        // SI为节点最大同步间隔，PH/PN为Hub/节点时钟精度
        simtime_t baseGuardTime = pSIFS + pExtraIFS + mClockResolution;  // GT0
        simtime_t syncInterval = beaconPeriodLength;                     // SI
        double hubClockPrecision = 0.0001;                               // PH，Hub时钟精度
        double nodeClockPrecision = 0.0001;                              // PN，节点时钟精度
        
        // 检查beaconPeriodLength是否有效
        if (beaconPeriodLength <= 0) {
            EV_ERROR << "[BEACON] Hub " << nodeId << ": Invalid beacon period length: " << beaconPeriodLength << ". Using default 32ms." << endl;
            beaconPeriodLength = 0.032; // 默认32ms
            syncInterval = beaconPeriodLength;
        }
        
        // 计算集中式guard时间
        centralizedGuardTime = baseGuardTime + syncInterval * (hubClockPrecision + nodeClockPrecision);
        
        EV << "[GUARD_TIME] Hub " << nodeId << ": Centralized guard time calculation." << endl;
        EV << "[GUARD_TIME] Base guard time (GT0): " << baseGuardTime << endl;
        EV << "[GUARD_TIME] Sync interval (SI): " << syncInterval << endl;
        EV << "[GUARD_TIME] Hub precision (PH): " << hubClockPrecision << ", Node precision (PN): " << nodeClockPrecision << endl;
        EV << "[GUARD_TIME] Centralized guard time (GTc): " << centralizedGuardTime << endl;
        
        // 创建信标帧，并设置信标帧参数
        improvedwban::WBANBeaconPacket *beacon = new improvedwban::WBANBeaconPacket("WBAN-Beacon");
        beacon->setBitLength(128);                                                                              // 设置信标帧的长度为128bit
        beacon->setSourceId(nodeId);                                                                            // 发送方的ID，这里的ID为0，代表hub
        beacon->setDestinationId(-1);                                                                           // -1代表广播通信方式
        beacon->setSequenceNumber(seqNum++);                                                                    // 为当前信标帧分配一个序列号
        beacon->setCreationTime(simTime());                                                                     // 记录当前信标帧的创建时间 
        beacon->setPayloadLength(0);                                                                            // 信标帧的有效载荷长度（数据部分）为0
        beacon->setBeaconPeriodLength(totalSlots);                                                              // 信标周期个数，单位（信标个数）
        
        // 设置新的Frame Control字段
        beacon->setFrameType(improvedwban::MANAGEMENT_FRAME);                                                   // 信标帧是管理帧类型
        beacon->setFrameSubtype(improvedwban::MGMT_BEACON);                                                     // 信标子类型
        
        // 检查allocationSlotLength是否有效
        if (allocationSlotLength <= 0) {
            EV_ERROR << "[BEACON] Hub " << nodeId << ": Invalid allocation slot length: " << allocationSlotLength << ". Using default 1ms." << endl;
            allocationSlotLength = 0.001; // 默认1ms
        }
        
        beacon->setAllocationSlotLength(allocationSlotLength.dbl() * 1000000);                                  // 单个时隙的长度，单位（微秒-μs）
        
        // 向下层发送信标帧
        sendDown(beacon);
        
        // 更新超帧开始时间
        superframeStartTime = simTime();
        
        // 立即调度下一个信标，以建立精确的周期
        scheduleAt(simTime() + beaconPeriodLength, beaconTimer);

         //安排EAP1开始
         if (eap1StartTimer->isScheduled()) {
            cancelEvent(eap1StartTimer);
         }
         scheduleAt(simTime() + 0.001, eap1StartTimer);
    }
}

// 2.2. 各阶段的启动函数
void IEEE802_15_6Mac::startEAP1() {
    // 安排EAP1结束
    currentPhase = EAP1;
    scheduleAt(simTime() + eap1Duration, eap1EndTimer);
    EV << "EAP1 phase started, duration: " << eap1Duration << " s" << endl;
    tryInitiateTransmission();
}

void IEEE802_15_6Mac::startRAP1() {
    currentPhase = RAP1;
    // 安排RAP1结束
    scheduleAt(simTime() + rap1Duration, rap1EndTimer);
    EV << "RAP1 phase started, duration: " << rap1Duration << " s" << endl;
    tryInitiateTransmission();

    // 在随机访问阶段检查并发送队列中的数据包
    // checkAndSendQueuedPackets();
    // UP=0至UP=7的节点通过CSMA/CA方式竞争发送
    // checkAndSendMediumPriorityPackets(0, 7);
}

void IEEE802_15_6Mac::startType1Polled() {
    currentPhase = TYPE1_POLLED;
    // 安排Type1轮询结束
    scheduleAt(simTime() + type1PolledDuration, type1PolledEndTimer);
    EV << "Phase changed to TYPE1_POLLED for Node " << nodeId << ", duration: " << type1PolledDuration << " s" << endl;
    
    // --- 修改: Hub启动轮询流程 ---
    if (isHub) {
        EV << "[POLL_START] Hub " << nodeId << " is initiating polling sequence in Type1 phase." << endl;
        
        // 随机选择起始轮询节点，确保轮询公平性
        if (polledNodeList.empty()) {
            currentPollingNodeIndex = 0;
            EV << "[POLL_START] Hub " << nodeId << " has an empty polling list. No polling will occur." << endl;
        } else {
            currentPollingNodeIndex = intrand(polledNodeList.size());
            EV << "[POLL_START] Hub " << nodeId << ": Starting polling from random index " << currentPollingNodeIndex 
               << " out of " << polledNodeList.size() << " polled nodes." << endl;
        }
        
        // 取消可能已调度的pollTimer，防止重复调度
        if (pollTimer->isScheduled()) {
            cancelEvent(pollTimer);
            EV << "[POLL_START] Hub " << nodeId << ": Cancelled existing pollTimer before scheduling new one." << endl;
        }
        
        // 检查singlePollSlotDuration是否有效
        if (singlePollSlotDuration <= 0) {
            EV_ERROR << "[POLL_START] Hub " << nodeId << ": Invalid single poll slot duration: " << singlePollSlotDuration << ". Using default 2ms." << endl;
            singlePollSlotDuration = 0.002; // 默认2ms
        }
        
        EV << "[POLL_START] Hub " << nodeId << " scheduling first poll timer to fire immediately." << endl;
        scheduleAt(simTime(), pollTimer); // 立即开始第一个轮询
    }
}

void IEEE802_15_6Mac::startEAP2() {
    currentPhase = EAP2;
    // 安排EAP2结束
    scheduleAt(simTime() + eap2Duration, eap2EndTimer);
    EV << "EAP2 phase started, duration: " << eap2Duration << " s" << endl;
    // 仅允许UP=7的节点通过CSMA/CA方式竞争发送
    tryInitiateTransmission();
}

void IEEE802_15_6Mac::startRAP2() {
    currentPhase = RAP2;
    // 安排RAP2结束
    scheduleAt(simTime() + rap2Duration, rap2EndTimer);
    EV << "RAP2 phase started, duration: " << rap2Duration << " s" << endl;
    tryInitiateTransmission();
}

void IEEE802_15_6Mac::startType2Polled() {
    currentPhase = TYPE2_POLLED;
    // 安排Type2轮询结束
    scheduleAt(simTime() + type2PolledDuration, type2PolledEndTimer);
    
    EV << "Phase changed to TYPE2_POLLED for Node " << nodeId << ", duration: " << type2PolledDuration << " s" << endl;
    
    if (isHub) {
        EV << "[POLL_START] Hub " << nodeId << " is initiating polling sequence in Type2 phase." << endl;
        
        // 随机选择起始轮询节点，确保轮询公平性
        if (polledNodeList.empty()) {
            currentPollingNodeIndex = 0;
            EV << "[POLL_START] Hub " << nodeId << " has an empty polling list. No polling will occur." << endl;
        } else {
            currentPollingNodeIndex = intrand(polledNodeList.size());
            EV << "[POLL_START] Hub " << nodeId << ": Starting polling from random index " << currentPollingNodeIndex 
               << " out of " << polledNodeList.size() << " polled nodes." << endl;
        }
        
        // 取消可能已调度的pollTimer，防止重复调度
        if (pollTimer->isScheduled()) {
            cancelEvent(pollTimer);
            EV << "[POLL_START] Hub " << nodeId << ": Cancelled existing pollTimer before scheduling new one." << endl;
        }
        
        // 检查singlePollSlotDuration是否有效
        if (singlePollSlotDuration <= 0) {
            EV_ERROR << "[POLL_START] Hub " << nodeId << ": Invalid single poll slot duration: " << singlePollSlotDuration << ". Using default 2ms." << endl;
            singlePollSlotDuration = 0.002; // 默认2ms
        }
        
        EV << "[POLL_START] Hub " << nodeId << " scheduling first poll timer to fire immediately." << endl;
        scheduleAt(simTime(), pollTimer); // 立即开始第一个轮询
    }
}

void IEEE802_15_6Mac::startMCAP() {
    currentPhase = CAP;
    // 安排CAP结束
    scheduleAt(simTime() + mcapDuration, capEndTimer);
    
    EV << "CAP phase started, duration: " << mcapDuration << " s" << endl;
    tryInitiateTransmission();
    // 在争用访问阶段检查并发送队列中的数据包
    // checkAndSendQueuedPackets();
}

//==============================================================================
// 3. 数据包处理 (Packet Handling - Upper & Lower Layers)
//==============================================================================

// 3.1. Node节点：处理来自上层（应用层）的数据包，将其封装为MAC帧
void IEEE802_15_6Mac::handleUpperPacket(cMessage *msg) {
    EV << "[DEBUG-1] Node " << nodeId << ": handleUpperPacket() called. A packet arrived from App layer." << endl;
    // 将基类型msg转换为继承类WBANDataPacket的类型
    improvedwban::WBANDataPacket *pkt = dynamic_cast<improvedwban::WBANDataPacket*>(msg);
    
    // 判断转换是否成功，如果转换失败，说明msg不是WBANDataPacket类型，直接删除
    if (!pkt) {
        EV_ERROR << "Message from upper layer is not a WBANDataPacket. Deleting." << endl;
        delete msg;
        return;
    }
    
    // 转换成功之后，可以使用子类的种种方法
    pkt->setSourceId(nodeId);                                               // 设置源节点ID
    pkt->setSequenceNumber(seqNum++);                                       // 使用全局序列号，先使用，后递增
    pkt->setMacArrivalTime(simTime());                                      // 记录数据包到达MAC层的时间
    pkt->setByteLength(MAC_HEADER_BYTES + pkt->getPayloadLength());         // 设置MAC帧的负载长度
    
    // 根据IEEE 802.15.6标准设置Frame Control字段
    // 数据帧的frameType已经在WBANDataPacket类中设置为DATA_FRAME
    // 根据用户优先级设置frameSubtype
    int userPriority = pkt->getFrameSubtype();
    if (userPriority >= 0 && userPriority <= 7) {
        pkt->setFrameSubtype(userPriority);  // 用户优先级0-7对应数据帧子类型0-7
    } else if (userPriority == 8) {
        pkt->setFrameSubtype(improvedwban::DATA_EMERGENCY);  // 紧急帧
    } else {
        pkt->setFrameSubtype(0);  // 默认为优先级0
    }
    
    EV << "[ENQUEUE] Node " << nodeId << " received packet from App. Assigned Priority=" << pkt->getFrameSubtype()<< endl;

    // 将数据包放入队列
    enqueuePacket(pkt);

    // 新包入队后，立即尝试发送
    tryInitiateTransmission();
}

// 3.2. Node节点：将数据包放入队列
void IEEE802_15_6Mac::enqueuePacket(improvedwban::WBANDataPacket *pkt) {
    // 检查队列是否已满，如果已满，则直接丢弃该MAC帧
    if (dataQueue.size() >= (size_t)queueSize) {                            // 队列已满，此过程可能会产生丢包
        EV << "[DEBUG-2A] Node " << nodeId << ": QUEUE FULL. Dropping packet Seq=" << pkt->getSequenceNumber() << endl;
        emit(packetDroppedSignal, 1L);                                      // 发送一个丢包信号
        delete pkt;                                                         // 不删除数据包会导致内存泄漏
    } else {
        EV << "[DEBUG-2B] Node " << nodeId << ": Packet Seq=" << pkt->getSequenceNumber() << " ENQUEUED. Queue size is now " << dataQueue.size() << "." << endl;
        dataQueue.push(pkt);
        emit(queueLengthSignal, (long)dataQueue.size());                    // 将MAC帧加入队列，当有新帧入队时，会发送一次当前的队列长度信号
    }
}


// 3.3. Hub/Node节点：处理来自下层（物理层）的数据包
void IEEE802_15_6Mac::handleLowerPacket(cMessage *msg) {
    EV << "[DEBUG-3] Node " << nodeId << ": handleLowerPacket() called. A packet arrived from PHY layer." << endl;
    improvedwban::WBANPacket *pkt = dynamic_cast<improvedwban::WBANPacket*>(msg);       // 暂时还不清楚数据包的类型，只能转换为WBANPacket   
    if (!pkt) { 
        EV_ERROR << "Message from lower layer is not a WBANDataPacket. Deleting." << endl;
        delete msg; 
        return; 
    }

    // 根据IEEE 802.15.6标准的Frame Control字段判断帧类型
    EV << "[LOWER_LAYER] Node " << nodeId << " received a packet with Frame Type=" << pkt->getFrameType() 
       << ", Frame Subtype=" << pkt->getFrameSubtype() << " from source " << pkt->getSourceId() << endl;

    switch (pkt->getFrameType()) {
        case improvedwban::DATA_FRAME: {               // 数据帧
            improvedwban::WBANDataPacket *dataPkt = dynamic_cast<improvedwban::WBANDataPacket*>(pkt);
            if (dataPkt) {
                handleDataPacket(dataPkt);              // 执行数据帧的处理函数
            } else {
                delete pkt;
            }
            break;
        }

        case improvedwban::MANAGEMENT_FRAME: {             // 管理帧
            // 根据子类型进一步判断管理帧类型
            int frameSubtype = pkt->getFrameSubtype();
            
            switch (frameSubtype) {
                case improvedwban::MGMT_BEACON: {         // 信标帧
                    improvedwban::WBANBeaconPacket *beaconPkt = dynamic_cast<improvedwban::WBANBeaconPacket*>(pkt);
                    if (beaconPkt) {
                        handleBeacon(beaconPkt);          // 执行信标帧的处理函数
                    } else {
                        delete pkt;
                    }
                    break;
                }
                
                case improvedwban::MGMT_CONNECTION_REQUEST:
                case improvedwban::MGMT_CONNECTION_ASSIGNMENT:
                case improvedwban::MGMT_DISCONNECTION:
                case improvedwban::MGMT_SECURITY_ASSOCIATION:
                case improvedwban::MGMT_SECURITY_DISASSOCIATION:
                case improvedwban::MGMT_PTK:
                case improvedwban::MGMT_GTK:
                case improvedwban::MGMT_COMMAND:
                default: {
                    // 处理其他管理帧（包括轮询帧）
                    EV << "[LOWER_LAYER] Node " << nodeId << " identified it as a MANAGEMENT_PACKET." << endl;
                    
                    // 检查数据包有效性
                    if (!pkt) {
                        EV_ERROR << "[LOWER_LAYER] Node " << nodeId << ": Received null MANAGEMENT_PACKET. Ignoring." << endl;
                        break;
                    }
                    
                    // 检查数据包长度是否有效
                    if (pkt->getByteLength() <= 0) {
                        EV_ERROR << "[LOWER_LAYER] Node " << nodeId << ": Received MANAGEMENT_PACKET with invalid length: " 
                                << pkt->getByteLength() << ". Ignoring." << endl;
                        delete pkt;
                        break;
                    }
                    
                    // 检查源ID是否有效
                    if (pkt->getSourceId() < 0) {
                        EV_ERROR << "[LOWER_LAYER] Node " << nodeId << ": Received MANAGEMENT_PACKET with invalid source ID: " 
                                << pkt->getSourceId() << ". Ignoring." << endl;
                        delete pkt;
                        break;
                    }
                    
                    // 只有非Hub节点且数据包来自Hub才处理轮询
                    if (!isHub && pkt->getSourceId() == 0) { // 假设Hub ID为0
                        // 检查当前是否在正确的轮询阶段
                        if (currentPhase != TYPE1_POLLED && currentPhase != TYPE2_POLLED) {
                            EV_ERROR << "[LOWER_LAYER] Node " << nodeId << ": Received poll but not in polling phase (current phase: " 
                                    << currentPhase << "). Ignoring." << endl;
                            delete pkt;
                            break;
                        }
                        
                        // 检查是否已经有待处理的轮询数据包
                        if (pendingPolledPacket != nullptr) {
                            EV_WARN << "[LOWER_LAYER] Node " << nodeId << ": Already has a pending polled packet. Replacing with new one." << endl;
                            delete pendingPolledPacket;
                            pendingPolledPacket = nullptr;
                        }
                        
                        // 处理轮询帧
                        improvedwban::WBANManagementPacket *mgmtPkt = check_and_cast<improvedwban::WBANManagementPacket*>(pkt);
                        if (mgmtPkt) {
                            handleManagementPacket(mgmtPkt);
                        } else {
                            EV_ERROR << "[LOWER_LAYER] Node " << nodeId << ": Failed to cast to WBANManagementPacket. Ignoring." << endl;
                            delete pkt;
                        }
                    } else {
                        // 其他管理帧的逻辑
                        EV << "[LOWER_LAYER] Node " << nodeId << " is either a Hub or packet is not from Hub. Ignoring." << endl;
                        delete pkt;
                    }
                    break;
                }
            }
            break;
        }
        
        case improvedwban::CONTROL_FRAME: {             // 控制帧
            // 根据子类型进一步判断控制帧类型
            int frameSubtype = pkt->getFrameSubtype();
            
            
            switch (frameSubtype) {
                case improvedwban::CTRL_I_ACK:           // I-Ack（即时确认帧）
                case improvedwban::CTRL_B_ACK:           // B-Ack（块确认帧）
                case improvedwban::CTRL_I_ACK_POLL:      // I-Ack+Poll（即时确认+轮询帧）
                case improvedwban::CTRL_B_ACK_POLL:      // B-Ack+Poll（块确认+轮询帧）
                default: {
                    improvedwban::WBANAckPacket *ackPkt = dynamic_cast<improvedwban::WBANAckPacket*>(pkt);
                    if (ackPkt) {
                        handleAck(ackPkt);               // 执行确认帧的处理函数
                    } else {
                        delete pkt;
                    }
                    break;
                }
            }
            break;
        }
        
        default:
            EV << "Unknown frame type: " << pkt->getFrameType() << endl;
            delete pkt;
            break;
    }
}

// 3.4. 处理接收到的信标帧（节点侧）
void IEEE802_15_6Mac::handleBeacon(improvedwban::WBANBeaconPacket *pkt) {
    // 节点在收到信标帧之后，开始处理信标帧
    EV << "[DEBUG-3] Node " << nodeId << ": handleBeaconPacket() called. A beacon has been received and successfully identified." << endl;
    
    // 检查数据包有效性
    if (!pkt) {
        EV_ERROR << "[BEACON_PKT] Node " << nodeId << ": Received NULL beacon packet. Ignoring." << endl;
        return;
    }
    
    // 检查信标包长度是否有效
    if (pkt->getByteLength() <= 0) {
        EV_ERROR << "[BEACON_PKT] Node " << nodeId << ": Received beacon packet with invalid length: " 
                << pkt->getByteLength() << ". Ignoring." << endl;
        delete pkt;
        return;
    }
    
    // 检查源ID是否有效
    if (pkt->getSourceId() < 0) {
        EV_ERROR << "[BEACON_PKT] Node " << nodeId << ": Received beacon packet with invalid source ID: " 
                << pkt->getSourceId() << ". Ignoring." << endl;
        delete pkt;
        return;
    }
    
    if (!isHub) {
        // 计算时钟差值 D = TS - TL (TS为信标帧发送时间，TL为本地接收时间)
        simtime_t beaconSendTime = pkt->getCreationTime();
        simtime_t localReceiveTime = simTime();
        
        // 检查时间戳是否有效
        if (beaconSendTime <= 0 || beaconSendTime > localReceiveTime) {
            EV_ERROR << "[BEACON_PKT] Node " << nodeId << ": Invalid timestamp in beacon packet. Beacon send time: " 
                    << beaconSendTime << ", Local receive time: " << localReceiveTime << ". Ignoring." << endl;
            delete pkt;
            return;
        }
        
        simtime_t clockOffset = beaconSendTime - localReceiveTime;
        
        // 记录时钟偏移量用于调试
        EV << "[SYNC] Node " << nodeId << ": Clock synchronization via Beacon. "
           << "Beacon send time: " << beaconSendTime 
           << ", Local receive time: " << localReceiveTime
           << ", Clock offset: " << clockOffset << "s" << endl;
        
        // 更新超帧开始时间，考虑时钟偏移
        superframeStartTime = localReceiveTime + clockOffset;        // 捕获超帧开始时间锚点
        
        // 更新本地时钟偏移量
        localClockOffset = clockOffset;
        
        // 计算同步间隔内的最大漂移 Dn = SIn × HubClockPPM
        simtime_t timeSinceLastSync = localReceiveTime - lastSyncTime;
        simtime_t maxDrift = nominalSyncInterval.dbl() * hubClockPPM * 1e-6;  // 转换ppm为实际时间
        
        // 计算并更新guard时间 (分布式补偿: GTn = GT0 + 2×Dn)
        // GT0 = pSIFS + pExtraIFS + mClockResolution
        simtime_t GT0 = pSIFS + pExtraIFS + mClockResolution;
        simtime_t Dn = std::max(fabs(clockOffset.dbl()), maxDrift.dbl());  // 取实际偏移和最大漂移中的较大值
        
        // 如果同步间隔超过标称同步间隔，计算额外guard时间
        if (timeSinceLastSync > nominalSyncInterval) {
            simtime_t excessTime = timeSinceLastSync - nominalSyncInterval;
            simtime_t Da = excessTime.dbl() * hubClockPPM * 1e-6;  // 额外漂移
            extraGuardTime = 2 * Da;  // GTa = 2×Da
            EV << "[SYNC] Node " << nodeId << ": Excess sync interval detected. "
               << "Time since last sync: " << timeSinceLastSync 
               << ", Excess time: " << excessTime
               << ", Extra guard time: " << extraGuardTime << "s" << endl;
        } else {
            extraGuardTime = SimTime(0, SIMTIME_US);
        }
        
        distributedGuardTime = GT0 + 2 * Dn + extraGuardTime;
        
        // 更新同步状态
        isSynchronized = true;
        lastSyncTime = localReceiveTime;
        syncFailureCount = 0;  // 重置同步失败计数
        
        EV << "[SYNC] Node " << nodeId << ": Clock synchronization completed. "
           << "GT0: " << GT0 << "s, Dn: " << Dn << "s, "
           << "Distributed Guard Time: " << distributedGuardTime << "s, "
           << "Time since last sync: " << timeSinceLastSync << "s" << endl;
        
        // 取消所有之前的定时器
        if (eap1StartTimer->isScheduled()) cancelEvent(eap1StartTimer);
        if (eap1EndTimer->isScheduled()) cancelEvent(eap1EndTimer); 
        if (rap1StartTimer->isScheduled()) cancelEvent(rap1StartTimer);
        if (rap1EndTimer->isScheduled()) cancelEvent(rap1EndTimer);
        if (type1PolledStartTimer->isScheduled()) cancelEvent(type1PolledStartTimer);
        if (type1PolledEndTimer->isScheduled()) cancelEvent(type1PolledEndTimer);  // 添加这行
        if (eap2StartTimer->isScheduled()) cancelEvent(eap2StartTimer);
        if (eap2EndTimer->isScheduled()) cancelEvent(eap2EndTimer); 
        if (rap2StartTimer->isScheduled()) cancelEvent(rap2StartTimer);
        if (rap2EndTimer->isScheduled()) cancelEvent(rap2EndTimer);
        if (type2PolledStartTimer->isScheduled()) cancelEvent(type2PolledStartTimer);
        if (type2PolledEndTimer->isScheduled()) cancelEvent(type2PolledEndTimer);  // 添加这行
        if (capStartTimer->isScheduled()) cancelEvent(capStartTimer);
        if (capEndTimer->isScheduled()) cancelEvent(capEndTimer);
        EV << "[DEBUG-3B] Node " << nodeId << ": Beacon processed. Scheduling EAP1, which will start the active period." << endl;
        
        // 取消所有正在进行的CSMA/CA定时器
        if (backoffTimer->isScheduled()) cancelEvent(backoffTimer);
        if (ccaTimer->isScheduled()) cancelEvent(ccaTimer);
        if (retransmissionTimer->isScheduled()) cancelEvent(retransmissionTimer);
       
        // 重置MAC状态标志和计数器
        isSensing = false;
        
        // 清理上一个超帧未完成的退避信息
        if (!dataQueue.empty()) {
            improvedwban::WBANDataPacket* pkt = dataQueue.front();
            if (pkt) {
                int seqNum = pkt->getSequenceNumber();
                if (backoffCounter.count(seqNum)) {
                    backoffCounter.erase(seqNum);
                    EV << "Node " << nodeId << " [LIFECYCLE]: Cleared frozen backoff state for Seq=" << seqNum << endl;
                }
            } else {
                EV_ERROR << "[BEACON_PKT] Node " << nodeId << ": NULL packet at front of queue. Removing it." << endl;
                dataQueue.pop();
            }
        }

        // 安排新的超帧开始
        currentPhase = INACTIVE; // 明确设置当前为非活动状态
        scheduleAt(simTime() + 0.001, eap1StartTimer);
        EV << "Node " << nodeId << " [LIFECYCLE]: MAC state reset complete. EAP1 scheduled." << endl;
    } else {
        EV_WARN << "[BEACON_PKT] Hub " << nodeId << " received beacon (should not happen)" << endl;
        delete pkt;
        return;
    }
    delete pkt;
}

// 3.5. 处理接收到的数据帧
void IEEE802_15_6Mac::handleDataPacket(improvedwban::WBANDataPacket *pkt) {

    EV << "节点"<< nodeId << "执行到了handleDatePacket函数" <<endl;
    
    // 检查数据包有效性
    if (!pkt) {
        EV_ERROR << "[DATA_PKT] Node " << nodeId << ": Received NULL data packet. Ignoring." << endl;
        return;
    }
    
    // 检查数据包长度是否有效
    if (pkt->getByteLength() <= 0) {
        EV_ERROR << "[DATA_PKT] Node " << nodeId << ": Received data packet with invalid length: " 
                << pkt->getByteLength() << ". Ignoring." << endl;
        delete pkt;
        return;
    }
    
    // 检查源ID和目的ID是否有效
    if (pkt->getSourceId() < 0 || pkt->getDestinationId() < -1) { // -1是广播地址
        EV_ERROR << "[DATA_PKT] Node " << nodeId << ": Received data packet with invalid source ID (" 
                << pkt->getSourceId() << ") or destination ID (" << pkt->getDestinationId() << "). Ignoring." << endl;
        delete pkt;
        return;
    }
    
    // 检查时间戳是否有效
    if (pkt->getCreationTime() <= 0 || pkt->getCreationTime() > simTime()) {
        EV_ERROR << "[DATA_PKT] Node " << nodeId << ": Received data packet with invalid timestamp: " 
                << pkt->getCreationTime() << ". Current time: " << simTime() << ". Ignoring." << endl;
        delete pkt;
        return;
    }
    
    // 处理数据包，当前节点或广播数据包
    if (pkt->getDestinationId() == nodeId || pkt->getDestinationId() == -1) {
        EV << "[DATA_PKT] Node " << nodeId << " Packet (SeqNum=" << pkt->getSequenceNumber()
                 << ") is for THIS node or BROADCAST. Sending to upper layer." << endl;
        
        // 更新接收字节数统计
        totalReceivedBytes += pkt->getByteLength();
        
        // 更新吞吐量统计
        updateThroughput();
        
        // 检查是否为轮询数据（高优先级数据）
        if (pkt->getFrameSubtype() >= 8) {
            // 更新轮询数据接收统计
            polledDataReceivedCount++;
            emit(polledDataReceivedSignal, polledDataReceivedCount);
        }
        
        // 发送确认帧
        improvedwban::WBANAckPacket *ack = new improvedwban::WBANAckPacket("WBAN-Ack");
        ack->setSourceId(nodeId);
        ack->setDestinationId(pkt->getSourceId());
        ack->setSequenceNumber(seqNum++);
        ack->setCreationTime(simTime());
        ack->setAcknowledgedSeqNum(pkt->getSequenceNumber());
        
        // 设置新的Frame Control字段
        ack->setFrameType(improvedwban::CONTROL_FRAME);                                                     // ACK帧是控制帧类型
        ack->setFrameSubtype(improvedwban::CTRL_I_ACK);                                                     // 即时确认子类型
        // 为ACK帧设置长度
        const int ACK_PACKET_BYTES = 6; // 假设ACK帧固定为6字节
        ack->setByteLength(ACK_PACKET_BYTES);

        // 发送确认帧
        sendDown(ack);
        
        // 计算端到端延迟
        simtime_t delay = simTime() - pkt->getCreationTime();
        EV << "[DATA_PKT] Node " << nodeId << " Calculating end-to-end delay: simTime=" << simTime() 
           << ", creationTime=" << pkt->getCreationTime() << ", delay=" << delay << endl;
        
        // 添加更多调试信息
        if (_isnan(delay.dbl())) {
            EV_ERROR << "[DATA_PKT] Node " << nodeId << " ERROR: End-to-end delay is NaN! simTime=" << simTime()
                    << ", creationTime=" << pkt->getCreationTime() << endl;
        } else if (delay < 0) {
            EV_ERROR << "[DATA_PKT] Node " << nodeId << " ERROR: End-to-end delay is negative! simTime=" << simTime()
                    << ", creationTime=" << pkt->getCreationTime() << ", delay=" << delay << endl;
        } else {
            EV << "[DATA_PKT] Node " << nodeId << " End-to-end delay calculated successfully: " << delay << endl;
        }
        
        // 注意：endToEndDelaySignal信号现在在handleAck函数中发送，而不是在这里
        
        // 向上层发送数据包
        sendUp(pkt);
    } else {
        EV << "[DATA_PKT] Node " << nodeId << " Packet (SeqNum=" << pkt->getSequenceNumber()
           << ") is NOT for this node (DestId=" << pkt->getDestinationId()
           << ", MyId=" << nodeId << "). Deleting." << endl;
        delete pkt;
    }
}

// 3.6. 处理接收到的ACK帧
void IEEE802_15_6Mac::handleAck(improvedwban::WBANAckPacket *pkt) {
    // 收到空ACK
    if (!pkt) { 
        EV_ERROR << "[ACK_PKT] Node " << nodeId << ": Received NULL ACK packet. Ignoring." << endl;
        return; 
    }
    
    // 检查ACK包长度是否有效
    if (pkt->getByteLength() <= 0) {
        EV_ERROR << "[ACK_PKT] Node " << nodeId << ": Received ACK packet with invalid length: " 
                << pkt->getByteLength() << ". Ignoring." << endl;
        delete pkt;
        return;
    }
    
    // 检查源ID和目的ID是否有效
    if (pkt->getSourceId() < 0 || pkt->getDestinationId() < 0) {
        EV_ERROR << "[ACK_PKT] Node " << nodeId << ": Received ACK packet with invalid source ID (" 
                << pkt->getSourceId() << ") or destination ID (" << pkt->getDestinationId() << "). Ignoring." << endl;
        delete pkt;
        return;
    }
    
    // 检查是否是自己发送的ACK包（由于Hub广播导致的回环）
    if (pkt->getSourceId() == nodeId) {
        EV << "[ACK_PKT] Node " << nodeId << ": Received own ACK packet (source ID matches my ID). Ignoring to avoid loop." << endl;
        delete pkt;
        return;
    }
    
    // ACK帧时间戳同步：计算时钟偏移
    if (pkt->getDestinationId() == nodeId) {
        // 计算时钟偏移：D = Hub发送时间 - 节点接收时间
        simtime_t hubSendTime = pkt->getCreationTime();
        simtime_t nodeReceiveTime = simTime();
        
        // 检查时间戳是否有效
        if (hubSendTime <= 0 || hubSendTime > nodeReceiveTime) {
            EV_WARN << "[ACK_PKT] Node " << nodeId << ": Invalid timestamp in ACK packet. Hub send time: " 
                   << hubSendTime << ", Node receive time: " << nodeReceiveTime << ". Skipping synchronization." << endl;
        } else {
            simtime_t clockOffset = hubSendTime - nodeReceiveTime;
            
            // 计算同步间隔内的最大漂移 Dn = SIn × HubClockPPM
            simtime_t timeSinceLastSync = nodeReceiveTime - lastSyncTime;
            simtime_t maxDrift = nominalSyncInterval.dbl() * hubClockPPM * 1e-6;  // 转换ppm为实际时间
            
            // 更新本地时钟偏移（使用加权平均，平滑调整）
            localClockOffset = 0.7 * localClockOffset + 0.3 * clockOffset;
            
            // 计算分布式guard时间：GTn = GT0 + 2×Dn
            // GT0 = pSIFS + pExtraIFS + mClockResolution
            simtime_t GT0 = pSIFS + pExtraIFS + mClockResolution;
            simtime_t Dn = std::max(fabs(localClockOffset.dbl()), maxDrift.dbl());  // 取实际偏移和最大漂移中的较大值
            
            // 如果同步间隔超过标称同步间隔，计算额外guard时间
            if (timeSinceLastSync > nominalSyncInterval) {
                simtime_t excessTime = timeSinceLastSync - nominalSyncInterval;
                simtime_t Da = excessTime.dbl() * hubClockPPM * 1e-6;  // 额外漂移
                extraGuardTime = 2 * Da;  // GTa = 2×Da
                EV << "[ACK_SYNC] Node " << nodeId << ": Excess sync interval detected. "
                   << "Time since last sync: " << timeSinceLastSync 
                   << ", Excess time: " << excessTime
                   << ", Extra guard time: " << extraGuardTime << "s" << endl;
            } else {
                extraGuardTime = SimTime(0, SIMTIME_US);
            }
            
            distributedGuardTime = GT0 + 2 * Dn + extraGuardTime;
            
            // 更新同步状态
            isSynchronized = true;
            lastSyncTime = nodeReceiveTime;
            syncFailureCount = 0;  // 重置同步失败计数
            
            EV << "[ACK_SYNC] Node " << nodeId << ": Clock synchronization via ACK frame." << endl;
            EV << "[ACK_SYNC] Hub send time: " << hubSendTime << ", Node receive time: " << nodeReceiveTime << endl;
            EV << "[ACK_SYNC] Clock offset: " << localClockOffset << ", Distributed guard time: " << distributedGuardTime << endl;
            EV << "[ACK_SYNC] Time since last sync: " << timeSinceLastSync << "s" << endl;
        }
    }
    
    // 检查是否是发给当前节点的ACK
    if (pkt->getDestinationId() == nodeId) {
        // 检查队列是否为空
        if (dataQueue.empty()) {
            EV_WARN << "[ACK_PKT] Node " << nodeId << ": Received ACK for sequence " << pkt->getAcknowledgedSeqNum() 
                   << " but data queue is empty. Ignoring." << endl;
            delete pkt;
            return;
        }
        
        // 检查队头数据包的序列号是否匹配
        improvedwban::WBANDataPacket* frontPkt = dataQueue.front();
        if (!frontPkt) {
            EV_ERROR << "[ACK_PKT] Node " << nodeId << ": NULL packet at front of queue. Removing it." << endl;
            dataQueue.pop();
            delete pkt;
            return;
        }
        
        if (frontPkt->getSequenceNumber() != pkt->getAcknowledgedSeqNum()) {
            EV_WARN << "[ACK_PKT] Node " << nodeId << ": Received ACK for sequence " << pkt->getAcknowledgedSeqNum() 
                   << " but queue front has sequence " << frontPkt->getSequenceNumber() << ". Ignoring." << endl;
            delete pkt;
            return;
        }
        
        // 收到确认，取消重传定时器
        if (retransmissionTimer->isScheduled()) {
            cancelEvent(retransmissionTimer);
            EV << "[ACK_PKT] Node " << nodeId << ": Cancelled retransmission timer for acknowledged packet." << endl;
        }
        
        // 获取队头数据包，并记录其优先级，因为我们马上要删除它了
        improvedwban::WBANDataPacket* ackedPkt = frontPkt; // 使用已经获取的指针
        int seqNum = ackedPkt->getSequenceNumber();
        int priority = ackedPkt->getFrameSubtype();
        
        EV << "[ACK_PKT] Node " << nodeId << ": ACK received for Seq=" << seqNum << " (Priority=" << priority << "). Packet sent successfully." << endl;
        
        // 计算端到端延迟（从数据包创建到收到ACK的时间）
        simtime_t delay = simTime() - ackedPkt->getCreationTime();
        EV << "[ACK_PKT] Node " << nodeId << " Calculating end-to-end delay: simTime=" << simTime() 
           << ", creationTime=" << ackedPkt->getCreationTime() << ", delay=" << delay << endl;
        
        // 添加更多调试信息
        if (_isnan(delay.dbl())) {
            EV_ERROR << "[ACK_PKT] Node " << nodeId << " ERROR: End-to-end delay is NaN! simTime=" << simTime()
                    << ", creationTime=" << ackedPkt->getCreationTime() << endl;
        } else if (delay < 0) {
            EV_ERROR << "[ACK_PKT] Node " << nodeId << " ERROR: End-to-end delay is negative! simTime=" << simTime()
                    << ", creationTime=" << ackedPkt->getCreationTime() << ", delay=" << delay << endl;
        } else {
            EV << "[ACK_PKT] Node " << nodeId << " End-to-end delay calculated successfully: " << delay << endl;
            // 只有非Hub节点才发送端到端延迟信号
            if (!isHub) {
                EV << "[ACK_PKT] Node " << nodeId << " 发送端到端延迟信号: " << delay << endl;
                emit(endToEndDelaySignal, delay);
            }
        }
        
        // 从 retransmissionCount 清除已成功发送数据包的所有相关状态
        retransmissionCount.erase(seqNum);
        backoffCounter.erase(seqNum);

        delete ackedPkt;
        dataQueue.pop();
        
        emit(queueLengthSignal, (long)dataQueue.size());
        
        // 一个包处理完了，马上尝试处理下一个！
        if (priority >= 8) {
            EV << "[ACK_PKT] Node " << nodeId << ": The acknowledged packet was a high-priority polled response." << endl;
            // 检查队列是否还有其他高优先级数据包可以立即发送
            if (!dataQueue.empty()) {
                improvedwban::WBANDataPacket* frontPkt = dataQueue.front();
                if (frontPkt && frontPkt->getFrameSubtype() >= 8) {
                    EV << "[ACK_PKT] Node " << nodeId << ": Found another high-priority packet (Seq=" 
                       << frontPkt->getSequenceNumber() << ") in queue. Sending it immediately." << endl;
                    // 如果是，立即发送下一个，继续利用轮询机会
                    sendPolledData();
                } else {
                    EV << "[ACK_PKT] Node " << nodeId << ": No more high-priority packets to send now. Returning to normal operation." << endl;
                    // 如果没有更多高优先级包，则尝试正常的CSMA/CA流程（虽然可能因为阶段不对而什么都不做）
                    tryInitiateTransmission();
                }
            } else {
                EV << "[ACK_PKT] Node " << nodeId << ": Queue is empty. Returning to normal operation." << endl;
                tryInitiateTransmission();
            }
        } else {
            // 如果是普通的CSMA/CA包，则按原计划开始下一次传输尝试
            tryInitiateTransmission();
        }
    } else {
        EV << "[ACK_PKT] Node " << nodeId << ": Received a stray ACK for destination " << pkt->getDestinationId() 
           << " (my ID: " << nodeId << "). Ignoring." << endl;
    }
    
    // 删除ACK包
    delete pkt;
}

// 3.7. 处理管理帧
void IEEE802_15_6Mac::handleManagementPacket(improvedwban::WBANManagementPacket *pkt) {
    // 检查数据包有效性
    if (!pkt) {
        EV_ERROR << "[MGMT_PKT] Node " << nodeId << ": Received NULL management packet. Ignoring." << endl;
        return;
    }
    
    // --- 新增: Node响应轮询的逻辑 ---
    if (!isHub && pkt->getDestinationId() == nodeId) {
         EV << "[POLL_RESPONSE] Node " << nodeId << ": SUCCESS! Received a POLL from Hub " << pkt->getSourceId() << "." << endl;
         
         // 更新轮询接收统计
         pollReceivedCount++;
         emit(pollReceivedSignal, pollReceivedCount);

        // T-Poll帧时间戳同步：计算时钟偏移
        if (pkt->getFrameSubtype() == improvedwban::CTRL_T_POLL) {
            // 添加边界检查，确保时间戳有效
            simtime_t hubSendTime = pkt->getAllocationTimestamp();
            simtime_t nodeReceiveTime = simTime();
            
            if (hubSendTime >= 0 && nodeReceiveTime >= hubSendTime) {
                simtime_t clockOffset = hubSendTime - nodeReceiveTime;
                
                // 计算同步间隔内的最大漂移 Dn = SIn × HubClockPPM
                simtime_t timeSinceLastSync = nodeReceiveTime - lastSyncTime;
                simtime_t maxDrift = nominalSyncInterval.dbl() * hubClockPPM * 1e-6;  // 转换ppm为实际时间
                
                // 更新本地时钟偏移
                localClockOffset = clockOffset;
                
                // 计算分布式guard时间：GTn = GT0 + 2×Dn
                // GT0 = pSIFS + pExtraIFS + mClockResolution
                simtime_t GT0 = pSIFS + pExtraIFS + mClockResolution;
                simtime_t Dn = std::max(fabs(clockOffset.dbl()), maxDrift.dbl());  // 取实际偏移和最大漂移中的较大值
                
                // 如果同步间隔超过标称同步间隔，计算额外guard时间
                if (timeSinceLastSync > nominalSyncInterval) {
                    simtime_t excessTime = timeSinceLastSync - nominalSyncInterval;
                    simtime_t Da = excessTime.dbl() * hubClockPPM * 1e-6;  // 额外漂移
                    extraGuardTime = 2 * Da;  // GTa = 2×Da
                    EV << "[T-POLL_SYNC] Node " << nodeId << ": Excess sync interval detected. "
                       << "Time since last sync: " << timeSinceLastSync 
                       << ", Excess time: " << excessTime
                       << ", Extra guard time: " << extraGuardTime << "s" << endl;
                } else {
                    extraGuardTime = SimTime(0, SIMTIME_US);
                }
                
                distributedGuardTime = GT0 + 2 * Dn + extraGuardTime;
                
                // 更新同步状态
                isSynchronized = true;
                lastSyncTime = nodeReceiveTime;
                syncFailureCount = 0;  // 重置同步失败计数
                
                EV << "[T-POLL_SYNC] Node " << nodeId << ": Clock synchronization via T-Poll frame." << endl;
                EV << "[T-POLL_SYNC] Hub send time: " << hubSendTime << ", Node receive time: " << nodeReceiveTime << endl;
                EV << "[T-POLL_SYNC] Clock offset: " << clockOffset << ", Distributed guard time: " << distributedGuardTime << endl;
                EV << "[T-POLL_SYNC] Time since last sync: " << timeSinceLastSync << "s" << endl;
            } else {
                EV_ERROR << "[T-POLL_SYNC] Node " << nodeId << ": Invalid timestamps in T-Poll frame. Hub send time: " 
                        << hubSendTime << ", Node receive time: " << nodeReceiveTime << ". Skipping synchronization." << endl;
            }
        }

        // 检查队列是否为空
        if (dataQueue.empty()) {
            EV << "[POLL_RESPONSE] Node " << nodeId << ": Queue is empty. Nothing to send in response to poll." << endl;
            delete pkt;
            return;
        }
        
        // 检查队列头部数据包有效性
        improvedwban::WBANDataPacket *dataPkt = dataQueue.front();
        if (!dataPkt) {
            EV_ERROR << "[POLL_RESPONSE] Node " << nodeId << ": NULL packet at front of queue. Removing it." << endl;
            dataQueue.pop();
            delete pkt;
            return;
        }

        // 根据轮询类型处理不同的轮询机制
        if (pkt->getPollType() == improvedwban::TYPE_I_ALLOCATION) {
            // Type-I轮询：基于时隙的即时分配
            EV << "[POLL_RESPONSE] Node " << nodeId << " received TYPE-I POLL (slot-based allocation)" << endl;
            
            // 检查数据包优先级
            if (dataPkt->getFrameSubtype() >= 8) {
                EV << "[POLL_RESPONSE] Node " << nodeId << ": Priority is >= 8. This is a POLLABLE packet. Preparing to send in allocated slots!" << endl;
                
                // 获取分配参数
                int currentSlot = pkt->getCurrentAllocationSlot();
                int endSlot = pkt->getPollPostWindow();
                int allocatedSlots = pkt->getAllocationDurationSlots();
                
                // 检查分配参数有效性
                if (currentSlot < 0 || endSlot < currentSlot || allocatedSlots <= 0) {
                    EV_ERROR << "[POLL_RESPONSE] Node " << nodeId << ": Invalid allocation parameters. currentSlot=" 
                            << currentSlot << ", endSlot=" << endSlot << ", allocatedSlots=" << allocatedSlots << endl;
                    delete pkt;
                    return;
                }
                
                EV << "[POLL_RESPONSE] Node " << nodeId << " allocated slots " << currentSlot << " to " 
                   << endSlot << " (" << allocatedSlots << " slots total)" << endl;
                
                // 检查超帧开始时间和时隙长度有效性
                if (superframeStartTime < 0) {
                    EV_ERROR << "[POLL_RESPONSE] Node " << nodeId << ": Invalid superframe start time: " 
                            << superframeStartTime << ". Cannot calculate allocation start time." << endl;
                    delete pkt;
                    return;
                }
                
                if (allocationSlotLength <= 0) {
                    EV_ERROR << "[POLL_RESPONSE] Node " << nodeId << ": Invalid allocation slot length: " 
                            << allocationSlotLength << ". Using default 1ms." << endl;
                    allocationSlotLength = 0.001; // 默认1ms
                }
                
                // 计算分配开始时间
                simtime_t allocationStartTime = superframeStartTime + currentSlot * allocationSlotLength;
                
                // 如果分配时间还未到，等待
                if (simTime() < allocationStartTime) {
                    EV << "[POLL_RESPONSE] Node " << nodeId << " waiting for allocated slot to start" << endl;
                    
                    // 检查pollTimer是否已调度
                    if (pollTimer->isScheduled()) {
                        EV_WARN << "[POLL_RESPONSE] Node " << nodeId << ": pollTimer is already scheduled. Cancelling it." << endl;
                        cancelEvent(pollTimer);
                    }
                    
                    scheduleAt(allocationStartTime, pollTimer);
                } else {
                    // 立即发送高优先级数据
                    sendPolledData();
                }
            } else {
                EV << "[POLL_RESPONSE] Node " << nodeId << ": Packet is NOT for polled access (Priority < 8). Doing nothing." << endl;
            }
        }
        else if (pkt->getPollType() == improvedwban::TYPE_II_ALLOCATION) {
            // Type-II轮询：基于帧数的即时分配
            EV << "[POLL_RESPONSE] Node " << nodeId << " received TYPE-II POLL (frame-based allocation)" << endl;
            
            // 检查数据包优先级
            if (dataPkt->getFrameSubtype() >= 8) {
                EV << "[POLL_RESPONSE] Node " << nodeId << ": Priority is >= 8. This is a POLLABLE packet. Preparing to send with frame limit!" << endl;
                
                // 获取分配参数
                int maxFrames = pkt->getMaxFramesToSend();
                int allocatedFrames = pkt->getAllocationDurationFrames();
                
                // 检查分配参数有效性
                if (maxFrames <= 0 || allocatedFrames <= 0) {
                    EV_ERROR << "[POLL_RESPONSE] Node " << nodeId << ": Invalid frame allocation parameters. maxFrames=" 
                            << maxFrames << ", allocatedFrames=" << allocatedFrames << endl;
                    delete pkt;
                    return;
                }
                
                EV << "[POLL_RESPONSE] Node " << nodeId << " allocated " << allocatedFrames 
                   << " frames (max: " << maxFrames << ")" << endl;
                
                // 立即发送高优先级数据（Type-II轮询不需要等待特定时隙）
                sendPolledData();
            } else {
                EV << "[POLL_RESPONSE] Node " << nodeId << ": Packet is NOT for polled access (Priority < 8). Doing nothing." << endl;
            }
        }
        else if (pkt->getPollType() == improvedwban::FUTURE_ALLOCATION) {
            // 未来分配轮询
            EV << "[POLL_RESPONSE] Node " << nodeId << " received FUTURE ALLOCATION POLL" << endl;
            
            // 检查数据包优先级
            if (dataPkt->getFrameSubtype() >= 8) {
                // 获取未来分配参数
                int nextSuperframe = pkt->getNextSuperframe();
                int allocationStartSlot = pkt->getPollPostWindow();
                int allocatedSlots = pkt->getAllocationDurationSlots();
                
                // 检查分配参数有效性
                if (nextSuperframe < 0 || allocationStartSlot < 0 || allocatedSlots <= 0) {
                    EV_ERROR << "[POLL_RESPONSE] Node " << nodeId << ": Invalid future allocation parameters. nextSuperframe=" 
                            << nextSuperframe << ", allocationStartSlot=" << allocationStartSlot 
                            << ", allocatedSlots=" << allocatedSlots << endl;
                    delete pkt;
                    return;
                }
                
                EV << "[POLL_RESPONSE] Node " << nodeId << " allocated future slots " << allocationStartSlot 
                   << " to " << (allocationStartSlot + allocatedSlots) << " in superframe " << nextSuperframe << endl;
                
                // 检查超帧开始时间、信标周期和时隙长度有效性
                if (superframeStartTime < 0) {
                    EV_ERROR << "[POLL_RESPONSE] Node " << nodeId << ": Invalid superframe start time: " 
                            << superframeStartTime << ". Cannot calculate future allocation start time." << endl;
                    delete pkt;
                    return;
                }
                
                if (beaconPeriodLength <= 0) {
                    EV_ERROR << "[POLL_RESPONSE] Node " << nodeId << ": Invalid beacon period length: " 
                            << beaconPeriodLength << ". Using default 32ms." << endl;
                    beaconPeriodLength = 0.032; // 默认32ms
                }
                
                if (allocationSlotLength <= 0) {
                    EV_ERROR << "[POLL_RESPONSE] Node " << nodeId << ": Invalid allocation slot length: " 
                            << allocationSlotLength << ". Using default 1ms." << endl;
                    allocationSlotLength = 0.001; // 默认1ms
                }
                
                // 计算未来分配开始时间
                simtime_t futureAllocationStartTime = superframeStartTime + 
                                                     nextSuperframe * beaconPeriodLength + 
                                                     allocationStartSlot * allocationSlotLength;
                
                // 检查未来分配时间是否有效
                if (futureAllocationStartTime <= simTime()) {
                    EV_ERROR << "[POLL_RESPONSE] Node " << nodeId << ": Future allocation time is in the past: " 
                            << futureAllocationStartTime << ". Current time: " << simTime() << endl;
                    delete pkt;
                    return;
                }
                
                EV << "[POLL_RESPONSE] Node " << nodeId << " scheduling data transmission for future allocation" << endl;
                
                // 检查pollTimer是否已调度
                if (pollTimer->isScheduled()) {
                    EV_WARN << "[POLL_RESPONSE] Node " << nodeId << ": pollTimer is already scheduled. Cancelling it." << endl;
                    cancelEvent(pollTimer);
                }
                
                scheduleAt(futureAllocationStartTime, pollTimer);
            } else {
                EV << "[POLL_RESPONSE] Node " << nodeId << ": Packet is NOT for polled access (Priority < 8). Doing nothing." << endl;
            }
        }
        else {
            // 其他类型的轮询帧（默认处理）
            EV << "[POLL_RESPONSE] Node " << nodeId << " received POLL of unknown type: " << pkt->getPollType() << endl;

            if (dataPkt->getFrameSubtype() >= 8) {
                EV << "[POLL_RESPONSE] Node " << nodeId << ": Priority is >= 8. This is a POLLABLE packet. Responding to poll now!" << endl;
                sendPolledData(); // 直接发送数据，绕过CSMA/CA
            } else {
                EV << "[POLL_RESPONSE] Node " << nodeId << ": Packet is NOT for polled access (Priority < 8). Doing nothing." << endl;
            }
        }
    } else {
        EV << "[POLL_RESPONSE] Node " << nodeId << ": Received a Management Packet, but it's not a poll for me (isHub="
           << isHub << ", destId=" << pkt->getDestinationId() << "). Ignoring." << endl;
    }
    // delete pkt;
}

//==============================================================================
// 4. CSMA/CA 核心机制 (Core CSMA/CA Mechanism)
//==============================================================================

// 4.1. CSMA/CA机制的初始化函数
void IEEE802_15_6Mac::tryInitiateTransmission() {
    EV << "[DEBUG-4] Node " << nodeId << ": tryInitiateTransmission() called." << endl; // <-- 添加此行
    
    // 
    // 如果1.当前正在发送数据；2.如果当前正在进行载波监听；3.如果当前数据包已经安排了退避计数器backoffTimer；4.如果当前队列为空
    // 则不能进行初始化操作
    //

    if (isTransmitting || isSensing || backoffTimer->isScheduled() || dataQueue.empty()) {
         EV << "[DEBUG-4A] Node " << nodeId << ": Aborting tryInitiateTransmission. Reason: "
           << "isTransmitting=" << isTransmitting << ", isSensing=" << isSensing
           << ", backoffTimer->isScheduled()=" << backoffTimer->isScheduled()
           << ", dataQueue.empty()=" << dataQueue.empty() << endl; // <-- 添加此行
        return;
    }

    // 双重检查队列是否为空
    if (dataQueue.empty()) {
        EV << "[DEBUG-4A] Node " << nodeId << ": Aborting tryInitiateTransmission. Reason: dataQueue is empty." << endl;
        return;
    }

    // 仅查看队头的数据包
    improvedwban::WBANDataPacket *pkt = dataQueue.front();
    
    // 检查数据包指针有效性
    if (!pkt) {
        EV_ERROR << "[TRY_INITIATE_TRANSMISSION] Node " << nodeId << ": NULL packet at front of queue. Removing it." << endl;
        dataQueue.pop();
        return;
    }

    // ========================[核心修复]========================
    // 如果队头是高优先级（轮询）包，则CSMA/CA引擎不应处理它。
    // 它必须等待Hub的轮询指令，而不是自己尝试发送。
    if (pkt->getFrameSubtype() >= 8) {
        EV << "Node " << nodeId << ": Packet Seq=" << pkt->getSequenceNumber()
           << " is a high-priority (polled) packet (UP=" << pkt->getFrameSubtype()
           << "). CSMA/CA engine will ignore it. Waiting for a poll from Hub." << endl;
        return; // 直接返回，让数据包在队列中等待轮询
    }
    // ==========================================================

    // 检查当前阶段是否允许发送该优先级的包
    if (canSendInCurrentPhase(pkt->getFrameSubtype())) {
        EV << "[DEBUG-5A] Node " << nodeId << ": Packet Seq=" << pkt->getSequenceNumber() << " CAN be sent in current phase (" << currentPhase << ")." << endl; // <-- 添加此行
        int seqNum = pkt->getSequenceNumber();

        // 
        // 判断是"恢复"一个被冻结的退避，还是"开始"一个全新的退避
        // backoffCounter是一个map结构<seqNum, backoffCount>，如果该值存在，则代表该MAC帧进行过退避过程（退避计数器）
        // 此时只需要进行解冻就可以了
        //

        if (backoffCounter.count(seqNum)) {
            EV_DETAIL << "Node " << nodeId << ": Resuming frozen backoff process for Seq=" << seqNum << " at count " << backoffCounter[seqNum] << endl;
            // 直接在下一个时隙开始时安排信道检查，以从上次中断的地方继续
            if (!backoffTimer->isScheduled()) {
                scheduleAt(getNextSlotBoundary(), backoffTimer);
            }
        } else {
            // **开始新退避逻辑**：这是一个全新的发送尝试
            EV << "[DEBUG-5B] Node " << nodeId << ": Packet Seq=" << seqNum << " CANNOT be sent. Current phase is " << currentPhase << " which is not allowed." << endl; // <-- 添加此行
            startBackoffProcess();
        }
    } else {
        EV_DETAIL << "Node " << nodeId << ": Packet (Seq=" << pkt->getSequenceNumber()
                  << ") cannot be sent in current phase " << currentPhase << ". Waiting." << endl;
    }
}

// 4.2. 首次开始新的退避流程
void IEEE802_15_6Mac::startBackoffProcess() {
    if (dataQueue.empty()) return;                      // 检查队列长度是否为空，为空返回
    
    // 获取队头数据包
    improvedwban::WBANDataPacket* pkt = dataQueue.front();
    
    // 检查数据包指针有效性
    if (!pkt) {
        EV_ERROR << "[START_BACKOFF_PROCESS] Node " << nodeId << ": NULL packet at front of queue. Removing it." << endl;
        dataQueue.pop();
        return;
    }
    
    EV << "[DEBUG-6] Node " << nodeId << ": startBackoffProcess() called for Seq=" << pkt->getSequenceNumber() << "." << endl; // <-- 添加此行

    // 组装pkt帧
    int seqNum = pkt->getSequenceNumber();
    int priority = pkt->getFrameSubtype();
    
    // 如果是第一次发送这个包，初始化重传计数和CW
    if (retransmissionCount.find(seqNum) == retransmissionCount.end()) {
        retransmissionCount[seqNum] = 0;
        // 按照IEEE 802.15.6标准，初始化CW为CWmin
        if(currentCW.find(priority) == currentCW.end()) {
            currentCW[priority] = CWminPriority[priority];
        }
    }

    // 按照IEEE 802.15.6标准，从[1, CW]范围内随机采样退避计数器
    backoffCounter[seqNum] = intuniform(1, currentCW[priority]);
    
    EV_DETAIL << "Node " << nodeId << ": Starting backoff process for Seq=" << seqNum
            << ". CW=" << currentCW[priority]
            << ", BackoffCounter=" << backoffCounter[seqNum] << endl;
    
    if (backoffTimer->isScheduled()) {
        cancelEvent(backoffTimer);
    }
    scheduleAt(getNextSlotBoundary(), backoffTimer);
}

// 4.3. 在时隙边界检查信道并准备递减计数器
void IEEE802_15_6Mac::processBackoffSlot() {
       if (dataQueue.empty()) {
        EV_WARN << "Node " << nodeId << ": processBackoffSlot triggered, but data queue is empty. Aborting backoff.";
        return; // 如果队列空了，停止退避
    }

    improvedwban::WBANDataPacket *pkt = dataQueue.front();
    if (!pkt) {
        EV_ERROR << "[PROCESS_BACKOFF_SLOT] Node " << nodeId << ": NULL packet at front of queue. Removing it." << endl;
        dataQueue.pop();
        return;
    }

    if (!canSendInCurrentPhase(pkt->getFrameSubtype())) {
        EV_WARN << "Node " << nodeId << ": Phase changed to " << currentPhase
                << ". Aborting current backoff process for Seq=" << pkt->getSequenceNumber() << endl;
        // 中止退避，但不要丢弃包或状态，等待下一个合适阶段再由 tryInitiateTransmission 重新发起
        return;
    }

    // 检查是否正在传输或已经在进行CCA
    if (isTransmitting || isSensing) {
        EV << "Node " << nodeId << ": Backoff slot tick, but channel is busy (transmitting/sensing). Freezing and retrying next slot." << endl;
        // 这是冻结逻辑：直接安排下一个时隙的检测
        if(backoffTimer->isScheduled()) cancelEvent(backoffTimer);
        scheduleAt(getNextSlotBoundary(), backoffTimer);
        return;
    }

    // 信道看起来空闲，可以开始真正的CCA
    EV_DETAIL << "Node " << nodeId << ": Backoff slot tick. Starting carrier sensing (CCA)." << endl;
    startCarrierSensing();
}

// 4.4 开始CCA检测
void IEEE802_15_6Mac::startCarrierSensing() {
    // 检查当前节点是否处于载波监听状态
    if (isSensing) {
        EV << "Node " << nodeId << " already sensing, deferring" << endl;
        if (backoffTimer->isScheduled()) {
            cancelEvent(backoffTimer);
        }
        // 重新调度backoffTimer
        scheduleAt(simTime() + allocationSlotLength, backoffTimer);
        return;
    }
    // 如果没有处于载波监听状态，将isSensing设置为true
    isSensing = true;
    
    // 执行CCA检测
    scheduleAt(simTime() + ccaDuration, ccaTimer);
    
    EV << "Node " << nodeId << " started CCA, will complete at " 
       << simTime() + ccaDuration << endl;
}

// 4.5. 执行CCA检测
void IEEE802_15_6Mac::performCCA() {
    // 获取当前RSSI值
    double currentRSSI = getCurrentRSSI();
    
    // 判断信道是否忙碌
    bool channelBusy = (currentRSSI > ccaThreshold);
    
    EV << "Node " << nodeId << " CCA result: RSSI=" << currentRSSI 
       << " dBm, threshold=" << ccaThreshold << " dBm, busy=" << channelBusy << endl;
    
    // 更新统计
    if (channelBusy) {
        channelBusyCount++;
    } else {
        channelIdleCount++;
    }
    
    emit(channelStateSignal, channelBusy ? 1 : 0);
    
    // 处理CCA结果
    handleCCAResult(channelBusy);
}

// 4.6. 处理CCA结果
void IEEE802_15_6Mac::handleCCAResult(bool channelBusy) {
    isSensing = false;  // 载波监听结束
    
    if (channelBusy) {
         EV << "[DEBUG-7A] Node " << nodeId << ": CCA Result: CHANNEL BUSY. Deferring transmission." << endl; // <-- 添加此行
        deferTransmission();
    } else {
        EV << "[DEBUG-7B] Node " << nodeId << ": CCA Result: CHANNEL IDLE. Proceeding to decrement backoff counter." << endl; // <-- 添加此行
        channelIdle = true;
        decrementBackoffCounter(); // 只有信道空闲才递减
    }
}

// 4.7. 递减退避计数器
void IEEE802_15_6Mac::decrementBackoffCounter() {
    // 递减退避计数器
    if (dataQueue.empty()) return;
    
    improvedwban::WBANDataPacket *pkt = dataQueue.front();
    if (!pkt) {
        EV_ERROR << "[DECREMENT_BACKOFF] Node " << nodeId << ": NULL packet at front of queue. Removing it." << endl;
        dataQueue.pop();
        return;
    }
    
    int seqNum = pkt->getSequenceNumber();
    int priority = pkt->getFrameSubtype();
        
    if (backoffCounter.find(seqNum) == backoffCounter.end()) {
        EV << "Node " << nodeId << ": decrementBackoffCounter called for seq=" << seqNum 
                 << " but no backoff counter found. Aborting.";
        return;
    }

    // 按照IEEE 802.15.6标准，如果信道空闲，递减退避计数器
    if (backoffCounter[seqNum] > 0) {
        backoffCounter[seqNum]--;
        EV << "[DEBUG-8A] Node " << nodeId << ": Backoff counter decremented for Seq=" << seqNum << ". New value: " << backoffCounter[seqNum] << "." << endl;
    }

    // 如果退避计数器 = 0，则在下一个时隙进行发送
    if (backoffCounter[seqNum] == 0) {
        // ---- 新增的最后检查 ----
        if (!canSendInCurrentPhase(priority)) {
            EV << "[DEBUG-8B] Node " << nodeId << ": Backoff counter is ZERO! Preparing to send packet Seq=" << seqNum << "." << endl;
                    backoffCounter.erase(seqNum); 
            // if (backoffTimer->isScheduled()) cancelEvent(backoffTimer);
            // if (channelCheckTimer->isScheduled()) cancelEvent(channelCheckTimer);
            retransmissionCount.erase(seqNum);
            backoffCounter.erase(seqNum);
            return;
        }
        // ---- 检查结束 ----
        
        // [FIX] 退避结束，立即发送！
        EV << "Node " << nodeId << ": Backoff complete for Seq=" << seqNum << ". Sending packet." << endl;
        
        // 清理所有与退避相关的定时器，避免竞争条件
        if (backoffTimer->isScheduled()) cancelEvent(backoffTimer);

        sendFromQueue();
    } else {
        // --- 新逻辑: 安排下一个时隙的检测，对齐到全局网格 ---
        EV_DETAIL << "Node " << nodeId << ": Scheduling next backoff slot tick on the grid for Seq=" << seqNum << "." << endl;
        if (backoffTimer->isScheduled()) cancelEvent(backoffTimer);
        scheduleAt(getNextSlotBoundary(), backoffTimer);
    }
}

// 4.8. 信道检测-繁忙，推迟传输
void IEEE802_15_6Mac::deferTransmission() {
    // 标记信道忙
    channelIdle = false; 

    // 如果队列空，中止
    if (dataQueue.empty()) return;

    // 获取队头的数据包，
    improvedwban::WBANDataPacket* pkt = dataQueue.front();
    if (!pkt) {
        EV_ERROR << "[DEFER_TRANSMISSION] Node " << nodeId << ": NULL packet at front of queue. Removing it." << endl;
        dataQueue.pop();
        return;
    }
    
    int seqNum = pkt->getSequenceNumber();      // 获取数据包的序号
    int priority = pkt->getFrameSubtype();      // 获取数据包的优先级

    // 检查当前阶段是否允许重传
    if (!canSendInCurrentPhase(priority)) {
        EV << "Node " << nodeId << ": Cannot defer transmission in current phase " << currentPhase 
           << ". Waiting for next suitable phase." << endl;
        // 不进行退避，等待下一个合适的阶段
        return;
    }

    // 初始化重传计数（如果尚未初始化）
    if (retransmissionCount.find(seqNum) == retransmissionCount.end()) {
        retransmissionCount[seqNum] = 0;
    }
    
    // 按照IEEE 802.15.6标准，仅在偶数次重传失败时加倍CW
    if(currentCW.find(priority) == currentCW.end()) {
        currentCW[priority] = CWminPriority[priority];
    }
    
    // 仅当重传次数为偶数时（包括0）才加倍CW
    if (retransmissionCount[seqNum] % 2 == 0) {
        currentCW[priority] = std::min(2 * currentCW[priority], CWmaxPriority[priority]);
        EV << "Node " << nodeId << ": Even retransmission attempt (" << retransmissionCount[seqNum] 
           << "), doubling CW to " << currentCW[priority] << endl;
    } else {
        EV << "Node " << nodeId << ": Odd retransmission attempt (" << retransmissionCount[seqNum] 
           << "), keeping CW at " << currentCW[priority] << endl;
    }

    // 按照IEEE 802.15.6标准，从[1, CW]范围内随机采样退避计数器
    backoffCounter[seqNum] = intuniform(1, currentCW[priority]);

    EV << "Node " << nodeId << " deferred transmission. CW=" << currentCW[priority]
       << ", BackoffCounter=" << backoffCounter[seqNum] << endl;

    if (backoffTimer->isScheduled()) {
        cancelEvent(backoffTimer);
    }
    scheduleAt(getNextSlotBoundary(), backoffTimer);
}

//==============================================================================
// 5. 数据传输与重传 (Data Transmission & Retransmission)
//==============================================================================


// 5.1. 从队列头部发送数据包
void IEEE802_15_6Mac::sendFromQueue() {
    if(dataQueue.empty() || isTransmitting) return;
    
    // 添加额外的空指针检查
    if (dataQueue.empty()) {
        EV_ERROR << "[SEND_FROM_QUEUE] Node " << nodeId << ": Queue became empty after initial check. Aborting." << endl;
        return;
    }
    
    improvedwban::WBANDataPacket *pkt = dataQueue.front();
    if (!pkt) {
        EV_ERROR << "[SEND_FROM_QUEUE] Node " << nodeId << ": NULL packet at front of queue. Removing it." << endl;
        dataQueue.pop();
        return;
    }
    
    EV << "[DEBUG-9] Node " << nodeId << ": sendFromQueue() called. About to send packet Seq=" << pkt->getSequenceNumber() << " to physical layer." << endl;
    
    // 设置数据包离开队列的时间戳
    pkt->setQueueExitTime(simTime());
    
    // 注意：这里不应该再dup()了，因为我们是从队列里直接发送。
    // sendDown函数会传递所有权。
    // 但是，我们需要一个副本用于重传，所以这里逻辑需要小心。
    // 正确的做法是：发送时不要从队列中移除，收到ACK后再移除。
    
    sendDown(pkt->dup()); // 发送一个副本

    // 设置ACK超时定时器（注意：这里不再使用指数退避，而是使用固定的ACK超时）
    int seqNum = pkt->getSequenceNumber();
    if (retransmissionTimer->isScheduled()) {
        cancelEvent(retransmissionTimer);
    }
    // 使用固定的ACK超时时间，而不是指数退避
    scheduleAt(simTime() + baseRetransmissionTimeout, retransmissionTimer);

    EV << "Node " << nodeId << " sent packet seq=" << seqNum
       << ". Setting ACK timeout=" << baseRetransmissionTimeout << "s" << endl;
}

// 5.2. 处理重传超时
void IEEE802_15_6Mac::handleRetransmission(improvedwban::WBANDataPacket *pkt) {
    if (dataQueue.empty()) return;
    
    // 添加额外的空指针检查
    improvedwban::WBANDataPacket *frontPkt = dataQueue.front();
    if (!frontPkt) {
        EV_ERROR << "[HANDLE_RETRANSMISSION] Node " << nodeId << ": NULL packet at front of queue. Removing it." << endl;
        dataQueue.pop();
        return;
    }
    
    // 注意：由于可能不是优先队列，pkt参数不一定等于队头，但在这个场景下，超时总是针对队头的
    if (pkt != frontPkt) {
         EV_ERROR << "Retransmission timeout for a packet that is NOT at the head of the queue. This is an anomaly. Seq=" << pkt->getSequenceNumber();
         // 正常情况下这里应该有更复杂的处理，但根据现有逻辑，我们只处理队头
         return;
    }
    
    int seqNum = frontPkt->getSequenceNumber();
    int userPriority = frontPkt->getFrameSubtype();
    
    // 初始化重传计数
    if (retransmissionCount.find(seqNum) == retransmissionCount.end()) {
        retransmissionCount[seqNum] = 0;
    }

    // 未达到最大重传次数，更新重传次数，递增 + 1
    retransmissionCount[seqNum]++;

    // 检查是否超过最大重传次数（3次），执行丢包
    if (retransmissionCount[seqNum] > maxRetransmissions) {
        EV << "Node " << nodeId << " packet seq=" << seqNum
           << " exceeded max retransmissions, dropping." << endl;
        
        // 从队列中移除并删除
        delete dataQueue.front();
        dataQueue.pop();
        retransmissionCount.erase(seqNum);
        
        if(userPriority < 8) {
            backoffCounter.erase(seqNum);
            currentCW.erase(userPriority);
        }
        
        emit(packetDroppedSignal, 1L);
        emit(queueLengthSignal, (long)dataQueue.size());

        // 尝试发送队列中的下一个包
        tryInitiateTransmission();
        return;
    }

    // ========================[ 核心修复：分离处理逻辑 ]========================
    if (userPriority >= 8) {
        // 这是轮询数据包的重传逻辑
        EV << "Node " << nodeId << ": ACK timeout for POLLED packet Seq=" << seqNum
           << ". Retransmission attempt #" << retransmissionCount[seqNum] << "." << endl;
        
        EV << "Node " << nodeId << ": Polled packet Seq=" << seqNum
           << " will now wait for the NEXT POLL from the Hub to re-transmit. Taking no further action." << endl;
        
        // **关键：什么都不做！不调用 startBackoffProcess()**
        // 数据包会继续留在队列的顶部，等待下一次被 handleManagementPacket 挑选并发送。
    } 
    
    // 根据IEEE 802.15.6标准调整CW
    else{
        EV << "Node " << nodeId << ": ACK timeout for CSMA/CA packet Seq=" << seqNum
           << ". Retransmission attempt #" << retransmissionCount[seqNum] << "." << endl;
        
        // 检查当前阶段是否允许重传
        if (!canSendInCurrentPhase(userPriority)) {
            EV << "Node " << nodeId << ": Cannot retransmit in current phase " << currentPhase 
               << ". Waiting for next suitable phase." << endl;
            // 不进行重传，等待下一个合适的阶段
            return;
        }
        
        // 按照IEEE 802.15.6标准，仅在偶数次重传失败时加倍CW
        if(currentCW.find(userPriority) == currentCW.end()) {
            currentCW[userPriority] = CWminPriority[userPriority];
        }
        
        // 仅当重传次数为偶数时（包括0）才加倍CW
        if (retransmissionCount[seqNum] % 2 == 0) {
            currentCW[userPriority] = std::min(2 * currentCW[userPriority], CWmaxPriority[userPriority]);
            EV << "Node " << nodeId << ": Even retransmission attempt (" << retransmissionCount[seqNum] 
               << "), doubling CW to " << currentCW[userPriority] << endl;
        } else {
            EV << "Node " << nodeId << ": Odd retransmission attempt (" << retransmissionCount[seqNum] 
               << "), keeping CW at " << currentCW[userPriority] << endl;
        }
        
        // 按照IEEE 802.15.6标准，从[1, CW]范围内随机采样退避计数器
        backoffCounter[seqNum] = intuniform(1, currentCW[userPriority]);
        
        EV << "Node " << nodeId << ": Backoff counter set to " << backoffCounter[seqNum] 
           << " according to IEEE 802.15.6 standard." << endl;
        
        // 重新开始退避过程
        if (backoffTimer->isScheduled()) {
            cancelEvent(backoffTimer);
        }
        scheduleAt(getNextSlotBoundary(), backoffTimer);
    }
}

// 5.3. 将数据包发送到物理层
void IEEE802_15_6Mac::sendDown(improvedwban::WBANPacket *pkt) {
    // 设置传输状态
    isTransmitting = true;
    // 记录传输开始时间和队列退出时间
    if (auto dataPkt = dynamic_cast<improvedwban::WBANDataPacket*>(pkt)) {
        EV << "[SEND_DOWN] Node " << nodeId << " Sending packet with creationTime=" << dataPkt->getCreationTime() << endl;
        dataPkt->setQueueExitTime(simTime());
        dataPkt->setTransmissionStartTime(simTime());
    }
    // 向下层发送数据包
    emit(packetSentSignal, 1L);                                                     // 1L代表发送了一个长整型的数据包
    send(pkt, "lowerLayerOut");                                                     // 使用OMNeT++框架提供的通用传递方法，需要指定目标门名

    // 安排传输结束事件
    simtime_t duration = calculateTransmissionDuration(pkt);            // 计算帧传输的时间
    cMessage *txEndMsg = new cMessage("TX_END", TX_END);                          // 创建一个包含TX_END的消息
    scheduleAt(simTime() + duration, txEndMsg);                         // 在数据包传输完成之后，触发一个txEndMsg
}

// 5.4. 将数据包发送到应用层
void IEEE802_15_6Mac::sendUp(improvedwban::WBANPacket *pkt) {
    // 向上层发送数据包
    emit(packetReceivedSignal, 1L);
    send(pkt, "upperLayerOut");
}

// 5.5. 发送轮询的数据
void IEEE802_15_6Mac::sendPolledData() {
    // 检查队列和传输状态
    if (dataQueue.empty() || isTransmitting) {
        EV_ERROR << "[SEND_POLLED] Node " << nodeId << " tried to send polled data, but cannot. Queue empty? " << dataQueue.empty()
               << ", Transmitting? " << isTransmitting << endl;
        return;
    }

    // 检查队列头部数据包有效性
    improvedwban::WBANDataPacket *pkt = dataQueue.front();
    if (!pkt) {
        EV_ERROR << "[SEND_POLLED] Node " << nodeId << " has NULL packet at front of queue. Removing it." << endl;
        dataQueue.pop();
        return;
    }

    EV << "[SEND_POLLED] Node " << nodeId << " sending packet (Polled) Seq=" << pkt->getSequenceNumber() 
       << " with UserPriority=" << pkt->getFrameSubtype() << " to lower layer." << endl;
    
    // 检查当前轮询阶段，确保只在轮询阶段响应
    if (currentPhase != TYPE1_POLLED && currentPhase != TYPE2_POLLED) {
        EV_ERROR << "[SEND_POLLED] Node " << nodeId << " is not in polling phase (currentPhase=" 
                << currentPhase << "). Cannot send polled data." << endl;
        return;
    }
    
    // 检查超帧开始时间有效性
    if (superframeStartTime < 0) {
        EV_ERROR << "[SEND_POLLED] Node " << nodeId << " has invalid superframeStartTime: " 
                << superframeStartTime << ". Cannot calculate current slot." << endl;
        return;
    }
    
    // 应用guard时间补偿机制
    simtime_t guardTimeToApply;
    if (isHub) {
        // Hub使用集中式guard时间
        guardTimeToApply = centralizedGuardTime;
        EV << "[GUARD_TIME] Hub " << nodeId << " applying centralized guard time: " << guardTimeToApply << endl;
    } else {
        // 节点使用分布式guard时间
        guardTimeToApply = distributedGuardTime;
        
        // 检查同步状态，如果长时间未同步，增加额外的guard时间
        if (isSynchronized) {
            simtime_t timeSinceLastSync = simTime() - lastSyncTime;
            if (timeSinceLastSync > maxSyncInterval) {
                // 计算额外guard时间：GTa = 2×Da，其中Da = 超期时间×节点时钟PPM
                simtime_t excessTime = timeSinceLastSync - maxSyncInterval;
                simtime_t Da = excessTime.dbl() * nodeClockPPM * 1e-6;  // 转换ppm为实际时间
                simtime_t additionalGuardTime = 2 * Da;
                
                guardTimeToApply += additionalGuardTime;
                EV << "[GUARD_TIME] Node " << nodeId << " adding extra guard time due to sync timeout: " 
                   << additionalGuardTime << " (total: " << guardTimeToApply << ")" << endl;
                   
                // 增加同步失败计数
                syncFailureCount++;
                if (syncFailureCount > maxSyncFailures) {
                    EV_WARN << "[GUARD_TIME] Node " << nodeId << ": Sync failure count (" << syncFailureCount 
                           << ") exceeded maximum (" << maxSyncFailures << "). Resetting synchronization state." << endl;
                    isSynchronized = false;
                    localClockOffset = 0;
                }
            }
        } else {
            // 如果未同步，使用最大可能的guard时间
            simtime_t maxPossibleDrift = maxSyncInterval.dbl() * nodeClockPPM * 1e-6;
            guardTimeToApply = baseGuardTime + 2 * maxPossibleDrift;
            EV << "[GUARD_TIME] Node " << nodeId << " not synchronized, using maximum guard time: " << guardTimeToApply << endl;
        }
        
        EV << "[GUARD_TIME] Node " << nodeId << " applying distributed guard time: " << guardTimeToApply << endl;
    }
    
    // 检查guard时间有效性
    if (guardTimeToApply < 0) {
        EV_ERROR << "[SEND_POLLED] Node " << nodeId << " has invalid guard time: " << guardTimeToApply 
                << ". Resetting to 0." << endl;
        guardTimeToApply = 0;
    }
    
    // 根据轮询阶段设置数据包的发送参数
    if (currentPhase == TYPE1_POLLED) {
        // Type-I轮询：基于时隙的即时分配
        EV << "[SEND_POLLED] Node " << nodeId << " sending data in TYPE-I POLLED phase (slot-based allocation)" << endl;
        
        // 检查allocationSlotLength有效性
        if (allocationSlotLength <= 0) {
            EV_ERROR << "[SEND_POLLED] Node " << nodeId << " has invalid allocationSlotLength: " 
                    << allocationSlotLength << ". Using default 1ms." << endl;
            allocationSlotLength = 0.001; // 默认1ms
        }
        
        // 计算当前时隙
        int currentSlot = floor((simTime() - superframeStartTime) / allocationSlotLength);
        
        // 检查时隙计算结果有效性
        if (currentSlot < 0) {
            EV_ERROR << "[SEND_POLLED] Node " << nodeId << " calculated negative currentSlot: " 
                    << currentSlot << ". Resetting to 0." << endl;
            currentSlot = 0;
        }
        
        // 设置数据包的时隙信息
        pkt->setCurrentAllocationSlot(currentSlot);
        pkt->setAllocationTimestamp(simTime());
        
        // 如果guard时间大于0，则延迟发送
        if (guardTimeToApply > 0) {
            EV << "[GUARD_TIME] Node " << nodeId << " delaying transmission by guard time: " << guardTimeToApply << endl;
            
            // 创建一个自消息来延迟发送
            cMessage *guardTimer = new cMessage("guardTimer", GUARD_TIME_END);
            scheduleAt(simTime() + guardTimeToApply, guardTimer);
            
            // 保存要发送的数据包引用，以便在guard时间结束后发送
            pendingPolledPacket = pkt;
            return;
        }
    }
    else if (currentPhase == TYPE2_POLLED) {
        // Type-II轮询：基于帧数的即时分配
        EV << "[SEND_POLLED] Node " << nodeId << " sending data in TYPE-II POLLED phase (frame-based allocation)" << endl;
        
        // 设置数据包的帧信息
        pkt->setAllocationTimestamp(simTime());
        
        // Type-II轮询通常不需要等待特定时隙，但仍然需要应用guard时间
        if (guardTimeToApply > 0) {
            EV << "[GUARD_TIME] Node " << nodeId << " delaying transmission by guard time: " << guardTimeToApply << endl;
            
            // 创建一个自消息来延迟发送
            cMessage *guardTimer = new cMessage("guardTimer", GUARD_TIME_END);
            scheduleAt(simTime() + guardTimeToApply, guardTimer);
            
            // 保存要发送的数据包引用，以便在guard时间结束后发送
            pendingPolledPacket = pkt;
            return;
        }
    }
    
    // 检查重传定时器状态
    if (retransmissionTimer->isScheduled()) {
        EV_WARN << "[SEND_POLLED] Node " << nodeId << " retransmissionTimer is already scheduled. Cancelling it." << endl;
        cancelEvent(retransmissionTimer);
    }
    
    // 检查baseRetransmissionTimeout有效性
    if (baseRetransmissionTimeout <= 0) {
        EV_ERROR << "[SEND_POLLED] Node " << nodeId << " has invalid baseRetransmissionTimeout: " 
                << baseRetransmissionTimeout << ". Using default 0.01s." << endl;
        baseRetransmissionTimeout = 0.01; // 默认10ms
    }
    
    // 设置数据包的队列退出时间
    pkt->setQueueExitTime(simTime());
    
    // 如果没有guard时间或guard时间为0，立即发送
    sendDown(pkt->dup()); // 发送副本，原件保留直到收到ACK

    // 为这个轮询的数据包设置ACK超时
    double timeout = baseRetransmissionTimeout;
    scheduleAt(simTime() + timeout, retransmissionTimer);
    
    // 更新轮询发送统计
    polledDataSentCount++;
    emit(polledDataSentSignal, polledDataSentCount);
}

//==============================================================================
// 6. 辅助工具函数 (Utility & Helper Functions
//==============================================================================

// 6.1. 检查当前阶段是否允许发送指定优先级的包
bool IEEE802_15_6Mac::canSendInCurrentPhase(int priority) {
    switch (currentPhase) {
        case EAP1:
        case EAP2:
            return (priority == 7);
        case RAP1:
        case RAP2:
        case CAP:
            return (priority >= 0 && priority <= 7);
        case TYPE1_POLLED:  // 轮询阶段，节点不能主动发起CSMA/CA
        case TYPE2_POLLED:  // 只能被动响应Poll
            return false;
        default:    // 其他阶段不可以发送
            return false;
    }
}

// 6.2. 计算数据包的传输时间
simtime_t IEEE802_15_6Mac::calculateTransmissionDuration(improvedwban::WBANPacket *pkt) {
    // 计算传输持续时间
    int packetSize = pkt->getByteLength();                                          // 获取数据包的字节长度
    double dataRate = 250e3;  // 250 kbps (根据您的omnetpp.ini配置)

    // 传输时间 = 比特数 / 数据率
    simtime_t duration = (packetSize * 8.0) / dataRate;

    EV << "节点：" << nodeId << "计算的比特数为："
       << packetSize << " bytes: " << "，持续时间为：" << duration << " s" << endl;

    return duration;
}

// 6.3. 获取当前信道的RSSI值
double IEEE802_15_6Mac::getCurrentRSSI() {
    // 从Radio模块获取真实的RSSI值
    Radio *radioModule = dynamic_cast<Radio*>(getParentModule()->getSubmodule("radio"));
    return radioModule->getRSSI();
}

// 6.4. 判断信道是否忙碌
bool IEEE802_15_6Mac::isChannelBusy() {
    return getCurrentRSSI() > ccaThreshold;
}

// 6.5. Hub执行轮询的函数
void IEEE802_15_6Mac::executePolling() {
    // 检查是否仍在轮询阶段
    if (currentPhase != TYPE1_POLLED && currentPhase != TYPE2_POLLED) {
        EV_ERROR << "[POLL_EXEC] Hub " << nodeId << ": executePolling called, but currentPhase is " << currentPhase << " (not a polling phase). Stopping polling." << endl;
        return;
    }

    // 检查轮询节点列表是否为空
    if (polledNodeList.empty()) {
        EV_ERROR << "[POLL_EXEC] Hub " << nodeId << ": Polled node list is empty. Stopping polling." << endl;
        return;
    }

    // 检查当前轮询节点索引是否有效
    if (currentPollingNodeIndex < 0) {
        EV_ERROR << "[POLL_EXEC] Hub " << nodeId << ": Invalid polling node index: " << currentPollingNodeIndex << ". Resetting to 0." << endl;
        currentPollingNodeIndex = 0;
    }

    // 检查是否已轮询完所有节点
    if (currentPollingNodeIndex >= (int)polledNodeList.size()) {
      EV << "[POLL_EXEC] Hub " << nodeId << ": Polling round finished. All " << polledNodeList.size() << " nodes have been polled." << endl;
      
      // 检查是否仍在轮询阶段，如果是，则重新开始轮询
      if (currentPhase == TYPE1_POLLED || currentPhase == TYPE2_POLLED) {
          EV << "[POLL_EXEC] Hub " << nodeId << ": Still in polling phase. Restarting polling sequence from the beginning." << endl;
          currentPollingNodeIndex = 0; // 重置索引，重新开始轮询
          
          // 添加一个小延迟，避免立即重新轮询
          if (!pollTimer->isScheduled()) {
              simtime_t restartDelay = singlePollSlotDuration * 2; // 使用两倍的单个轮询时隙作为延迟
              scheduleAt(simTime() + restartDelay, pollTimer);
              EV << "[POLL_EXEC] Hub " << nodeId << ": Scheduled polling restart in " << restartDelay << " seconds." << endl;
          }
      } else {
          // 如果不在轮询阶段，取消已调度的pollTimer，防止重复轮询
          if (pollTimer->isScheduled()) {
              cancelEvent(pollTimer);
              EV << "[POLL_EXEC] Hub " << nodeId << ": Polling phase ended. Cancelled pollTimer." << endl;
          }
          // 重置轮询节点索引，准备下一轮
          currentPollingNodeIndex = 0;
      }
      return;
    }

    // 获取下一个要轮询的节点ID
    int targetNodeId = polledNodeList[currentPollingNodeIndex];
    
    // 获取目标节点的userPriority
    int targetNodePriority = 15; // 默认最高优先级
    cModule *targetNodeModule = getParentModule()->getParentModule()->getSubmodule("node", targetNodeId-1);
    if (targetNodeModule) {
        cModule *targetAppModule = targetNodeModule->getSubmodule("app");
        if (targetAppModule && targetAppModule->hasPar("userPriority")) {
            targetNodePriority = targetAppModule->par("userPriority");
        }
    }
    
    // 所有8-15优先级的节点都使用相同的轮询间隔，确保公平轮询
    simtime_t adjustedPollSlotDuration = singlePollSlotDuration;
    
     EV << "[POLL_EXEC] Hub " << nodeId << " is now polling Node " << targetNodeId << " (Index " << currentPollingNodeIndex 
        << ", Priority=" << targetNodePriority << ") with adjusted slot duration " << adjustedPollSlotDuration << "s." << endl;

    // 创建并发送一个管理帧作为轮询指令
    improvedwban::WBANManagementPacket *pollPkt = new improvedwban::WBANManagementPacket("WBAN-Poll");
    pollPkt->setSourceId(nodeId);
    pollPkt->setDestinationId(targetNodeId);
    pollPkt->setSequenceNumber(seqNum++);
    pollPkt->setCreationTime(simTime());
    
    // 设置新的Frame Control字段
    pollPkt->setFrameType(improvedwban::MANAGEMENT_FRAME);                                                    // 轮询帧是管理帧类型
    pollPkt->setFrameSubtype(improvedwban::MGMT_COMMAND);                                                    // 轮询帧使用命令子类型
    

    // 根据当前轮询阶段设置轮询类型和访问模式
    if (currentPhase == TYPE1_POLLED) {
        // Type-I轮询：基于时隙的即时分配
        pollPkt->setPollType(improvedwban::TYPE_I_ALLOCATION);
        pollPkt->setAccessMode(improvedwban::BEACON_MODE_WITH_SUPERFRAME);
        
        // 设置MAC头核心字段
        pollPkt->setMoreDataFlag(false);  // 0 = 授予即时分配
        pollPkt->setNextSuperframe(0);    // 0 = 当前超帧
        
        // 计算分配结束时隙号
        int currentSlot = 0;
        if (superframeStartTime >= 0 && allocationSlotLength > 0) {
            currentSlot = floor((simTime() - superframeStartTime) / allocationSlotLength);
        } else {
            EV_ERROR << "[POLL_EXEC] Hub " << nodeId << ": Invalid superframe start time or slot length. Using slot 0." << endl;
        }
        
        int allocationEndSlot = currentSlot + 5;  // 分配5个时隙
        pollPkt->setPollPostWindow(allocationEndSlot);
        pollPkt->setAllocationDurationSlots(5);  // 分配5个时隙
        pollPkt->setCurrentAllocationSlot(currentSlot);
        
        EV << "[POLL_EXEC] Hub " << nodeId << " sending TYPE-I POLL frame to Node " << targetNodeId 
           << ", allocation from slot " << currentSlot << " to " << allocationEndSlot << endl;
    } 
    else if (currentPhase == TYPE2_POLLED) {
        // Type-II轮询：基于帧数的即时分配
        pollPkt->setPollType(improvedwban::TYPE_II_ALLOCATION);
        pollPkt->setAccessMode(improvedwban::NON_BEACON_MODE_WITHOUT_SUPERFRAME);
        
        // 设置MAC头核心字段
        pollPkt->setMoreDataFlag(false);  // 0 = 授予即时分配
        pollPkt->setNextSuperframe(0);    // 0 = 当前超帧（保留字段）
        
        // 设置基于帧数的分配参数
        int maxFrames = 3;  // 允许发送最多3帧
        pollPkt->setMaxFramesToSend(maxFrames);
        pollPkt->setAllocationDurationFrames(3);  // 分配3帧
        
        EV << "[POLL_EXEC] Hub " << nodeId << " sending TYPE-II POLL frame to Node " << targetNodeId 
           << ", allocation of " << 3 << " frames (max: " << maxFrames << ")" << endl;
    }
    
    // 设置T-Poll帧的时间戳，用于节点同步
    // 添加边界检查，防止时间戳设置错误
    simtime_t currentTime = simTime();
    if (currentTime >= 0) {
        pollPkt->setAllocationTimestamp(currentTime);
        // 计算当前超帧号，确保不为负数
        if (beaconPeriodLength > 0) {
            int superframeNumber = floor(currentTime / beaconPeriodLength);
            if (superframeNumber >= 0) {
                pollPkt->setCurrentSuperframeNumber(superframeNumber);
            } else {
                EV_ERROR << "[POLL_EXEC] Hub " << nodeId << ": Invalid superframe number calculated: " << superframeNumber << ". Using 0 instead." << endl;
                pollPkt->setCurrentSuperframeNumber(0);
            }
        } else {
            EV_ERROR << "[POLL_EXEC] Hub " << nodeId << ": Invalid beacon period length: " << beaconPeriodLength << ". Using 0 for superframe number." << endl;
            pollPkt->setCurrentSuperframeNumber(0);
        }
    } else {
        EV_ERROR << "[POLL_EXEC] Hub " << nodeId << ": Invalid simulation time: " << currentTime << ". Not setting allocation timestamp." << endl;
    }
    
    // Hub端的集中式guard时间计算
    // 根据IEEE 802.15.6协议6.11.2节，集中式guard时间计算公式：
    // GTc = GT0 + SIN × (PH + PN)  (一个时隙由Hub控制、一个由节点控制)
    // 其中GT0 = pSIFS + pExtraIFS + mClockResolution
    simtime_t GT0 = pSIFS + pExtraIFS + mClockResolution;
    
    // 获取目标节点的时钟精度信息
    double targetNodePPM = nodeClockPPM;  // 默认值
    if (targetNodeModule) {
        cModule *targetMacModule = targetNodeModule->getSubmodule("mac");
        if (targetMacModule && targetMacModule->hasPar("nodeClockPPM")) {
            targetNodePPM = targetMacModule->par("nodeClockPPM");
        }
    }
    
    // 计算集中式guard时间
    // SIN = 节点最大同步间隔，使用maxSyncInterval
    // PH = Hub时钟精度，使用hubClockPPM
    // PN = 节点时钟精度，使用targetNodePPM
    simtime_t SIN = maxSyncInterval;
    double PH = hubClockPPM;
    double PN = targetNodePPM;
    
    // 计算集中式guard时间：GTc = GT0 + SIN × (PH + PN)
    simtime_t GTc = GT0 + SIN.dbl() * (PH + PN) * 1e-6;  // 转换ppm为实际时间
    
    // 更新集中式guard时间
    centralizedGuardTime = GTc;
    
    EV << "[GUARD_TIME] Hub " << nodeId << " calculated centralized guard time for Node " << targetNodeId << ": " << GTc << endl;
    EV << "[GUARD_TIME] GT0=" << GT0 << ", SIN=" << SIN << ", PH=" << PH << "ppm, PN=" << PN << "ppm" << endl;
    
    const int POLL_PACKET_BYTES = 8; // 假设一个很短的管理帧
    pollPkt->setByteLength(POLL_PACKET_BYTES);
    
    // 发送轮询帧
    sendDown(pollPkt);
    
    // 更新轮询统计
    pollSentCount++;
    emit(pollSentSignal, pollSentCount);
    
    // 增加轮询节点索引，准备下一次轮询
    currentPollingNodeIndex++;
    
    // 检查是否还有更多节点需要轮询
    if (currentPollingNodeIndex < (int)polledNodeList.size()) {
        // 调度下一个轮询事件
        if (!pollTimer->isScheduled()) {
            // 使用调整后的轮询时隙持续时间
            scheduleAt(simTime() + adjustedPollSlotDuration, pollTimer);
            EV << "[POLL_EXEC] Hub " << nodeId << ": Scheduled next poll for Node " << polledNodeList[currentPollingNodeIndex] 
               << " in " << adjustedPollSlotDuration << " seconds." << endl;
        }
    } else {
        EV << "[POLL_EXEC] Hub " << nodeId << ": All nodes have been polled. Polling sequence complete." << endl;
    }
}

// 6.6. 网络时间对其函数
simtime_t IEEE802_15_6Mac::getNextSlotBoundary() {
    // 添加边界检查，确保所有参数有效
    if (superframeStartTime < 0) {
        // 如果还没有收到过信标，无法计算，返回一个遥远的未来时间以防止错误调度
        EV_ERROR << "[SLOT_BOUNDARY] Node " << nodeId << ": Cannot get next slot boundary, not synchronized yet!";
        return simTime() + 100.0; // 或者其他错误处理
    }

    // 检查allocationSlotLength是否有效
    if (allocationSlotLength <= 0) {
        EV_ERROR << "[SLOT_BOUNDARY] Node " << nodeId << ": Invalid allocation slot length: " << allocationSlotLength;
        return simTime() + 100.0; // 返回一个遥远的未来时间以防止错误调度
    }

    // 1. 计算从超帧开始到现在经过了多长时间
    simtime_t elapsedTime = simTime() - superframeStartTime;

    // 2. 计算已经完整过去了多少个时隙
    //    必须使用浮点数除法来保证精度
    int passedSlots = floor(elapsedTime.dbl() / allocationSlotLength.dbl());

    // 3. 计算下一个时隙边界的绝对时间
    simtime_t nextSlotTime = superframeStartTime + (passedSlots + 1) * allocationSlotLength;
    
    // 4. [鲁棒性检查] 确保我们调度的时间在未来
    //    这可以处理因计算延迟导致 nextSlotTime <= simTime() 的边缘情况
    if (nextSlotTime <= simTime()) {
         nextSlotTime += allocationSlotLength;
    }

    return nextSlotTime;
}

// 计算吞吐量函数
void IEEE802_15_6Mac::updateThroughput() {
    simtime_t currentTime = simTime();
    simtime_t elapsed = currentTime - lastThroughputUpdateTime;
    
    // 每秒更新一次吞吐量
    if (elapsed >= 1.0) {
        // 计算吞吐量 (bps)
        currentThroughput = (totalReceivedBytes * 8.0) / elapsed.dbl();
        
        // 发射吞吐量信号
        emit(throughputSignal, currentThroughput);
        
        // 重置计数器和更新时间
        totalReceivedBytes = 0;
        lastThroughputUpdateTime = currentTime;
        
        EV << "[THROUGHPUT] Node " << nodeId << ": Current throughput = " << currentThroughput << " bps" << endl;
    }
}
}
