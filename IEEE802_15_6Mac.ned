package improvedwban.mac.ieee802_15_6;

import improvedwban.mac.IMac;

simple IEEE802_15_6Mac like IMac
{
    parameters:
        // ================ 基本参数 ================
        int nodeId;                                         // 节点ID，用于标识网络中的唯一节点
        bool isHub = default(false);                        // 是否为中心节点(Hub)，true表示Hub，false表示普通节点
        int queueSize;                                      // 队列大小，决定MAC层可以缓存的数据包数量
        
        // ================ 超帧参数 ================
        double beaconPeriodLength @unit(ms);                // 信标周期长度
        double allocationSlotLength @unit(ms);              // 单位时隙长度
        
        // 超帧阶段百分比参数 (总和应为1.0)
        double eap1Percentage = default(0.05);              // EAP1阶段百分比，优先访问阶段1 (5%)
        double rap1Percentage = default(0.20);              // RAP1阶段百分比，随机访问阶段1 (20%)
        double type1PolledPercentage = default(0.15);       // Type1轮询阶段百分比 (15%)
        double eap2Percentage = default(0.05);              // EAP2阶段百分比，优先访问阶段2 (5%)
        double rap2Percentage = default(0.20);              // RAP2阶段百分比，随机访问阶段2 (20%)
        double type2PolledPercentage = default(0.15);       // Type2轮询阶段百分比 (15%)
        double capPercentage = default(0.20);               // CAP阶段百分比，争用访问阶段 (20%)
        
        double singlePollSlotDuration @unit(s) = default(0.005s); // 单个轮询时隙长度
        
        // ================ 优先级参数 ================
        int CWminPriority0 = default(16);                    // 优先级0节点的CWmin
        int CWmaxPriority0 = default(64);                    // 优先级0节点的CWmax
        int CWminPriority1 = default(16);                    // 优先级1节点的CWmin
        int CWmaxPriority1 = default(32);                    // 优先级1节点的CWmax
        int CWminPriority2 = default(8);                     // 优先级2节点的CWmin
        int CWmaxPriority2 = default(32);                    // 优先级2节点的CWmax
        int CWminPriority3 = default(8);                     // 优先级3节点的CWmin
        int CWmaxPriority3 = default(16);                    // 优先级3节点的CWmax
        int CWminPriority4 = default(4);                     // 优先级4节点的CWmin
        int CWmaxPriority4 = default(16);                    // 优先级4节点的CWmax
        int CWminPriority5 = default(4);                     // 优先级5节点的CWmin
        int CWmaxPriority5 = default(8);                     // 优先级5节点的CWmax
        int CWminPriority6 = default(2);                     // 优先级6节点的CWmin
        int CWmaxPriority6 = default(8);                     // 优先级6节点的CWmax
        int CWminPriority7 = default(1);                     // 优先级7节点的CWmin
        int CWmaxPriority7 = default(4);                     // 优先级7节点的CWmax
        
        // ================ 载波监听参数 ================
        double ccaThreshold @unit(dBm) = default(-75dBm);    // CCA阈值，用于判断信道是否忙碌
        double ccaDuration @unit(us) = default(128us);       // CCA持续时间，执行一次CCA所需的时间
            
        // ================ 重传机制参数 ================
        int maxRetransmissions = default(3);                // 最大重传次数，数据包传输失败后的最大重传次数
        double baseRetransmissionTimeout @unit(ms) = default(10ms); // 基础重传超时时间，重传计时器的基础时间
        
        // ================ 轮询参数 ================
        bool enablePolling = default(true);                  // 是否启用轮询机制，Hub用于调度节点传输
        bool enableAdaptivePolling = default(true);          // 是否启用自适应轮询，根据网络状况动态调整轮询策略
        bool enablePriorityPolling = default(true);          // 是否启用优先级轮询，根据节点优先级进行轮询
        
        // ================ 物理层参数 ================
        double rssiThreshold @unit(dBm) = default(-85dBm);   // RSSI阈值，用于判断信道是否忙碌
        
        // ================ 分布式保护时间参数 ================
        bool enableDistributedGuardTime = default(true);     // 是否启用分布式保护时间，减少节点间干扰
        double guardTimeAdjustmentFactor = default(1.0);     // 保护时间调整因子，用于调整保护时间长度
        
        // ================ 时钟同步参数 ================
        bool enableClockSync = default(true);                // 是否启用时钟同步，确保网络中节点时间同步
        double clockSyncAccuracy @unit(s) = default(0.0001s); // 时钟同步精度，同步的时间精度
        double hubClockPPM = default(20.0);                 // Hub时钟精度 (ppm)，表示每百万秒的误差
        double nodeClockPPM = default(40.0);                // 节点时钟精度 (ppm)，表示每百万秒的误差
        double maxSyncInterval @unit(s) = default(10s);     // 最大同步间隔 (SIN)，超过此时间需要重新同步
        int maxSyncFailures = default(3);                   // 最大允许同步失败次数，超过此次数将重置同步状态
     
        // ================ 统计信号 ================
        @display("i=block/routing");
        
        // 数据包统计信号
        @signal[packetReceived](type=long);                 // MAC接收数据包统计信号
        @signal[packetSent](type=long);                     // MAC发送数据包统计信号
        @signal[packetDropped](type=long);                  // MAC丢弃数据包统计信号
        @signal[endToEndDelay](type=simtime_t);             // 端到端延迟统计信号
        @signal[queueLength](type=long);                    // 队列长度统计信号
        @signal[channelState](type=long);                   // CCA检测信道繁忙总数
        @signal[throughput](type=double);                   // 吞吐量统计信号
        
        
        // 轮询统计信号
        @signal[pollReceived](type=long);                   // 接收轮询统计信号
        @signal[pollSent](type=long);                       // 发送轮询统计信号
        
        
        // 其他统计信号
        @signal[distributedGuardTime](type=long);           // 分布式保护时间统计信号

        // ================ 统计量 ================
        @statistic[packetReceived](title="Received packets"; source=packetReceived; record=count,sum,vector; interpolationmode=none);
        @statistic[packetSent](title="Sent packets"; source=packetSent; record=count,sum,vector; interpolationmode=none);
        @statistic[packetDropped](title="Dropped packets"; source=packetDropped; record=count,sum,vector; interpolationmode=none);
        @statistic[endToEndDelay](title="End-to-end delay"; source=endToEndDelay; record=mean,min,max,vector; interpolationmode=none);
        @statistic[queueLength](title="Queue length"; source=queueLength; record=mean,min,max,vector; interpolationmode=none);
        @statistic[throughput](title="Throughput"; source=throughput; record=mean,min,max,vector; unit=bps; interpolationmode=none);
        @statistic[pollReceived](title="Polls received"; source=pollReceived; record=count,sum,vector; interpolationmode=none);
        @statistic[pollSent](title="Polls sent"; source=pollSent; record=count,sum,vector; interpolationmode=none);
        @statistic[distributedGuardTime](title="Distributed guard time events"; source=distributedGuardTime; record=count,sum,vector; interpolationmode=none);
        
    gates:
        // ================ 通信门 ================
        input upperLayerIn @labels(IPacket);                 // 上层输入门，接收来自上层的数据包
        output upperLayerOut @labels(IPacket);               // 上层输出门，向上层发送数据包
        input lowerLayerIn @labels(IPacket);                 // 下层输入门，接收来自下层的数据包
        output lowerLayerOut @labels(IPacket);               // 下层输出门，向下层发送数据包
}