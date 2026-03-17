/** Network topology
 *
 *    100Mb/s, 1ms                            100Mb/s, 1ms
 * n0----L0--------|                    |------L9-------n10
 *                 |   50Mbps/s, 18ms   |
 *                 n8--------L8--------n9
 *    100Mb/s, 1ms |                    |    100Mb/s, 1ms
 * n1----L1--------|                    |------L10------n11
 *                 |                    |
 *                 |                    |
 * n2----L2--------|                    |------L11------n12
 *                 |                    |
 *                 |                    |
 * n3----L3--------|                    |------L12------n13
 *                 |                    |
 *                 |                    |
 * n4----L4--------|                    |------L13------n14
 *                 |                    |
 *                 |                    |
 * n5----L5--------|                    |------L14------n15
 *                 |                    |
 *                 |                    |
 * n6----L6--------|                    |------L15------n16
 *                 |                    |
 *                 |                    |
 * n7----L7--------|                    |------L16------n17
 *
 * 8 BBRv2 flows, 50M, 40ms RTT, buffer = 2xBDP
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/traffic-control-module.h"
#include "ns3/dqc-module.h"
#include "ns3/log.h"
#include<stdio.h>
#include<iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>
#include <memory>
#include <chrono>
#include "queue_occupancy_trace_helper.h"
using namespace ns3;
using namespace dqc;
using namespace std;
NS_LOG_COMPONENT_DEFINE ("8-bbrv2");

const int NUM_FLOWS = 8;

uint32_t checkTimes;
double avgQueueSize;

// The times
double global_start_time;
double global_stop_time;
double sink_start_time;
double sink_stop_time;
double client_start_time;
double client_stop_time;

// Node containers for sender side (n0-n7 to n8)
NodeContainer n0n8;
NodeContainer n1n8;
NodeContainer n2n8;
NodeContainer n3n8;
NodeContainer n4n8;
NodeContainer n5n8;
NodeContainer n6n8;
NodeContainer n7n8;

// Bottleneck link
NodeContainer n8n9;

// Node containers for receiver side (n9 to n10-n17)
NodeContainer n9n10;
NodeContainer n9n11;
NodeContainer n9n12;
NodeContainer n9n13;
NodeContainer n9n14;
NodeContainer n9n15;
NodeContainer n9n16;
NodeContainer n9n17;

// IP interface containers
Ipv4InterfaceContainer i0i8;
Ipv4InterfaceContainer i1i8;
Ipv4InterfaceContainer i2i8;
Ipv4InterfaceContainer i3i8;
Ipv4InterfaceContainer i4i8;
Ipv4InterfaceContainer i5i8;
Ipv4InterfaceContainer i6i8;
Ipv4InterfaceContainer i7i8;
Ipv4InterfaceContainer i8i9;
Ipv4InterfaceContainer i9i10;
Ipv4InterfaceContainer i9i11;
Ipv4InterfaceContainer i9i12;
Ipv4InterfaceContainer i9i13;
Ipv4InterfaceContainer i9i14;
Ipv4InterfaceContainer i9i15;
Ipv4InterfaceContainer i9i16;
Ipv4InterfaceContainer i9i17;

typedef struct
{
uint64_t bps;
uint32_t msDelay;
uint32_t msQdelay;
}link_config_t;

// Link configurations: L0-L7 (sender side), L8 (bottleneck), L9-L16 (receiver side)

const uint64_t TOPO_SENDER_BW       =   2000000;    // in bps
const uint64_t TOPO_SENDER_PDELAY   =   1;    // in ms
const uint64_t TOPO_BOTTLE_BW       =   20000000;    // in bps
const uint64_t TOPO_BOTTLE_PDELAY   =   18;    // in ms
const uint64_t TOPO_DEFAULT_QDELAY  =   400;    // in ms

link_config_t p4p[]={
[0]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L0: n0-n8
[1]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L1: n1-n8
[2]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L2: n2-n8
[3]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L3: n3-n8
[4]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L4: n4-n8
[5]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L5: n5-n8
[6]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L6: n6-n8
[7]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L7: n7-n8
[8]={TOPO_BOTTLE_BW,TOPO_BOTTLE_PDELAY,TOPO_DEFAULT_QDELAY},   // L8: n8-n9 (bottleneck)
[9]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L9: n9-n10
[10]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L10: n9-n11
[11]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L11: n9-n12
[12]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L12: n9-n13
[13]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L13: n9-n14
[14]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L14: n9-n15
[15]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L15: n9-n16
[16]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L16: n9-n17
};


static Ptr<DqcSender> InstallDqc( dqc::CongestionControlType cc_type,
                        Ptr<Node> sender,Ptr<Node> receiver,
                        uint16_t send_port,uint16_t recv_port,
                        float startTime,float stopTime,
                        DqcTrace *trace, DqcTraceState *stat,
                        uint32_t max_bps=0,uint32_t cid=0,bool ecn=false,uint32_t emucons=1)
{
    Ptr<DqcSender> sendApp = CreateObject<DqcSender> (cc_type,ecn);
    Ptr<DqcReceiver> recvApp = CreateObject<DqcReceiver>(100);  // 100ms goodput统计间隔，更实时
    sender->AddApplication (sendApp);
    receiver->AddApplication (recvApp);
    sendApp->SetNumEmulatedConnections(emucons);
    Ptr<Ipv4> ipv4 = receiver->GetObject<Ipv4> ();
    Ipv4Address receiverIp = ipv4->GetAddress (1, 0).GetLocal ();
    recvApp->Bind(recv_port);
    sendApp->Bind(send_port);
    sendApp->ConfigurePeer(receiverIp,recv_port);
    sendApp->SetStartTime (Seconds (startTime));
    sendApp->SetStopTime (Seconds (stopTime));
    recvApp->SetStartTime (Seconds (startTime));
    recvApp->SetStopTime (Seconds (stopTime));
    if(max_bps>0){
        sendApp->SetMaxBandwidth(max_bps);
    }
    if(cid){
       sendApp->SetSenderId(cid);
        sendApp->SetCongestionId(cid);
    }
    if(trace){
        sendApp->SetBwTraceFuc(MakeCallback(&DqcTrace::OnBw,trace));
        sendApp->SetRttTraceFuc(MakeCallback(&DqcTrace::OnRtt,trace));
        sendApp->SetQueueDelayTraceFuc(MakeCallback(&DqcTrace::OnQueueDelay,trace));
        sendApp->SetSendRateTraceFuc(MakeCallback(&DqcTrace::OnSendRate,trace));
        sendApp->SetRecvRateTraceFuc(MakeCallback(&DqcTrace::OnRecvRate,trace));
        sendApp->SetInflightTraceFuc(MakeCallback(&DqcTrace::OnInflight,trace));
        sendApp->SetBbrModeTraceFuc(MakeCallback(&DqcTrace::OnBbrMode,trace));
        sendApp->SetLossRateTraceFuc(MakeCallback(&DqcTrace::OnLossRate,trace));
        sendApp->SetAckEventTraceFuc(MakeCallback(&DqcTrace::OnAckEvent,trace));
        sendApp->SetAckEpisodeTraceFuc(MakeCallback(&DqcTrace::OnAckEpisode,trace));
        // recvApp->SetOwdTraceFuc(MakeCallback(&DqcTrace::OnOwd,trace));
        // recvApp->SetGoodputTraceFuc(MakeCallback(&DqcTrace::OnGoodput,trace));
        // recvApp->SetStatsTraceFuc(MakeCallback(&DqcTrace::OnStats,trace));
        // trace->SetStatsTraceFuc(MakeCallback(&DqcTraceState::OnStats,stat));
    }
    return sendApp;
}

void ns3_bbrv2(int ins, std::string algo, DqcTraceState *stat, int sim_time=60, int loss_integer=0){
    std::string instance="8_bbrv2";  // Use script filename instead of instance number
    uint64_t linkBw   = p4p[8].bps;
    uint16_t sendPort=1000;
    uint16_t recvPort=5000;

    double sim_dur=sim_time;
    int end_time=sim_time;
    float appStop=end_time;

    NodeContainer c;
    c.Create (18);  // 18 nodes: n0-n7 (senders), n8-n9 (routers), n10-n17 (receivers)

    // Sender side connections
    n0n8 = NodeContainer (c.Get (0), c.Get (8));
    n1n8 = NodeContainer (c.Get (1), c.Get (8));
    n2n8 = NodeContainer (c.Get (2), c.Get (8));
    n3n8 = NodeContainer (c.Get (3), c.Get (8));
    n4n8 = NodeContainer (c.Get (4), c.Get (8));
    n5n8 = NodeContainer (c.Get (5), c.Get (8));
    n6n8 = NodeContainer (c.Get (6), c.Get (8));
    n7n8 = NodeContainer (c.Get (7), c.Get (8));

    // Bottleneck link
    n8n9 = NodeContainer (c.Get (8), c.Get (9));

    // Receiver side connections
    n9n10 = NodeContainer (c.Get (9), c.Get (10));
    n9n11 = NodeContainer (c.Get (9), c.Get (11));
    n9n12 = NodeContainer (c.Get (9), c.Get (12));
    n9n13 = NodeContainer (c.Get (9), c.Get (13));
    n9n14 = NodeContainer (c.Get (9), c.Get (14));
    n9n15 = NodeContainer (c.Get (9), c.Get (15));
    n9n16 = NodeContainer (c.Get (9), c.Get (16));
    n9n17 = NodeContainer (c.Get (9), c.Get (17));

    link_config_t *config=p4p;
    uint32_t bufSize=0;

    InternetStackHelper internet;
    internet.Install (c);

    NS_LOG_INFO ("Create channels");
    PointToPointHelper p2p;
    TrafficControlHelper tch;
    std::vector<std::shared_ptr<QueueOccupancyTracer>> queue_tracers;

    //L0-L7 and L9-L16: Edge links (100Mbps)
    bufSize =TOPO_SENDER_BW * TOPO_DEFAULT_QDELAY / 8000;
    p2p.SetQueue ("ns3::DropTailQueue",
                "Mode", StringValue ("QUEUE_MODE_BYTES"),
                "MaxBytes", UintegerValue (bufSize));
    p2p.SetDeviceAttribute ("DataRate", DataRateValue(DataRate (config[0].bps)));
    p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (config[0].msDelay)));

    // Sender side edge links
    NetDeviceContainer devn0n8 = p2p.Install (n0n8);
    NetDeviceContainer devn1n8 = p2p.Install (n1n8);
    NetDeviceContainer devn2n8 = p2p.Install (n2n8);
    NetDeviceContainer devn3n8 = p2p.Install (n3n8);
    NetDeviceContainer devn4n8 = p2p.Install (n4n8);
    NetDeviceContainer devn5n8 = p2p.Install (n5n8);
    NetDeviceContainer devn6n8 = p2p.Install (n6n8);
    NetDeviceContainer devn7n8 = p2p.Install (n7n8);

    // Receiver side edge links
    NetDeviceContainer devn9n10 = p2p.Install (n9n10);
    NetDeviceContainer devn9n11 = p2p.Install (n9n11);
    NetDeviceContainer devn9n12 = p2p.Install (n9n12);
    NetDeviceContainer devn9n13 = p2p.Install (n9n13);
    NetDeviceContainer devn9n14 = p2p.Install (n9n14);
    NetDeviceContainer devn9n15 = p2p.Install (n9n15);
    NetDeviceContainer devn9n16 = p2p.Install (n9n16);
    NetDeviceContainer devn9n17 = p2p.Install (n9n17);

    //L8: Bottleneck link (50Mbps)
    bufSize = TOPO_BOTTLE_BW * TOPO_DEFAULT_QDELAY / 8000;//与msQdelay相关，这里代表1个BDP
    p2p.SetQueue ("ns3::DropTailQueue",
                "Mode", StringValue ("QUEUE_MODE_BYTES"),
                "MaxBytes", UintegerValue (bufSize));
    p2p.SetDeviceAttribute ("DataRate", DataRateValue(DataRate (config[8].bps)));
    p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (config[8].msDelay)));
    NetDeviceContainer devn8n9 = p2p.Install (n8n9);
    queue_tracers.push_back(InstallBottleneckQueueOccupancyTrace(devn8n9.Get(0), instance, NUM_FLOWS));

    Ipv4AddressHelper ipv4;

    // Sender side IP addresses
    ipv4.SetBase ("10.1.1.0", "255.255.255.0");
    i0i8 = ipv4.Assign (devn0n8);
    tch.Uninstall (devn0n8);
    ipv4.SetBase ("10.1.2.0", "255.255.255.0");
    i1i8 = ipv4.Assign (devn1n8);
    tch.Uninstall (devn1n8);
    ipv4.SetBase ("10.1.3.0", "255.255.255.0");
    i2i8 = ipv4.Assign (devn2n8);
    tch.Uninstall (devn2n8);
    ipv4.SetBase ("10.1.4.0", "255.255.255.0");
    i3i8 = ipv4.Assign (devn3n8);
    tch.Uninstall (devn3n8);
    ipv4.SetBase ("10.1.5.0", "255.255.255.0");
    i4i8 = ipv4.Assign (devn4n8);
    tch.Uninstall (devn4n8);
    ipv4.SetBase ("10.1.6.0", "255.255.255.0");
    i5i8 = ipv4.Assign (devn5n8);
    tch.Uninstall (devn5n8);
    ipv4.SetBase ("10.1.7.0", "255.255.255.0");
    i6i8 = ipv4.Assign (devn6n8);
    tch.Uninstall (devn6n8);
    ipv4.SetBase ("10.1.8.0", "255.255.255.0");
    i7i8 = ipv4.Assign (devn7n8);
    tch.Uninstall (devn7n8);

    // Bottleneck link IP address
    ipv4.SetBase ("10.1.9.0", "255.255.255.0");
    i8i9 = ipv4.Assign (devn8n9);
    tch.Uninstall (devn8n9);

    // Receiver side IP addresses
    ipv4.SetBase ("10.1.10.0", "255.255.255.0");
    i9i10 = ipv4.Assign (devn9n10);
    tch.Uninstall (devn9n10);
    ipv4.SetBase ("10.1.11.0", "255.255.255.0");
    i9i11 = ipv4.Assign (devn9n11);
    tch.Uninstall (devn9n11);
    ipv4.SetBase ("10.1.12.0", "255.255.255.0");
    i9i12 = ipv4.Assign (devn9n12);
    tch.Uninstall (devn9n12);
    ipv4.SetBase ("10.1.13.0", "255.255.255.0");
    i9i13 = ipv4.Assign (devn9n13);
    tch.Uninstall (devn9n13);
    ipv4.SetBase ("10.1.14.0", "255.255.255.0");
    i9i14 = ipv4.Assign (devn9n14);
    tch.Uninstall (devn9n14);
    ipv4.SetBase ("10.1.15.0", "255.255.255.0");
    i9i15 = ipv4.Assign (devn9n15);
    tch.Uninstall (devn9n15);
    ipv4.SetBase ("10.1.16.0", "255.255.255.0");
    i9i16 = ipv4.Assign (devn9n16);
    tch.Uninstall (devn9n16);
    ipv4.SetBase ("10.1.17.0", "255.255.255.0");
    i9i17 = ipv4.Assign (devn9n17);
    tch.Uninstall (devn9n17);

    // Set up the routing
    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

    // Use original BBRv2 for all 8 flows
    dqc::CongestionControlType cc = kBBRv2;

    uint32_t max_bps=0;
    int test_pair=1;
    uint32_t sender_id=1;

    std::vector<std::unique_ptr<DqcTrace>> traces;
    std::vector<Ptr<DqcSender>> senders;
    std::string log;
    std::string delimiter="_";
    std::string prefix=instance+delimiter;  // instance already includes script name and algorithm
    log=prefix+std::to_string(test_pair);
    std::unique_ptr<DqcTrace> trace;

    float flow_start_times[NUM_FLOWS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Flow 1: n0 -> n10
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    stat->RegisterCongestionType(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE
|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE|DqcTraceEnable::E_DQC_QUEUE_DELAY);
    Ptr<DqcSender> sender1 = InstallDqc(cc,c.Get(0),c.Get(10),sendPort,recvPort,flow_start_times[0]+0.001,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender1);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 2: n1 -> n11
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE
|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE|DqcTraceEnable::E_DQC_QUEUE_DELAY);
    Ptr<DqcSender> sender2 = InstallDqc(cc,c.Get(1),c.Get(11),sendPort,recvPort,flow_start_times[1]+0.002,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender2);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 3: n2 -> n12
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE
|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE|DqcTraceEnable::E_DQC_QUEUE_DELAY);
    Ptr<DqcSender> sender3 = InstallDqc(cc,c.Get(2),c.Get(12),sendPort,recvPort,flow_start_times[2]+0.003,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender3);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 4: n3 -> n13
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE
|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE|DqcTraceEnable::E_DQC_QUEUE_DELAY);
    Ptr<DqcSender> sender4 = InstallDqc(cc,c.Get(3),c.Get(13),sendPort,recvPort,flow_start_times[3]+0.004,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender4);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 5: n4 -> n14
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE
|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE|DqcTraceEnable::E_DQC_QUEUE_DELAY);
    Ptr<DqcSender> sender5 = InstallDqc(cc,c.Get(4),c.Get(14),sendPort,recvPort,flow_start_times[4]+0.005,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender5);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 6: n5 -> n15
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE
|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE|DqcTraceEnable::E_DQC_QUEUE_DELAY);
    Ptr<DqcSender> sender6 = InstallDqc(cc,c.Get(5),c.Get(15),sendPort,recvPort,flow_start_times[5]+0.006,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender6);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 7: n6 -> n16
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE
|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE|DqcTraceEnable::E_DQC_QUEUE_DELAY);
    Ptr<DqcSender> sender7 = InstallDqc(cc,c.Get(6),c.Get(16),sendPort,recvPort,flow_start_times[6]+0.007,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender7);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 8: n7 -> n17
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE
|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE|DqcTraceEnable::E_DQC_QUEUE_DELAY);
    Ptr<DqcSender> sender8 = InstallDqc(cc,c.Get(7),c.Get(17),sendPort,recvPort,flow_start_times[7]+0.008,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender8);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    Simulator::Stop (Seconds(sim_dur));
    Simulator::Run ();
    Simulator::Destroy();
    stat->Flush(linkBw,sim_dur);
}

int main (int argc, char *argv[]){
    int sim_time=30;
    int ins[]={1};
    std::string trace_path="";

    // Command line arguments
    CommandLine cmd;
    cmd.AddValue("sim_time", "Simulation time in seconds", sim_time);
    cmd.AddValue("trace_path", "Output trace directory path", trace_path);
    cmd.Parse(argc, argv);
    if(!trace_path.empty()){
        if(trace_path.back() != '/'){
            trace_path.push_back('/');
        }
        set_dqc_trace_folder(trace_path);
    }
    SetQueueOccupancyTraceFolder(trace_path);

    // Print configuration
    std::cout << "=== 8 BBRv2 Flows Configuration ===" << std::endl;
    std::cout << "Number of flows: " << NUM_FLOWS << std::endl;
    std::cout << "Congestion control: BBRv2" << std::endl;
    std::cout << "Bottleneck bandwidth: "<<TOPO_BOTTLE_BW/1000000<<" Mbps" << std::endl;
    std::cout << "Bottleneck delay: "<<TOPO_BOTTLE_PDELAY<<" ms" << std::endl;
    std::cout << "Simulation time: " << sim_time << " seconds" << std::endl;
    std::cout << "====================================" << std::endl;

    const char *algos[]={"bbrv2"};
    for (size_t c = 0; c < sizeof(algos) / sizeof(algos[0]); ++c){
        std::string cong=std::string(algos[c]);
        std::string name=cong;
        std::unique_ptr<DqcTraceState> stat;
        stat.reset(new DqcTraceState(name));
        auto inner_start = std::chrono::high_resolution_clock::now();
        for (size_t i = 0; i < sizeof(ins) / sizeof(ins[0]); ++i){
            ns3_bbrv2(ins[i],cong,stat.get(),sim_time);
        }
        auto inner_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> tm = inner_end - inner_start;
        std::chrono::duration<double, std::ratio<60>> minutes =inner_end- inner_start;
        stat->RecordRuningTime(tm.count(),minutes.count());
    }
    return 0;
}
