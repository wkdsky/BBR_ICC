/** Network topology
 *
 *    10Mb/s, 1ms                            10Mb/s, 1ms
 * n0----L0--------|                    |------L5-------n5
 *                 |   8Mbps/s, 28ms    |
 *                 n4--------L4--------n5
 *    10Mb/s, 1ms  |                    |    10Mb/s, 1ms
 * n1----L1--------|                    |------L6-------n6
 *                 |                    |
 *                 |                    |
 * n2----L2--------|                    |------L7-------n7
 *                 |                    |
 *                 |                    |
 * n3----L3--------|                    |------L8-------n8
 *
 * 4 BBRv2 flows, 8Mbps bottleneck, 60ms RTT, buffer = 1xBDP
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
#ifndef DQC_SCENARIO_LOG_COMPONENT
#define DQC_SCENARIO_LOG_COMPONENT "4-bbrv2"
#endif
#ifndef DQC_SCENARIO_INSTANCE
#define DQC_SCENARIO_INSTANCE "4_bbrv2"
#endif
#ifndef DQC_DEFAULT_CC
#define DQC_DEFAULT_CC "bbrv2"
#endif
#ifndef DQC_SCENARIO_TITLE
#define DQC_SCENARIO_TITLE "4 BBRv2-Style Flows"
#endif
NS_LOG_COMPONENT_DEFINE (DQC_SCENARIO_LOG_COMPONENT);

const int NUM_FLOWS = 4;

uint32_t checkTimes;
double avgQueueSize;

// The times
double global_start_time;
double global_stop_time;
double sink_start_time;
double sink_stop_time;
double client_start_time;
double client_stop_time;

// Node containers for sender side (n0-n3 to n4)
NodeContainer n0n4;
NodeContainer n1n4;
NodeContainer n2n4;
NodeContainer n3n4;

// Bottleneck link
NodeContainer n4n5;

// Node containers for receiver side (n5 to n6-n9)
NodeContainer n5n6;
NodeContainer n5n7;
NodeContainer n5n8;
NodeContainer n5n9;

// IP interface containers
Ipv4InterfaceContainer i0i4;
Ipv4InterfaceContainer i1i4;
Ipv4InterfaceContainer i2i4;
Ipv4InterfaceContainer i3i4;
Ipv4InterfaceContainer i4i5;
Ipv4InterfaceContainer i5i6;
Ipv4InterfaceContainer i5i7;
Ipv4InterfaceContainer i5i8;
Ipv4InterfaceContainer i5i9;

typedef struct
{
uint64_t bps;
uint32_t msDelay;
uint32_t msQdelay;
}link_config_t;

// Link configurations: L0-L3 (sender side), L4 (bottleneck), L5-L8 (receiver side)

const uint64_t TOPO_SENDER_BW       =   50000000;    // in bps
const uint64_t TOPO_SENDER_PDELAY   =   1;    // in ms
const uint64_t TOPO_BOTTLE_BW       =   20000000;    // in bps
const uint64_t TOPO_BOTTLE_PDELAY   =   18;    // in ms
const uint64_t TOPO_DEFAULT_QDELAY  =   160;    // in ms

link_config_t p4p[]={
[0]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L0: n0-n4
[1]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L1: n1-n4
[2]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L2: n2-n4
[3]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L3: n3-n4
[4]={TOPO_BOTTLE_BW,TOPO_BOTTLE_PDELAY,TOPO_DEFAULT_QDELAY},   // L4: n4-n5 (bottleneck)
[5]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L5: n5-n6
[6]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L6: n5-n7
[7]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L7: n5-n8
[8]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L8: n5-n9
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
        // recvApp->SetOwdTraceFuc(MakeCallback(&DqcTrace::OnOwd,trace));
        // recvApp->SetGoodputTraceFuc(MakeCallback(&DqcTrace::OnGoodput,trace));
        // recvApp->SetStatsTraceFuc(MakeCallback(&DqcTrace::OnStats,trace));
        // trace->SetStatsTraceFuc(MakeCallback(&DqcTraceState::OnStats,stat));
    }
    return sendApp;
}

void ns3_bbrv2(int ins, std::string algo, DqcTraceState *stat, int sim_time=60, int loss_integer=0){
    std::string instance=DQC_SCENARIO_INSTANCE;  // Use script filename instead of instance number
    uint64_t linkBw   = TOPO_BOTTLE_BW;
    uint16_t sendPort=1000;
    uint16_t recvPort=5000;

    double sim_dur=sim_time;
    int end_time=sim_time;
    float appStop=end_time;

    NodeContainer c;
    c.Create (10);  // 10 nodes: n0-n3 (senders), n4-n5 (routers), n6-n9 (receivers)

    // Sender side connections
    n0n4 = NodeContainer (c.Get (0), c.Get (4));
    n1n4 = NodeContainer (c.Get (1), c.Get (4));
    n2n4 = NodeContainer (c.Get (2), c.Get (4));
    n3n4 = NodeContainer (c.Get (3), c.Get (4));

    // Bottleneck link
    n4n5 = NodeContainer (c.Get (4), c.Get (5));

    // Receiver side connections
    n5n6 = NodeContainer (c.Get (5), c.Get (6));
    n5n7 = NodeContainer (c.Get (5), c.Get (7));
    n5n8 = NodeContainer (c.Get (5), c.Get (8));
    n5n9 = NodeContainer (c.Get (5), c.Get (9));

    uint32_t bufSize=0;

    InternetStackHelper internet;
    internet.Install (c);

    NS_LOG_INFO ("Create channels");
    PointToPointHelper p2p;
    TrafficControlHelper tch;
    std::vector<std::shared_ptr<QueueOccupancyTracer>> queue_tracers;

    //L0-L3 and L5-L8: Edge links (10Mbps)
    bufSize =TOPO_SENDER_BW * TOPO_DEFAULT_QDELAY / 8000;
    p2p.SetQueue ("ns3::DropTailQueue",
                "Mode", StringValue ("QUEUE_MODE_BYTES"),
                "MaxBytes", UintegerValue (bufSize));
    p2p.SetDeviceAttribute ("DataRate", DataRateValue(DataRate (TOPO_SENDER_BW)));
    p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (TOPO_SENDER_PDELAY)));

    // Sender side edge links
    NetDeviceContainer devn0n4 = p2p.Install (n0n4);
    NetDeviceContainer devn1n4 = p2p.Install (n1n4);
    NetDeviceContainer devn2n4 = p2p.Install (n2n4);
    NetDeviceContainer devn3n4 = p2p.Install (n3n4);

    // Receiver side edge links
    NetDeviceContainer devn5n6 = p2p.Install (n5n6);
    NetDeviceContainer devn5n7 = p2p.Install (n5n7);
    NetDeviceContainer devn5n8 = p2p.Install (n5n8);
    NetDeviceContainer devn5n9 = p2p.Install (n5n9);

    //L4: Bottleneck link (8Mbps)
    bufSize =TOPO_BOTTLE_BW * TOPO_DEFAULT_QDELAY / 8000;//与msQdelay相关，这里代表1个BDP
    p2p.SetQueue ("ns3::DropTailQueue",
                "Mode", StringValue ("QUEUE_MODE_BYTES"),
                "MaxBytes", UintegerValue (bufSize));
    p2p.SetDeviceAttribute ("DataRate", DataRateValue(DataRate (TOPO_BOTTLE_BW)));
    p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (TOPO_BOTTLE_PDELAY)));
    NetDeviceContainer devn4n5 = p2p.Install (n4n5);
    queue_tracers.push_back(InstallBottleneckQueueOccupancyTrace(devn4n5.Get(0), instance, NUM_FLOWS));

    Ipv4AddressHelper ipv4;

    // Sender side IP addresses
    ipv4.SetBase ("10.1.1.0", "255.255.255.0");
    i0i4 = ipv4.Assign (devn0n4);
    tch.Uninstall (devn0n4);
    ipv4.SetBase ("10.1.2.0", "255.255.255.0");
    i1i4 = ipv4.Assign (devn1n4);
    tch.Uninstall (devn1n4);
    ipv4.SetBase ("10.1.3.0", "255.255.255.0");
    i2i4 = ipv4.Assign (devn2n4);
    tch.Uninstall (devn2n4);
    ipv4.SetBase ("10.1.4.0", "255.255.255.0");
    i3i4 = ipv4.Assign (devn3n4);
    tch.Uninstall (devn3n4);

    // Bottleneck link IP address
    ipv4.SetBase ("10.1.5.0", "255.255.255.0");
    i4i5 = ipv4.Assign (devn4n5);
    tch.Uninstall (devn4n5);

    // Receiver side IP addresses
    ipv4.SetBase ("10.1.6.0", "255.255.255.0");
    i5i6 = ipv4.Assign (devn5n6);
    tch.Uninstall (devn5n6);
    ipv4.SetBase ("10.1.7.0", "255.255.255.0");
    i5i7 = ipv4.Assign (devn5n7);
    tch.Uninstall (devn5n7);
    ipv4.SetBase ("10.1.8.0", "255.255.255.0");
    i5i8 = ipv4.Assign (devn5n8);
    tch.Uninstall (devn5n8);
    ipv4.SetBase ("10.1.9.0", "255.255.255.0");
    i5i9 = ipv4.Assign (devn5n9);
    tch.Uninstall (devn5n9);

    // Set up the routing
    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

    dqc::CongestionControlType cc = kBBRv2;
    if (algo == "bbrv2plus") {
        cc = kBBRv2Plus;
    } else if (algo == "bbrv2plus_ecn") {
        cc = kBBRv2PlusEcn;
    } else if (algo == "bbrv2_noprobe_rtt") {
        cc = kBBRv2NoProbeRtt;
    }

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

    float flow_start_times[NUM_FLOWS] = {0.0, 0.0, 0.0, 0.0};

    // Flow 1: n0 -> n6
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    stat->RegisterCongestionType(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE
|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_QUEUE_DELAY);
    Ptr<DqcSender> sender1 = InstallDqc(cc,c.Get(0),c.Get(6),sendPort,recvPort,flow_start_times[0]+0.001,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender1);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 2: n1 -> n7
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE
|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_QUEUE_DELAY);
    Ptr<DqcSender> sender2 = InstallDqc(cc,c.Get(1),c.Get(7),sendPort,recvPort,flow_start_times[1]+0.002,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender2);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 3: n2 -> n8
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE
|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_QUEUE_DELAY);
    Ptr<DqcSender> sender3 = InstallDqc(cc,c.Get(2),c.Get(8),sendPort,recvPort,flow_start_times[2]+0.003,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender3);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 4: n3 -> n9
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE
|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_QUEUE_DELAY);
    Ptr<DqcSender> sender4 = InstallDqc(cc,c.Get(3),c.Get(9),sendPort,recvPort,flow_start_times[3]+0.004,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender4);
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
    std::string cc_name=DQC_DEFAULT_CC;

    // Command line arguments
    CommandLine cmd;
    cmd.AddValue("sim_time", "Simulation time in seconds", sim_time);
    cmd.AddValue("trace_path", "Output trace directory path", trace_path);
    cmd.AddValue("cc", "Congestion control: bbrv2, bbrv2_noprobe_rtt, bbrv2plus, bbrv2plus_ecn", cc_name);
    cmd.Parse(argc, argv);
    if(!trace_path.empty()){
        if(trace_path.back() != '/'){
            trace_path.push_back('/');
        }
        set_dqc_trace_folder(trace_path);
    }
    SetQueueOccupancyTraceFolder(trace_path);

    // Print configuration
    std::cout << "=== " << DQC_SCENARIO_TITLE << " Configuration ===" << std::endl;
    std::cout << "Number of flows: " << NUM_FLOWS << std::endl;
    std::cout << "Congestion control: " << cc_name << std::endl;
    std::cout << "Bottleneck bandwidth: "<<TOPO_BOTTLE_BW/1000000<<" Mbps" << std::endl;
    std::cout << "Bottleneck delay: "<<TOPO_BOTTLE_PDELAY<<" ms" << std::endl;
    std::cout << "Simulation time: " << sim_time << " seconds" << std::endl;
    std::cout << "====================================" << std::endl;

    std::vector<std::string> algos{cc_name};
    for (size_t c = 0; c < algos.size(); ++c){
        std::string cong=algos[c];
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
