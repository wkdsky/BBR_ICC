/** Network topology
 *
 *    10Mb/s, 1ms                            10Mb/s, 1ms
 * n0----L0--------|                    |------L33------n33
 *                 |   64Mbps/s, 28ms   |
 *                 n32-------L32-------n33
 *    10Mb/s, 1ms  |                    |    10Mb/s, 1ms
 * n1----L1--------|                    |------L34------n34
 *                 |                    |
 *      ...        |                    |       ...
 *                 |                    |
 * n31---L31-------|                    |------L64------n64
 *
 * 32 BBRv2 flows, 64Mbps bottleneck, 60ms RTT, buffer = 2xBDP
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
NS_LOG_COMPONENT_DEFINE ("32-bbrv2");

const int NUM_FLOWS = 32;

uint32_t checkTimes;
double avgQueueSize;

// The times
double global_start_time;
double global_stop_time;
double sink_start_time;
double sink_stop_time;
double client_start_time;
double client_stop_time;

// Node containers for sender side (n0-n31 to n32)
NodeContainer senderNodes[NUM_FLOWS];

// Bottleneck link
NodeContainer n32n33;

// Node containers for receiver side (n33 to n34-n65)
NodeContainer receiverNodes[NUM_FLOWS];

// IP interface containers
Ipv4InterfaceContainer senderInterfaces[NUM_FLOWS];
Ipv4InterfaceContainer bottleneckInterface;
Ipv4InterfaceContainer receiverInterfaces[NUM_FLOWS];

typedef struct
{
uint64_t bps;
uint32_t msDelay;
uint32_t msQdelay;
}link_config_t;

// Link configurations: L0-L31 (sender side), L32 (bottleneck), L33-L64 (receiver side)

const uint64_t TOPO_SENDER_BW       =   10 * 1000000;    // in bps
const uint64_t TOPO_SENDER_PDELAY   =   1;    // in ms
const uint64_t TOPO_BOTTLE_BW       =   64 * 1000000;    // in bps
const uint64_t TOPO_BOTTLE_PDELAY   =   28;    // in ms
const uint64_t TOPO_DEFAULT_QDELAY  =   (TOPO_SENDER_PDELAY*2+TOPO_BOTTLE_PDELAY)*2;    // in ms

link_config_t p4p[65];  // 32 sender links + 1 bottleneck + 32 receiver links

void InitializeLinks() {
    // Sender side links (L0-L31)
    for (int i = 0; i < NUM_FLOWS; i++) {
        p4p[i] = {TOPO_SENDER_BW, TOPO_SENDER_PDELAY, TOPO_DEFAULT_QDELAY};
    }

    // Bottleneck link (L32)
    p4p[32] = {TOPO_BOTTLE_BW, TOPO_BOTTLE_PDELAY, TOPO_DEFAULT_QDELAY};

    // Receiver side links (L33-L64)
    for (int i = 33; i < 65; i++) {
        p4p[i] = {TOPO_SENDER_BW, TOPO_SENDER_PDELAY, TOPO_DEFAULT_QDELAY};
    }
}


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
    }
    return sendApp;
}

void ns3_bbrv2(int ins, std::string algo, DqcTraceState *stat, int sim_time=60, int loss_integer=0){
    std::string instance="32_bbrv2";  // Use script filename instead of instance number
    uint64_t linkBw   = p4p[32].bps;
    uint16_t sendPort=1000;
    uint16_t recvPort=5000;

    double sim_dur=sim_time;
    int end_time=sim_time;
    float appStop=end_time;

    NodeContainer c;
    c.Create (66);  // 66 nodes: n0-n31 (senders), n32-n33 (routers), n34-n65 (receivers)

    // Sender side connections
    for (int i = 0; i < NUM_FLOWS; i++) {
        senderNodes[i] = NodeContainer(c.Get(i), c.Get(32));
    }

    // Bottleneck link
    n32n33 = NodeContainer (c.Get (32), c.Get (33));

    // Receiver side connections
    for (int i = 0; i < NUM_FLOWS; i++) {
        receiverNodes[i] = NodeContainer(c.Get(33), c.Get(34 + i));
    }

    uint32_t bufSize=0;

    InternetStackHelper internet;
    internet.Install (c);

    NS_LOG_INFO ("Create channels");
    PointToPointHelper p2p;
    TrafficControlHelper tch;
    std::vector<std::shared_ptr<QueueOccupancyTracer>> queue_tracers;

    //L0-L31 and L33-L64: Edge links (10Mbps)
    bufSize =TOPO_SENDER_BW * TOPO_DEFAULT_QDELAY / 8000;
    p2p.SetQueue ("ns3::DropTailQueue",
                "Mode", StringValue ("QUEUE_MODE_BYTES"),
                "MaxBytes", UintegerValue (bufSize));
    p2p.SetDeviceAttribute ("DataRate", DataRateValue(DataRate (TOPO_SENDER_BW)));
    p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (TOPO_SENDER_PDELAY)));

    // Sender side edge links
    NetDeviceContainer senderDevices[NUM_FLOWS];
    for (int i = 0; i < NUM_FLOWS; i++) {
        senderDevices[i] = p2p.Install(senderNodes[i]);
    }

    // Receiver side edge links
    NetDeviceContainer receiverDevices[NUM_FLOWS];
    for (int i = 0; i < NUM_FLOWS; i++) {
        receiverDevices[i] = p2p.Install(receiverNodes[i]);
    }

    //L32: Bottleneck link (64Mbps)
    bufSize = TOPO_BOTTLE_BW * TOPO_DEFAULT_QDELAY / 8000;//与msQdelay相关，这里代表1个BDP
    p2p.SetQueue ("ns3::DropTailQueue",
                "Mode", StringValue ("QUEUE_MODE_BYTES"),
                "MaxBytes", UintegerValue (bufSize));
    p2p.SetDeviceAttribute ("DataRate", DataRateValue(DataRate (TOPO_BOTTLE_BW)));
    p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (TOPO_BOTTLE_PDELAY)));
    NetDeviceContainer devn32n33 = p2p.Install (n32n33);
    queue_tracers.push_back(InstallBottleneckQueueOccupancyTrace(devn32n33.Get(0), instance, NUM_FLOWS));

    Ipv4AddressHelper ipv4;

    // Sender side IP addresses
    for (int i = 0; i < NUM_FLOWS; i++) {
        std::ostringstream subnet;
        subnet << "10.1." << (i + 1) << ".0";
        ipv4.SetBase(subnet.str().c_str(), "255.255.255.0");
        senderInterfaces[i] = ipv4.Assign(senderDevices[i]);
        tch.Uninstall(senderDevices[i]);
    }

    // Bottleneck link IP address
    ipv4.SetBase ("10.1.33.0", "255.255.255.0");
    bottleneckInterface = ipv4.Assign (devn32n33);
    tch.Uninstall (devn32n33);

    // Receiver side IP addresses
    for (int i = 0; i < NUM_FLOWS; i++) {
        std::ostringstream subnet;
        subnet << "10.1." << (34 + i) << ".0";
        ipv4.SetBase(subnet.str().c_str(), "255.255.255.0");
        receiverInterfaces[i] = ipv4.Assign(receiverDevices[i]);
        tch.Uninstall(receiverDevices[i]);
    }

    // Set up the routing
    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

    // Use BBRv2 for all flows
    dqc::CongestionControlType cc = kBBRv2;

    uint32_t max_bps=0;
    int test_pair=1;
    uint32_t sender_id=1;

    std::vector<std::unique_ptr<DqcTrace>> traces;
    std::vector<Ptr<DqcSender>> senders;
    std::string log;
    std::string delimiter="_";
    std::string prefix=instance+delimiter;  // instance already includes script name and algorithm

    // Create all 32 flows
    for (int i = 0; i < NUM_FLOWS; i++) {
        log = prefix + std::to_string(test_pair);
        std::unique_ptr<DqcTrace> trace(new DqcTrace(test_pair));
        stat->ReisterAvgDelayId(test_pair);
        if (i == 0) {
            stat->RegisterCongestionType(test_pair);
        }
        trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE|DqcTraceEnable::E_DQC_QUEUE_DELAY);

        float flow_start_time = 0.001 * (i + 1);
        Ptr<DqcSender> sender = InstallDqc(cc, c.Get(i), c.Get(34 + i),
                                           sendPort, recvPort,
                                           flow_start_time, appStop,
                                           trace.get(), stat, max_bps, sender_id);
        senders.push_back(sender);
        sender_id++;
        test_pair++;
        sendPort++;
        recvPort++;
        traces.push_back(std::move(trace));
    }

    Simulator::Stop (Seconds(sim_dur));
    Simulator::Run ();
    Simulator::Destroy();
    stat->Flush(linkBw,sim_dur);
}

int main (int argc, char *argv[]){
    int sim_time=30;
    int ins[]={1};
    std::string trace_path="";

    // Initialize link configurations
    InitializeLinks();

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
    std::cout << "=== 32 BBRv2 Flows Configuration ===" << std::endl;
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
