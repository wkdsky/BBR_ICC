/** Network topology
 *
 * n0----L0--------|                    |------L17------n18
 *                 |                    |
 * n1----L1--------|                    |------L18------n19
 *                 |                    |
 * n2----L2--------|                    |------L19------n20
 *                 |                    |
 * n3----L3--------|                    |------L20------n21
 *                 |                    |
 * n4----L4--------|                    |------L21------n22
 *                 |                    |
 * n5----L5--------|                    |------L22------n23
 *                 |                    |
 * n6----L6--------|                    |------L23------n24
 *                 |                    |
 * n7----L7--------|                    |------L24------n25
 *                 |                    |
 * n8----L8--------|   n16-----L16-----n17   |------L25------n26
 *                 |                    |
 * n9----L9--------|                    |------L26------n27
 *                 |                    |
 * n10---L10-------|                    |------L27------n28
 *                 |                    |
 * n11---L11-------|                    |------L28------n29
 *                 |                    |
 * n12---L12-------|                    |------L29------n30
 *                 |                    |
 * n13---L13-------|                    |------L30------n31
 *                 |                    |
 * n14---L14-------|                    |------L31------n32
 *                 |                    |
 * n15---L15-------|                    |------L32------n33
 *
 * 16 FreqCCv3 flows, bottleneck link L16
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
#define DQC_SCENARIO_LOG_COMPONENT "freqccv3-16flow"
#endif
#ifndef DQC_SCENARIO_INSTANCE
#define DQC_SCENARIO_INSTANCE "freqccv3_16flow"
#endif
#ifndef DQC_SCENARIO_TITLE
#define DQC_SCENARIO_TITLE "16 FreqCCv3 Flows"
#endif
NS_LOG_COMPONENT_DEFINE (DQC_SCENARIO_LOG_COMPONENT);

const int NUM_FLOWS = 16;
const int BOTTLENECK_LEFT = NUM_FLOWS;
const int BOTTLENECK_RIGHT = NUM_FLOWS + 1;
const int RECEIVER_BASE = NUM_FLOWS + 2;
const int TOTAL_NODES = 2 * NUM_FLOWS + 2;

uint32_t checkTimes;
double avgQueueSize;

// The times
double global_start_time;
double global_stop_time;
double sink_start_time;
double sink_stop_time;
double client_start_time;
double client_stop_time;

NodeContainer senderNodes[NUM_FLOWS];
NodeContainer bottleneckNodes;
NodeContainer receiverNodes[NUM_FLOWS];

Ipv4InterfaceContainer senderInterfaces[NUM_FLOWS];
Ipv4InterfaceContainer bottleneckInterface;
Ipv4InterfaceContainer receiverInterfaces[NUM_FLOWS];

typedef struct
{
uint64_t bps;
uint32_t msDelay;
uint32_t msQdelay;
}link_config_t;

const uint64_t TOPO_SENDER_BW       =   10 * 1000000;    // in bps
const uint64_t TOPO_SENDER_PDELAY   =   1;    // in ms
const uint64_t TOPO_BOTTLE_BW       =   32 * 1000000;    // in bps
const uint64_t TOPO_BOTTLE_PDELAY   =   18;    // in ms
const uint64_t TOPO_DEFAULT_QDELAY  =   (TOPO_SENDER_PDELAY*2+TOPO_BOTTLE_PDELAY)*2;    // in ms

link_config_t p4p[2 * NUM_FLOWS + 1];

void InitializeLinks() {
    for (int i = 0; i < NUM_FLOWS; i++) {
        p4p[i] = {TOPO_SENDER_BW, TOPO_SENDER_PDELAY, TOPO_DEFAULT_QDELAY};
    }

    p4p[NUM_FLOWS] = {TOPO_BOTTLE_BW, TOPO_BOTTLE_PDELAY, TOPO_DEFAULT_QDELAY};

    for (int i = NUM_FLOWS + 1; i < 2 * NUM_FLOWS + 1; i++) {
        p4p[i] = {TOPO_SENDER_BW, TOPO_SENDER_PDELAY, TOPO_DEFAULT_QDELAY};
    }
}

double g_freq_hz[NUM_FLOWS] = {
    60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0,
    60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0
};
std::string g_amp_mode[NUM_FLOWS] = {
    "miu2", "miu2", "miu2", "miu2", "miu2", "miu2", "miu2", "miu2",
    "miu2", "miu2", "miu2", "miu2", "miu2", "miu2", "miu2", "miu2"
};
double g_fixed_mbps[NUM_FLOWS] = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};
double g_interval_window_rtt_mult = 1.0;

static Ptr<DqcSender> InstallDqc( dqc::CongestionControlType cc_type,
                        Ptr<Node> sender,Ptr<Node> receiver,
                        uint16_t send_port,uint16_t recv_port,
                        float startTime,float stopTime,
                        DqcTrace *trace, DqcTraceState *stat,
                        double freq_hz, const std::string& amp_mode, double fixed_mbps,
                        uint32_t max_bps=0,uint32_t cid=0,bool ecn=false,uint32_t emucons=1)
{
    Ptr<DqcSender> sendApp = CreateObject<DqcSender> (cc_type,ecn);
    Ptr<DqcReceiver> recvApp = CreateObject<DqcReceiver>(100);
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
        sendApp->SetUpPhaseTraceFuc(MakeCallback(&DqcTrace::OnUpPhase,trace));
        sendApp->SetFreqAnalysisTraceFuc(MakeCallback(&DqcTrace::OnFreqAnalysis,trace));
        sendApp->SetRttFreqAnalysisTraceFuc(MakeCallback(&DqcTrace::OnRttFreqAnalysis,trace));
        recvApp->SetGoodputTraceFuc(MakeCallback(&DqcTrace::OnGoodput,trace));
        sendApp->SetLossRateTraceFuc(MakeCallback(&DqcTrace::OnLossRate,trace));
    }
    if(cc_type == kFreqCCv3){
        sendApp->ConfigureFreqCC(freq_hz, amp_mode, fixed_mbps);
        sendApp->SetFreqCCIntervalWindowMultiplier(g_interval_window_rtt_mult);
    }
    return sendApp;
}

void ns3_freqccv3(int ins, std::string algo, DqcTraceState *stat, int sim_time=60, int loss_integer=0){
    std::string instance = DQC_SCENARIO_INSTANCE;
    uint64_t linkBw = p4p[NUM_FLOWS].bps;
    uint16_t sendPort = 1000;
    uint16_t recvPort = 5000;

    double sim_dur = sim_time;
    float appStop = sim_time;

    NodeContainer c;
    c.Create (TOTAL_NODES);

    for (int i = 0; i < NUM_FLOWS; i++) {
        senderNodes[i] = NodeContainer(c.Get(i), c.Get(BOTTLENECK_LEFT));
    }

    bottleneckNodes = NodeContainer(c.Get(BOTTLENECK_LEFT), c.Get(BOTTLENECK_RIGHT));

    for (int i = 0; i < NUM_FLOWS; i++) {
        receiverNodes[i] = NodeContainer(c.Get(BOTTLENECK_RIGHT), c.Get(RECEIVER_BASE + i));
    }

    InternetStackHelper internet;
    internet.Install (c);

    NS_LOG_INFO ("Create channels");
    PointToPointHelper p2p;
    TrafficControlHelper tch;
    std::vector<std::shared_ptr<QueueOccupancyTracer>> queue_tracers;

    uint32_t bufSize = TOPO_SENDER_BW * TOPO_DEFAULT_QDELAY / 8000;
    p2p.SetQueue ("ns3::DropTailQueue",
                "Mode", StringValue ("QUEUE_MODE_BYTES"),
                "MaxBytes", UintegerValue (bufSize));
    p2p.SetDeviceAttribute ("DataRate", DataRateValue(DataRate (TOPO_SENDER_BW)));
    p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (TOPO_SENDER_PDELAY)));

    NetDeviceContainer senderDevices[NUM_FLOWS];
    for (int i = 0; i < NUM_FLOWS; i++) {
        senderDevices[i] = p2p.Install(senderNodes[i]);
    }

    NetDeviceContainer receiverDevices[NUM_FLOWS];
    for (int i = 0; i < NUM_FLOWS; i++) {
        receiverDevices[i] = p2p.Install(receiverNodes[i]);
    }

    bufSize = TOPO_BOTTLE_BW * TOPO_DEFAULT_QDELAY / 8000;
    p2p.SetQueue ("ns3::DropTailQueue",
                "Mode", StringValue ("QUEUE_MODE_BYTES"),
                "MaxBytes", UintegerValue (bufSize));
    p2p.SetDeviceAttribute ("DataRate", DataRateValue(DataRate (TOPO_BOTTLE_BW)));
    p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (TOPO_BOTTLE_PDELAY)));
    NetDeviceContainer bottleneckDevices = p2p.Install (bottleneckNodes);
    queue_tracers.push_back(InstallBottleneckQueueOccupancyTrace(bottleneckDevices.Get(0), instance, NUM_FLOWS));

    Ipv4AddressHelper ipv4;

    for (int i = 0; i < NUM_FLOWS; i++) {
        std::ostringstream subnet;
        subnet << "10.1." << (i + 1) << ".0";
        ipv4.SetBase(subnet.str().c_str(), "255.255.255.0");
        senderInterfaces[i] = ipv4.Assign(senderDevices[i]);
        tch.Uninstall(senderDevices[i]);
    }

    {
        std::ostringstream subnet;
        subnet << "10.1." << (NUM_FLOWS + 1) << ".0";
        ipv4.SetBase(subnet.str().c_str(), "255.255.255.0");
        bottleneckInterface = ipv4.Assign (bottleneckDevices);
        tch.Uninstall (bottleneckDevices);
    }

    for (int i = 0; i < NUM_FLOWS; i++) {
        std::ostringstream subnet;
        subnet << "10.1." << (NUM_FLOWS + 2 + i) << ".0";
        ipv4.SetBase(subnet.str().c_str(), "255.255.255.0");
        receiverInterfaces[i] = ipv4.Assign(receiverDevices[i]);
        tch.Uninstall(receiverDevices[i]);
    }

    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

    dqc::CongestionControlType cc = kFreqCCv3;
    uint32_t max_bps = 0;
    int test_pair = 1;
    uint32_t sender_id = 1;

    std::vector<std::unique_ptr<DqcTrace>> traces;
    std::vector<Ptr<DqcSender>> senders;
    std::string prefix = instance + "_";

    for (int i = 0; i < NUM_FLOWS; i++) {
        std::string log = prefix + std::to_string(test_pair);
        std::unique_ptr<DqcTrace> trace(new DqcTrace(test_pair));
        stat->ReisterAvgDelayId(test_pair);
        if (i == 0) {
            stat->RegisterCongestionType(test_pair);
        }
        trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_UP_PHASE|DqcTraceEnable::E_DQC_FREQ_ANALYSIS|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_QUEUE_DELAY);

        float flow_start_time = 0.001 * (i + 1);
        Ptr<DqcSender> sender = InstallDqc(cc,
                                           c.Get(i),
                                           c.Get(RECEIVER_BASE + i),
                                           sendPort,
                                           recvPort,
                                           flow_start_time,
                                           appStop,
                                           trace.get(),
                                           stat,
                                           g_freq_hz[i],
                                           g_amp_mode[i],
                                           g_fixed_mbps[i],
                                           max_bps,
                                           sender_id);
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
    int sim_time = 30;
    int ins[] = {1};
    std::string trace_path = "";

    InitializeLinks();

    CommandLine cmd;
    cmd.AddValue("sim_time", "Simulation time in seconds", sim_time);
    cmd.AddValue("trace_path", "Output trace directory path", trace_path);
    for (int i = 0; i < NUM_FLOWS; i++) {
        std::string idx = std::to_string(i + 1);
        cmd.AddValue("freq" + idx, "Flow " + idx + " oscillation frequency (Hz)", g_freq_hz[i]);
        cmd.AddValue("amp" + idx, "Flow " + idx + " amplitude mode", g_amp_mode[i]);
        cmd.AddValue("fixed" + idx, "Flow " + idx + " fixed amplitude (Mbps)", g_fixed_mbps[i]);
    }
    cmd.AddValue("interval_win_rtt_mult", "Interval-phase STFT window multiplier on min_rtt", g_interval_window_rtt_mult);
    cmd.Parse(argc, argv);
    if(!trace_path.empty()){
        if(trace_path.back() != '/'){
            trace_path.push_back('/');
        }
        set_dqc_trace_folder(trace_path);
    }
    SetQueueOccupancyTraceFolder(trace_path);

    std::cout << "=== " << DQC_SCENARIO_TITLE << " Configuration ===" << std::endl;
    std::cout << "Number of flows: " << NUM_FLOWS << std::endl;
    std::cout << "Congestion control: FreqCCv3" << std::endl;
    std::cout << "Bottleneck bandwidth: "<<TOPO_BOTTLE_BW/1000000<<" Mbps" << std::endl;
    std::cout << "Bottleneck delay: "<<TOPO_BOTTLE_PDELAY<<" ms" << std::endl;
    std::cout << "Simulation time: " << sim_time << " seconds" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Per-flow parameters:" << std::endl;
    for(int i = 0; i < NUM_FLOWS; i++){
        std::cout << "  Flow " << (i+1) << ": freq=" << g_freq_hz[i] << "Hz, amp=" << g_amp_mode[i] << ", fixed=" << g_fixed_mbps[i] << "Mbps" << std::endl;
    }
    std::cout << "Interval window multiplier: " << g_interval_window_rtt_mult << " * min_rtt" << std::endl;
    std::cout << "====================================" << std::endl;

    const char *algos[] = {"freqccv3"};
    for (size_t c = 0; c < sizeof(algos) / sizeof(algos[0]); ++c){
        std::string cong = std::string(algos[c]);
        std::string name = cong;
        std::unique_ptr<DqcTraceState> stat;
        stat.reset(new DqcTraceState(name));
        auto inner_start = std::chrono::high_resolution_clock::now();
        for (size_t i = 0; i < sizeof(ins) / sizeof(ins[0]); ++i){
            ns3_freqccv3(ins[i],cong,stat.get(),sim_time);
        }
        auto inner_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> tm = inner_end - inner_start;
        std::chrono::duration<double, std::ratio<60>> minutes = inner_end - inner_start;
        stat->RecordRuningTime(tm.count(),minutes.count());
    }
    return 0;
}
