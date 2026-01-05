/** Network topology (same as bbr-rtt.cc)
 *
 *    10Mb/s, 2ms                            10Mb/s, 2ms
 * n0------l0------|                    |-------l3------n4
 *                 |    8Mbps/s, 16ms   |
 *                 n2--------l2--------n3
 *    10Mb/s, 2ms  |                    |    10Mb/s, 2ms
 * n1------l1------|                    |-------l4------n5
 *
 * 2 FreqCCv2 flows with different RTT configurations
 * FreqCCv2 features:
 *   - PROBE_DOWN pacing gain: 0.5 (vs BBRv2's 0.75)
 *   - PROBE_UP pacing gain: 1.1 (vs BBRv2's 1.25)
 *   - PROBE_DOWN exit: RTT <= 1.05*min_RTT OR duration >= 2*min_RTT
 *   - Supports oscillation modes: after_drain, only_probeBW, refill_up
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
using namespace ns3;
using namespace dqc;
using namespace std;
NS_LOG_COMPONENT_DEFINE ("freqccv2-rtt");

const int NUM_FLOWS = 2;

uint32_t checkTimes;
double avgQueueSize;

// The times
double global_start_time;
double global_stop_time;
double sink_start_time;
double sink_stop_time;
double client_start_time;
double client_stop_time;

NodeContainer n0n2;
NodeContainer n2n3;
NodeContainer n3n4;
NodeContainer n1n2;
NodeContainer n3n5;

Ipv4InterfaceContainer i0i2;
Ipv4InterfaceContainer i1i2;
Ipv4InterfaceContainer i2i3;
Ipv4InterfaceContainer i3i4;
Ipv4InterfaceContainer i3i5;

typedef struct
{
uint64_t bps;
uint32_t msDelay;
uint32_t msQdelay;
}link_config_t;
//unrelated topology
/*
   L3      L1      L4
configuration same as the above dumbbell topology
n0--L0--n2--L1--n3--L2--n4
n1--L3--n2--L1--n3--L4--n5
*/
link_config_t p4p[]={
[0]={10*1000000,2,50},
[1]={10*1000000,2,50},
[2]={8*1000000,16,50},
[3]={10*1000000,2,50},
[4]={10*1000000,2,50},
};

// FreqCCv2 configuration parameters for each flow
double g_freqccv2_freq[NUM_FLOWS] = {60.0, 60.0};       // Oscillation frequency in Hz
std::string g_freqccv2_amplitude[NUM_FLOWS] = {"0", "0"};    // Amplitude mode
double g_freqccv2_fixed_amplitude[NUM_FLOWS] = {8.0/NUM_FLOWS*0.5, 8.0/NUM_FLOWS*0.5}; // Fixed amplitude in Mbps
std::string g_freqccv2_osc_mode[NUM_FLOWS] = {"refill_up", "refill_up"};  // Oscillation mode

static Ptr<DqcSender> InstallDqc( dqc::CongestionControlType cc_type,
                        Ptr<Node> sender,Ptr<Node> receiver,
                        uint16_t send_port,uint16_t recv_port,
                        float startTime,float stopTime,
                        DqcTrace *trace, DqcTraceState *stat,
                        uint32_t max_bps=0,uint32_t cid=0,bool ecn=false,uint32_t emucons=1)
{
    Ptr<DqcSender> sendApp = CreateObject<DqcSender> (cc_type,ecn);
    Ptr<DqcReceiver> recvApp = CreateObject<DqcReceiver>();
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
        sendApp->SetSendRateTraceFuc(MakeCallback(&DqcTrace::OnSendRate,trace));
        sendApp->SetRecvRateTraceFuc(MakeCallback(&DqcTrace::OnRecvRate,trace));
        sendApp->SetBbrModeTraceFuc(MakeCallback(&DqcTrace::OnBbrMode,trace));
        recvApp->SetOwdTraceFuc(MakeCallback(&DqcTrace::OnOwd,trace));
        recvApp->SetGoodputTraceFuc(MakeCallback(&DqcTrace::OnGoodput,trace));
        recvApp->SetStatsTraceFuc(MakeCallback(&DqcTrace::OnStats,trace));
        trace->SetStatsTraceFuc(MakeCallback(&DqcTraceState::OnStats,stat));
    }
    return sendApp;
}

void ns3_freqccv2(int ins,std::string algo,DqcTraceState *stat,int sim_time=200,int loss_integer=0){
    std::string instance=std::to_string(ins);
    uint64_t linkBw   = p4p[2].bps;
    uint16_t sendPort=1000;
    uint16_t recvPort=5000;

    double sim_dur=sim_time;
    int start_time=0;
    int end_time=sim_time;
    float appStart=start_time;
    float appStop=end_time;

    NodeContainer c;
    c.Create (6);
    n0n2 = NodeContainer (c.Get (0), c.Get (2));//l0
    n1n2 = NodeContainer (c.Get (1), c.Get (2));//l1
    n2n3 = NodeContainer (c.Get (2), c.Get (3));//l2
    n3n4 = NodeContainer (c.Get (3), c.Get (4));//l3
    n3n5 = NodeContainer (c.Get (3), c.Get (5));//l4
    link_config_t *config=p4p;
    uint32_t bufSize=0;

    InternetStackHelper internet;
    internet.Install (c);

    NS_LOG_INFO ("Create channels");
    PointToPointHelper p2p;
    TrafficControlHelper tch;
    //L0
    bufSize =config[0].bps * config[0].msQdelay/8000;//1BDP
    p2p.SetQueue ("ns3::DropTailQueue",
                "Mode", StringValue ("QUEUE_MODE_BYTES"),
                "MaxBytes", UintegerValue (bufSize));
    p2p.SetDeviceAttribute ("DataRate", DataRateValue(DataRate (config[0].bps)));
    p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (config[0].msDelay)));
    NetDeviceContainer devn0n2 = p2p.Install (n0n2);
    //L1
    bufSize =config[1].bps * config[1].msQdelay/8000;
    p2p.SetQueue ("ns3::DropTailQueue",
                "Mode", StringValue ("QUEUE_MODE_BYTES"),
                "MaxBytes", UintegerValue (bufSize));
    p2p.SetDeviceAttribute ("DataRate", DataRateValue(DataRate (config[1].bps)));
    p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (config[1].msDelay)));
    NetDeviceContainer devn1n2 = p2p.Install (n1n2);
    //L2
    bufSize =config[2].bps * config[2].msQdelay/8000;
    p2p.SetQueue ("ns3::DropTailQueue",
                "Mode", StringValue ("QUEUE_MODE_BYTES"),
                "MaxBytes", UintegerValue (bufSize));
    p2p.SetDeviceAttribute ("DataRate", DataRateValue(DataRate (config[2].bps)));
    p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (config[2].msDelay)));
    NetDeviceContainer devn2n3 = p2p.Install (n2n3);
    //L3
    bufSize =config[3].bps * config[3].msQdelay/8000;
    p2p.SetQueue ("ns3::DropTailQueue",
                "Mode", StringValue ("QUEUE_MODE_BYTES"),
                "MaxBytes", UintegerValue (bufSize));
    p2p.SetDeviceAttribute ("DataRate", DataRateValue(DataRate (config[3].bps)));
    p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (config[3].msDelay)));
    NetDeviceContainer devn3n4 = p2p.Install (n3n4);
    //L4
    bufSize =config[4].bps * config[4].msQdelay/8000;
    p2p.SetQueue ("ns3::DropTailQueue",
                "Mode", StringValue ("QUEUE_MODE_BYTES"),
                "MaxBytes", UintegerValue (bufSize));
    p2p.SetDeviceAttribute ("DataRate", DataRateValue(DataRate (config[4].bps)));
    p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (config[4].msDelay)));
    NetDeviceContainer devn3n5 = p2p.Install (n3n5);

    Ipv4AddressHelper ipv4;

    ipv4.SetBase ("10.1.1.0", "255.255.255.0");
    i0i2 = ipv4.Assign (devn0n2);
    tch.Uninstall (devn0n2);

    ipv4.SetBase ("10.1.2.0", "255.255.255.0");
    i1i2 = ipv4.Assign (devn1n2);
    tch.Uninstall (devn1n2);

    ipv4.SetBase ("10.1.3.0", "255.255.255.0");
    i2i3 = ipv4.Assign (devn2n3);
    tch.Uninstall (devn2n3);

    ipv4.SetBase ("10.1.4.0", "255.255.255.0");
    i3i4 = ipv4.Assign (devn3n4);
    tch.Uninstall (devn3n4);

    ipv4.SetBase ("10.1.5.0", "255.255.255.0");
    i3i5 = ipv4.Assign (devn3n5);
    tch.Uninstall (devn3n5);

    // Set up the routing
    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

    // Use FreqCCv2 congestion control
    dqc::CongestionControlType cc = kFreqCCv2;

    uint32_t max_bps=0;
    int test_pair=1;
    uint32_t sender_id=1;

    std::vector<std::unique_ptr<DqcTrace>> traces;
    std::vector<Ptr<DqcSender>> senders;
    std::string log;
    std::string delimiter="_";
    std::string prefix=instance+delimiter+algo+delimiter+"rtt"+delimiter;
    log=prefix+std::to_string(test_pair);
    std::unique_ptr<DqcTrace> trace;

    // Flow 1: n0 -> n4
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    stat->RegisterCongestionType(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_BBR_MODE);

    Ptr<DqcSender> sender1 = InstallDqc(cc,c.Get(0),c.Get(4),sendPort,recvPort,appStart+0.01,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender1);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 2: n1 -> n5
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_BBR_MODE);
    Ptr<DqcSender> sender2 = InstallDqc(cc,c.Get(1),c.Get(5),sendPort,recvPort,appStart+0.01,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender2);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Configure FreqCCv2 parameters for each sender
    for(int i = 0; i < NUM_FLOWS; i++){
        Simulator::Schedule(Seconds(appStart + 0.001), &DqcSender::ConfigureFreqCC, senders[i],
            g_freqccv2_freq[i], g_freqccv2_amplitude[i], g_freqccv2_fixed_amplitude[i], g_freqccv2_osc_mode[i]);
    }

    Simulator::Stop (Seconds(sim_dur));
    Simulator::Run ();
    Simulator::Destroy();
    stat->Flush(linkBw,sim_dur);
}

int main (int argc, char *argv[]){
    int sim_time=50;
    int ins[]={1};

    // Print configuration
    std::cout << "=== FreqCCv2-RTT Configuration ===" << std::endl;
    std::cout << std::endl;
    for(int i = 0; i < NUM_FLOWS; i++){
        std::cout << "Flow " << (i+1) << ": freq=" << g_freqccv2_freq[i]
                  << " Hz, amp=" << g_freqccv2_amplitude[i]
                  << ", fixed=" << g_freqccv2_fixed_amplitude[i] << " Mbps"
                  << ", mode=" << g_freqccv2_osc_mode[i] << std::endl;
    }
    std::cout << "Simulation time: " << sim_time << " seconds" << std::endl;
    std::cout << "==================================" << std::endl;

    const char *algos[]={"freqccv2"};

    for(size_t c=0;c<sizeof(algos)/sizeof(algos[0]);c++){

        std::string cong=std::string(algos[c]);
        std::string name=cong;
        std::unique_ptr<DqcTraceState> stat;
        stat.reset(new DqcTraceState(name));
        auto inner_start = std::chrono::high_resolution_clock::now();

        for(size_t i=0; i < sizeof(ins)/sizeof(ins[0]); i++){
            ns3_freqccv2(ins[i],cong,stat.get(),sim_time);
        }
        auto inner_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> tm = inner_end - inner_start;
        std::chrono::duration<double, std::ratio<60>> minutes =inner_end- inner_start;

        stat->RecordRuningTime(tm.count(),minutes.count());
    }
    return 0;
}
