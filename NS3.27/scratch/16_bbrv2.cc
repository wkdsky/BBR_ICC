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
 * 16 BBRv2 flows, bottleneck link L16
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
NS_LOG_COMPONENT_DEFINE ("16-bbrv2");

const int NUM_FLOWS = 16;

uint32_t checkTimes;
double avgQueueSize;

// The times
double global_start_time;
double global_stop_time;
double sink_start_time;
double sink_stop_time;
double client_start_time;
double client_stop_time;

// Node containers for sender side (n0-n15 to n16)
NodeContainer n0n16;
NodeContainer n1n16;
NodeContainer n2n16;
NodeContainer n3n16;
NodeContainer n4n16;
NodeContainer n5n16;
NodeContainer n6n16;
NodeContainer n7n16;
NodeContainer n8n16;
NodeContainer n9n16;
NodeContainer n10n16;
NodeContainer n11n16;
NodeContainer n12n16;
NodeContainer n13n16;
NodeContainer n14n16;
NodeContainer n15n16;

// Bottleneck link
NodeContainer n16n17;

// Node containers for receiver side (n17 to n18-n33)
NodeContainer n17n18;
NodeContainer n17n19;
NodeContainer n17n20;
NodeContainer n17n21;
NodeContainer n17n22;
NodeContainer n17n23;
NodeContainer n17n24;
NodeContainer n17n25;
NodeContainer n17n26;
NodeContainer n17n27;
NodeContainer n17n28;
NodeContainer n17n29;
NodeContainer n17n30;
NodeContainer n17n31;
NodeContainer n17n32;
NodeContainer n17n33;

// IP interface containers
Ipv4InterfaceContainer i0i16;
Ipv4InterfaceContainer i1i16;
Ipv4InterfaceContainer i2i16;
Ipv4InterfaceContainer i3i16;
Ipv4InterfaceContainer i4i16;
Ipv4InterfaceContainer i5i16;
Ipv4InterfaceContainer i6i16;
Ipv4InterfaceContainer i7i16;
Ipv4InterfaceContainer i8i16;
Ipv4InterfaceContainer i9i16;
Ipv4InterfaceContainer i10i16;
Ipv4InterfaceContainer i11i16;
Ipv4InterfaceContainer i12i16;
Ipv4InterfaceContainer i13i16;
Ipv4InterfaceContainer i14i16;
Ipv4InterfaceContainer i15i16;
Ipv4InterfaceContainer i16i17;
Ipv4InterfaceContainer i17i18;
Ipv4InterfaceContainer i17i19;
Ipv4InterfaceContainer i17i20;
Ipv4InterfaceContainer i17i21;
Ipv4InterfaceContainer i17i22;
Ipv4InterfaceContainer i17i23;
Ipv4InterfaceContainer i17i24;
Ipv4InterfaceContainer i17i25;
Ipv4InterfaceContainer i17i26;
Ipv4InterfaceContainer i17i27;
Ipv4InterfaceContainer i17i28;
Ipv4InterfaceContainer i17i29;
Ipv4InterfaceContainer i17i30;
Ipv4InterfaceContainer i17i31;
Ipv4InterfaceContainer i17i32;
Ipv4InterfaceContainer i17i33;

typedef struct
{
uint64_t bps;
uint32_t msDelay;
uint32_t msQdelay;
}link_config_t;

// Link configurations: L0-L15 (sender side), L16 (bottleneck), L17-L32 (receiver side)

const uint64_t TOPO_SENDER_BW       =   10 * 1000000;    // in bps
const uint64_t TOPO_SENDER_PDELAY   =   1;    // in ms
const uint64_t TOPO_BOTTLE_BW       =   32 * 1000000;    // in bps
const uint64_t TOPO_BOTTLE_PDELAY   =   18;    // in ms
const uint64_t TOPO_DEFAULT_QDELAY  =   (TOPO_SENDER_PDELAY*2+TOPO_BOTTLE_PDELAY)*2;    // in ms

link_config_t p4p[]={
[0]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L0: n0-n16
[1]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L1: n1-n16
[2]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L2: n2-n16
[3]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L3: n3-n16
[4]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L4: n4-n16
[5]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L5: n5-n16
[6]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L6: n6-n16
[7]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L7: n7-n16
[8]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L8: n8-n16
[9]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},   // L9: n9-n16
[10]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L10: n10-n16
[11]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L11: n11-n16
[12]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L12: n12-n16
[13]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L13: n13-n16
[14]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L14: n14-n16
[15]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L15: n15-n16
[16]={TOPO_BOTTLE_BW,TOPO_BOTTLE_PDELAY,TOPO_DEFAULT_QDELAY},  // L16: n16-n17 (bottleneck)
[17]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L17: n17-n18
[18]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L18: n17-n19
[19]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L19: n17-n20
[20]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L20: n17-n21
[21]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L21: n17-n22
[22]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L22: n17-n23
[23]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L23: n17-n24
[24]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L24: n17-n25
[25]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L25: n17-n26
[26]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L26: n17-n27
[27]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L27: n17-n28
[28]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L28: n17-n29
[29]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L29: n17-n30
[30]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L30: n17-n31
[31]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L31: n17-n32
[32]={TOPO_SENDER_BW,TOPO_SENDER_PDELAY,TOPO_DEFAULT_QDELAY},  // L32: n17-n33
};


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
    std::string instance="16_bbrv2";  // Use script filename instead of instance number
    uint64_t linkBw   = p4p[16].bps;
    uint16_t sendPort=1000;
    uint16_t recvPort=5000;

    double sim_dur=sim_time;
    int end_time=sim_time;
    float appStop=end_time;

    NodeContainer c;
    c.Create (34);  // 34 nodes: n0-n15 (senders), n16-n17 (routers), n18-n33 (receivers)

    // Sender side connections
    n0n16 = NodeContainer (c.Get (0), c.Get (16));
    n1n16 = NodeContainer (c.Get (1), c.Get (16));
    n2n16 = NodeContainer (c.Get (2), c.Get (16));
    n3n16 = NodeContainer (c.Get (3), c.Get (16));
    n4n16 = NodeContainer (c.Get (4), c.Get (16));
    n5n16 = NodeContainer (c.Get (5), c.Get (16));
    n6n16 = NodeContainer (c.Get (6), c.Get (16));
    n7n16 = NodeContainer (c.Get (7), c.Get (16));
    n8n16 = NodeContainer (c.Get (8), c.Get (16));
    n9n16 = NodeContainer (c.Get (9), c.Get (16));
    n10n16 = NodeContainer (c.Get (10), c.Get (16));
    n11n16 = NodeContainer (c.Get (11), c.Get (16));
    n12n16 = NodeContainer (c.Get (12), c.Get (16));
    n13n16 = NodeContainer (c.Get (13), c.Get (16));
    n14n16 = NodeContainer (c.Get (14), c.Get (16));
    n15n16 = NodeContainer (c.Get (15), c.Get (16));

    // Bottleneck link
    n16n17 = NodeContainer (c.Get (16), c.Get (17));

    // Receiver side connections
    n17n18 = NodeContainer (c.Get (17), c.Get (18));
    n17n19 = NodeContainer (c.Get (17), c.Get (19));
    n17n20 = NodeContainer (c.Get (17), c.Get (20));
    n17n21 = NodeContainer (c.Get (17), c.Get (21));
    n17n22 = NodeContainer (c.Get (17), c.Get (22));
    n17n23 = NodeContainer (c.Get (17), c.Get (23));
    n17n24 = NodeContainer (c.Get (17), c.Get (24));
    n17n25 = NodeContainer (c.Get (17), c.Get (25));
    n17n26 = NodeContainer (c.Get (17), c.Get (26));
    n17n27 = NodeContainer (c.Get (17), c.Get (27));
    n17n28 = NodeContainer (c.Get (17), c.Get (28));
    n17n29 = NodeContainer (c.Get (17), c.Get (29));
    n17n30 = NodeContainer (c.Get (17), c.Get (30));
    n17n31 = NodeContainer (c.Get (17), c.Get (31));
    n17n32 = NodeContainer (c.Get (17), c.Get (32));
    n17n33 = NodeContainer (c.Get (17), c.Get (33));

    uint32_t bufSize=0;

    InternetStackHelper internet;
    internet.Install (c);

    NS_LOG_INFO ("Create channels");
    PointToPointHelper p2p;
    TrafficControlHelper tch;

    //L0-L15 and L17-L32: Edge links
    bufSize =TOPO_SENDER_BW * TOPO_DEFAULT_QDELAY / 8000;
    p2p.SetQueue ("ns3::DropTailQueue",
                "Mode", StringValue ("QUEUE_MODE_BYTES"),
                "MaxBytes", UintegerValue (bufSize));
    p2p.SetDeviceAttribute ("DataRate", DataRateValue(DataRate (TOPO_SENDER_BW)));
    p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (TOPO_SENDER_PDELAY)));

    // Sender side edge links
    NetDeviceContainer devn0n16 = p2p.Install (n0n16);
    NetDeviceContainer devn1n16 = p2p.Install (n1n16);
    NetDeviceContainer devn2n16 = p2p.Install (n2n16);
    NetDeviceContainer devn3n16 = p2p.Install (n3n16);
    NetDeviceContainer devn4n16 = p2p.Install (n4n16);
    NetDeviceContainer devn5n16 = p2p.Install (n5n16);
    NetDeviceContainer devn6n16 = p2p.Install (n6n16);
    NetDeviceContainer devn7n16 = p2p.Install (n7n16);
    NetDeviceContainer devn8n16 = p2p.Install (n8n16);
    NetDeviceContainer devn9n16 = p2p.Install (n9n16);
    NetDeviceContainer devn10n16 = p2p.Install (n10n16);
    NetDeviceContainer devn11n16 = p2p.Install (n11n16);
    NetDeviceContainer devn12n16 = p2p.Install (n12n16);
    NetDeviceContainer devn13n16 = p2p.Install (n13n16);
    NetDeviceContainer devn14n16 = p2p.Install (n14n16);
    NetDeviceContainer devn15n16 = p2p.Install (n15n16);

    // Receiver side edge links
    NetDeviceContainer devn17n18 = p2p.Install (n17n18);
    NetDeviceContainer devn17n19 = p2p.Install (n17n19);
    NetDeviceContainer devn17n20 = p2p.Install (n17n20);
    NetDeviceContainer devn17n21 = p2p.Install (n17n21);
    NetDeviceContainer devn17n22 = p2p.Install (n17n22);
    NetDeviceContainer devn17n23 = p2p.Install (n17n23);
    NetDeviceContainer devn17n24 = p2p.Install (n17n24);
    NetDeviceContainer devn17n25 = p2p.Install (n17n25);
    NetDeviceContainer devn17n26 = p2p.Install (n17n26);
    NetDeviceContainer devn17n27 = p2p.Install (n17n27);
    NetDeviceContainer devn17n28 = p2p.Install (n17n28);
    NetDeviceContainer devn17n29 = p2p.Install (n17n29);
    NetDeviceContainer devn17n30 = p2p.Install (n17n30);
    NetDeviceContainer devn17n31 = p2p.Install (n17n31);
    NetDeviceContainer devn17n32 = p2p.Install (n17n32);
    NetDeviceContainer devn17n33 = p2p.Install (n17n33);

    //L16: Bottleneck link
    bufSize = TOPO_BOTTLE_BW * TOPO_DEFAULT_QDELAY / 8000;//与msQdelay相关，这里代表1个BDP
    p2p.SetQueue ("ns3::DropTailQueue",
                "Mode", StringValue ("QUEUE_MODE_BYTES"),
                "MaxBytes", UintegerValue (bufSize));
    p2p.SetDeviceAttribute ("DataRate", DataRateValue(DataRate (TOPO_BOTTLE_BW)));
    p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (TOPO_BOTTLE_PDELAY)));
    NetDeviceContainer devn16n17 = p2p.Install (n16n17);

    Ipv4AddressHelper ipv4;

    // Sender side IP addresses
    ipv4.SetBase ("10.1.1.0", "255.255.255.0");
    i0i16 = ipv4.Assign (devn0n16);
    tch.Uninstall (devn0n16);
    ipv4.SetBase ("10.1.2.0", "255.255.255.0");
    i1i16 = ipv4.Assign (devn1n16);
    tch.Uninstall (devn1n16);
    ipv4.SetBase ("10.1.3.0", "255.255.255.0");
    i2i16 = ipv4.Assign (devn2n16);
    tch.Uninstall (devn2n16);
    ipv4.SetBase ("10.1.4.0", "255.255.255.0");
    i3i16 = ipv4.Assign (devn3n16);
    tch.Uninstall (devn3n16);
    ipv4.SetBase ("10.1.5.0", "255.255.255.0");
    i4i16 = ipv4.Assign (devn4n16);
    tch.Uninstall (devn4n16);
    ipv4.SetBase ("10.1.6.0", "255.255.255.0");
    i5i16 = ipv4.Assign (devn5n16);
    tch.Uninstall (devn5n16);
    ipv4.SetBase ("10.1.7.0", "255.255.255.0");
    i6i16 = ipv4.Assign (devn6n16);
    tch.Uninstall (devn6n16);
    ipv4.SetBase ("10.1.8.0", "255.255.255.0");
    i7i16 = ipv4.Assign (devn7n16);
    tch.Uninstall (devn7n16);
    ipv4.SetBase ("10.1.9.0", "255.255.255.0");
    i8i16 = ipv4.Assign (devn8n16);
    tch.Uninstall (devn8n16);
    ipv4.SetBase ("10.1.10.0", "255.255.255.0");
    i9i16 = ipv4.Assign (devn9n16);
    tch.Uninstall (devn9n16);
    ipv4.SetBase ("10.1.11.0", "255.255.255.0");
    i10i16 = ipv4.Assign (devn10n16);
    tch.Uninstall (devn10n16);
    ipv4.SetBase ("10.1.12.0", "255.255.255.0");
    i11i16 = ipv4.Assign (devn11n16);
    tch.Uninstall (devn11n16);
    ipv4.SetBase ("10.1.13.0", "255.255.255.0");
    i12i16 = ipv4.Assign (devn12n16);
    tch.Uninstall (devn12n16);
    ipv4.SetBase ("10.1.14.0", "255.255.255.0");
    i13i16 = ipv4.Assign (devn13n16);
    tch.Uninstall (devn13n16);
    ipv4.SetBase ("10.1.15.0", "255.255.255.0");
    i14i16 = ipv4.Assign (devn14n16);
    tch.Uninstall (devn14n16);
    ipv4.SetBase ("10.1.16.0", "255.255.255.0");
    i15i16 = ipv4.Assign (devn15n16);
    tch.Uninstall (devn15n16);

    // Bottleneck link IP address
    ipv4.SetBase ("10.1.17.0", "255.255.255.0");
    i16i17 = ipv4.Assign (devn16n17);
    tch.Uninstall (devn16n17);

    // Receiver side IP addresses
    ipv4.SetBase ("10.1.18.0", "255.255.255.0");
    i17i18 = ipv4.Assign (devn17n18);
    tch.Uninstall (devn17n18);
    ipv4.SetBase ("10.1.19.0", "255.255.255.0");
    i17i19 = ipv4.Assign (devn17n19);
    tch.Uninstall (devn17n19);
    ipv4.SetBase ("10.1.20.0", "255.255.255.0");
    i17i20 = ipv4.Assign (devn17n20);
    tch.Uninstall (devn17n20);
    ipv4.SetBase ("10.1.21.0", "255.255.255.0");
    i17i21 = ipv4.Assign (devn17n21);
    tch.Uninstall (devn17n21);
    ipv4.SetBase ("10.1.22.0", "255.255.255.0");
    i17i22 = ipv4.Assign (devn17n22);
    tch.Uninstall (devn17n22);
    ipv4.SetBase ("10.1.23.0", "255.255.255.0");
    i17i23 = ipv4.Assign (devn17n23);
    tch.Uninstall (devn17n23);
    ipv4.SetBase ("10.1.24.0", "255.255.255.0");
    i17i24 = ipv4.Assign (devn17n24);
    tch.Uninstall (devn17n24);
    ipv4.SetBase ("10.1.25.0", "255.255.255.0");
    i17i25 = ipv4.Assign (devn17n25);
    tch.Uninstall (devn17n25);
    ipv4.SetBase ("10.1.26.0", "255.255.255.0");
    i17i26 = ipv4.Assign (devn17n26);
    tch.Uninstall (devn17n26);
    ipv4.SetBase ("10.1.27.0", "255.255.255.0");
    i17i27 = ipv4.Assign (devn17n27);
    tch.Uninstall (devn17n27);
    ipv4.SetBase ("10.1.28.0", "255.255.255.0");
    i17i28 = ipv4.Assign (devn17n28);
    tch.Uninstall (devn17n28);
    ipv4.SetBase ("10.1.29.0", "255.255.255.0");
    i17i29 = ipv4.Assign (devn17n29);
    tch.Uninstall (devn17n29);
    ipv4.SetBase ("10.1.30.0", "255.255.255.0");
    i17i30 = ipv4.Assign (devn17n30);
    tch.Uninstall (devn17n30);
    ipv4.SetBase ("10.1.31.0", "255.255.255.0");
    i17i31 = ipv4.Assign (devn17n31);
    tch.Uninstall (devn17n31);
    ipv4.SetBase ("10.1.32.0", "255.255.255.0");
    i17i32 = ipv4.Assign (devn17n32);
    tch.Uninstall (devn17n32);
    ipv4.SetBase ("10.1.33.0", "255.255.255.0");
    i17i33 = ipv4.Assign (devn17n33);
    tch.Uninstall (devn17n33);

    // Set up the routing
    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

    // Use original BBRv2 for all 16 flows
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

    float flow_start_times[NUM_FLOWS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Flow 1: n0 -> n18
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    stat->RegisterCongestionType(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender1 = InstallDqc(cc,c.Get(0),c.Get(18),sendPort,recvPort,flow_start_times[0]+0.001,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender1);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 2: n1 -> n19
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender2 = InstallDqc(cc,c.Get(1),c.Get(19),sendPort,recvPort,flow_start_times[1]+0.002,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender2);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 3: n2 -> n20
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender3 = InstallDqc(cc,c.Get(2),c.Get(20),sendPort,recvPort,flow_start_times[2]+0.003,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender3);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 4: n3 -> n21
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender4 = InstallDqc(cc,c.Get(3),c.Get(21),sendPort,recvPort,flow_start_times[3]+0.004,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender4);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 5: n4 -> n22
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender5 = InstallDqc(cc,c.Get(4),c.Get(22),sendPort,recvPort,flow_start_times[4]+0.005,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender5);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 6: n5 -> n23
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender6 = InstallDqc(cc,c.Get(5),c.Get(23),sendPort,recvPort,flow_start_times[5]+0.006,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender6);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 7: n6 -> n24
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender7 = InstallDqc(cc,c.Get(6),c.Get(24),sendPort,recvPort,flow_start_times[6]+0.007,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender7);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 8: n7 -> n25
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender8 = InstallDqc(cc,c.Get(7),c.Get(25),sendPort,recvPort,flow_start_times[7]+0.008,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender8);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 9: n8 -> n26
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender9 = InstallDqc(cc,c.Get(8),c.Get(26),sendPort,recvPort,flow_start_times[8]+0.009,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender9);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 10: n9 -> n27
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender10 = InstallDqc(cc,c.Get(9),c.Get(27),sendPort,recvPort,flow_start_times[9]+0.010,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender10);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 11: n10 -> n28
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender11 = InstallDqc(cc,c.Get(10),c.Get(28),sendPort,recvPort,flow_start_times[10]+0.011,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender11);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 12: n11 -> n29
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender12 = InstallDqc(cc,c.Get(11),c.Get(29),sendPort,recvPort,flow_start_times[11]+0.012,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender12);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 13: n12 -> n30
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender13 = InstallDqc(cc,c.Get(12),c.Get(30),sendPort,recvPort,flow_start_times[12]+0.013,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender13);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 14: n13 -> n31
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender14 = InstallDqc(cc,c.Get(13),c.Get(31),sendPort,recvPort,flow_start_times[13]+0.014,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender14);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 15: n14 -> n32
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender15 = InstallDqc(cc,c.Get(14),c.Get(32),sendPort,recvPort,flow_start_times[14]+0.015,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender15);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 16: n15 -> n33
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log=prefix+std::to_string(test_pair);
    trace->Log(log,DqcTraceEnable::E_DQC_GOODPUT|DqcTraceEnable::E_DQC_RTT|DqcTraceEnable::E_DQC_BW|DqcTraceEnable::E_DQC_OWD
|DqcTraceEnable::E_DQC_STAT|DqcTraceEnable::E_DQC_SEND_RATE|DqcTraceEnable::E_DQC_RECV_RATE|DqcTraceEnable::E_DQC_INFLIGHT|DqcTraceEnable::E_DQC_BBR_MODE|DqcTraceEnable::E_DQC_LOSS_RATE|DqcTraceEnable::E_DQC_ACK_EVENT|DqcTraceEnable::E_DQC_ACK_EPISODE);
    Ptr<DqcSender> sender16 = InstallDqc(cc,c.Get(15),c.Get(33),sendPort,recvPort,flow_start_times[15]+0.016,appStop,trace.get(),stat,max_bps,sender_id);
    senders.push_back(sender16);
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
    int sim_time=80;
    int ins[]={1};

    // Command line arguments
    CommandLine cmd;
    cmd.AddValue("sim_time", "Simulation time in seconds", sim_time);
    cmd.Parse(argc, argv);

    // Print configuration
    std::cout << "=== 16 BBRv2 Flows Configuration ===" << std::endl;
    std::cout << "Number of flows: " << NUM_FLOWS << std::endl;
    std::cout << "Congestion control: BBRv2" << std::endl;
    std::cout << "Bottleneck bandwidth: "<<TOPO_BOTTLE_BW/1000000<<" Mbps" << std::endl;
    std::cout << "Bottleneck delay: "<<TOPO_BOTTLE_PDELAY<<" ms" << std::endl;
    std::cout << "Simulation time: " << sim_time << " seconds" << std::endl;
    std::cout << "=====================================" << std::endl;

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
