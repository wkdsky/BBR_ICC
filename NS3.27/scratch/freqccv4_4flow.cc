/** Network topology
 *
 *    10Mb/s, 1ms                            10Mb/s, 1ms
 * n0----L0--------|                    |------L5-------n6
 *                 |   8Mbps/s, 28ms    |
 *                 n4--------L4--------n5
 *    10Mb/s, 1ms  |                    |    10Mb/s, 1ms
 * n1----L1--------|                    |------L6-------n7
 *                 |                    |
 *                 |                    |
 * n2----L2--------|                    |------L7-------n8
 *                 |                    |
 *                 |                    |
 * n3----L3--------|                    |------L8-------n9
 *
 * 4 FreqCCv4 flows, 8Mbps bottleneck, 60ms RTT, buffer = 1xBDP
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
#include <algorithm>
#include <cctype>
#include <fstream>
#include<iostream>
#include <map>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>
#include <memory>
#include <chrono>
#include "ns3/freqccv4_sender.h"
#include "ns3/send_packet_manager.h"
#include "queue_occupancy_trace_helper.h"
using namespace ns3;
using namespace dqc;
using namespace std;
#ifndef DQC_SCENARIO_LOG_COMPONENT
#define DQC_SCENARIO_LOG_COMPONENT "freqccv4-4flow"
#endif
#ifndef DQC_SCENARIO_INSTANCE
#define DQC_SCENARIO_INSTANCE "freqccv4_4flow"
#endif
#ifndef DQC_SCENARIO_TITLE
#define DQC_SCENARIO_TITLE "4 FreqCCv4 Flows"
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
std::string g_trace_output_dir;

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

const uint64_t TOPO_SENDER_BW       =   8 * 1000000;    // in bps
const uint64_t TOPO_SENDER_PDELAY   =   1;    // in ms
const uint64_t TOPO_BOTTLE_BW       =   20 * 1000000;     // in bps
const uint64_t TOPO_BOTTLE_PDELAY   =   18;    // in ms
const uint64_t TOPO_DEFAULT_QDELAY  =   (TOPO_SENDER_PDELAY*2+TOPO_BOTTLE_PDELAY)*2;    // in ms

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


// FreqCCv4 oscillation parameters for each flow
double g_freq_hz[NUM_FLOWS] = {5.0, 5.0, 5.0, 5.0};           // Oscillation frequency in Hz
std::string g_amp_mode[NUM_FLOWS] = {"fixed_mbps", "fixed_mbps", "fixed_mbps", "fixed_mbps"};  // Amplitude mode
double g_fixed_mbps[NUM_FLOWS] = {50.0, 50.0, 50.0, 50.0};        // Fixed amplitude in Mbps
double g_interval_window_rtt_mult = 1.0;                     // Interval STFT window = mult * min_rtt
bool g_dynamic_delay_enable = true;
bool g_enable_convergence_gate_trace = false;
bool g_enable_convergence_gate_control = false;
bool g_enable_cruise_window_trace = true;
std::string g_flow_start_mode = "same_start";
uint64_t g_flow_size_bytes = 15ULL * 1000ULL * 1000ULL;
int64_t g_process_interval_us = 100;
bool g_smoke_mode = false;
bool g_enable_heavy_trace = false;
bool g_gate_state_machine_self_test = false;
bool g_trusted_bw_selection_self_test = false;
bool g_trusted_bw_pacing_self_test = false;
bool g_use_engine_timer = true;
std::string g_gate_trace_mode = "round_only";
uint64_t g_gate_trace_sample_interval_us = 10000;
bool g_enable_equivalence_audit_trace = false;
std::string g_freq_bbr_config_path =
    "/home/wkd/BBR_ICC/NS3.27/examples/CCconfig/freqccv4_default.conf";
FreqBbrConfig g_freq_bbr_config;

std::string Trim(const std::string& in)
{
    size_t begin = 0;
    while(begin < in.size() &&
          std::isspace(static_cast<unsigned char>(in[begin]))){
        ++begin;
    }
    size_t end = in.size();
    while(end > begin &&
          std::isspace(static_cast<unsigned char>(in[end - 1]))){
        --end;
    }
    return in.substr(begin, end - begin);
}

bool ParseBoolValue(const std::string& value, bool* out)
{
    std::string lower;
    lower.reserve(value.size());
    for(char c : value){
        lower.push_back(static_cast<char>(
            std::tolower(static_cast<unsigned char>(c))));
    }
    if(lower == "true" || lower == "1" || lower == "yes" || lower == "on"){
        *out = true;
        return true;
    }
    if(lower == "false" || lower == "0" || lower == "no" || lower == "off"){
        *out = false;
        return true;
    }
    return false;
}

bool ParseDoubleValue(const std::string& value, double* out)
{
    try {
        size_t used = 0;
        const double parsed = std::stod(value, &used);
        if(Trim(value.substr(used)).empty()){
            *out = parsed;
            return true;
        }
    } catch(...) {
    }
    return false;
}

bool ParseUintValue(const std::string& value, uint32_t* out)
{
    try {
        size_t used = 0;
        const unsigned long parsed = std::stoul(value, &used);
        if(Trim(value.substr(used)).empty()){
            *out = static_cast<uint32_t>(parsed);
            return true;
        }
    } catch(...) {
    }
    return false;
}

bool ParseUint64Value(const std::string& value, uint64_t* out)
{
    try {
        size_t used = 0;
        const unsigned long long parsed = std::stoull(value, &used);
        if(Trim(value.substr(used)).empty()){
            *out = static_cast<uint64_t>(parsed);
            return true;
        }
    } catch(...) {
    }
    return false;
}

void WarnConfigLine(const std::string& path,
                    uint32_t line_no,
                    const std::string& message)
{
    std::cerr << "[freqBbrConfig warning] " << path << ":" << line_no
              << ": " << message << std::endl;
}

bool SetFreqBbrConfigValue(FreqBbrConfig* config,
                           const std::string& key,
                           const std::string& value,
                           const std::string& path,
                           uint32_t line_no)
{
    double d = 0.0;
    uint32_t u32 = 0;
    uint64_t u64 = 0;
    bool b = false;

    if(key.rfind("flow.", 0) == 0){
        const size_t id_begin = 5;
        const size_t id_end = key.find('.', id_begin);
        if(id_end == std::string::npos){
            WarnConfigLine(path, line_no, "invalid flow key: " + key);
            return false;
        }
        uint32_t flow_id = 0;
        if(!ParseUintValue(key.substr(id_begin, id_end - id_begin), &flow_id)){
            WarnConfigLine(path, line_no, "invalid flow id in key: " + key);
            return false;
        }
        const std::string field = key.substr(id_end + 1);
        FreqBbrFlowConfig& flow = config->flow[flow_id];
        if(field == "modulation_freq_hz"){
            if(!ParseDoubleValue(value, &d)){
                WarnConfigLine(path, line_no, "invalid double for " + key);
                return false;
            }
            flow.modulation_freq_hz = d;
            flow.has_modulation_freq_hz = true;
            return true;
        }
        if(field == "fixed_amplitude_mbps"){
            if(!ParseDoubleValue(value, &d)){
                WarnConfigLine(path, line_no, "invalid double for " + key);
                return false;
            }
            flow.fixed_amplitude_mbps = d;
            flow.has_fixed_amplitude_mbps = true;
            return true;
        }
        WarnConfigLine(path, line_no, "unknown flow field: " + key);
        return false;
    }

#define SET_DOUBLE(KEY, FIELD) \
    if(key == KEY){ \
        if(!ParseDoubleValue(value, &d)){ \
            WarnConfigLine(path, line_no, "invalid double for " + key); \
            return false; \
        } \
        config->FIELD = d; \
        return true; \
    }
#define SET_U32(KEY, FIELD) \
    if(key == KEY){ \
        if(!ParseUintValue(value, &u32)){ \
            WarnConfigLine(path, line_no, "invalid uint for " + key); \
            return false; \
        } \
        config->FIELD = u32; \
        return true; \
    }
#define SET_U64(KEY, FIELD) \
    if(key == KEY){ \
        if(!ParseUint64Value(value, &u64)){ \
            WarnConfigLine(path, line_no, "invalid uint64 for " + key); \
            return false; \
        } \
        config->FIELD = u64; \
        return true; \
    }
#define SET_BOOL(KEY, FIELD) \
    if(key == KEY){ \
        if(!ParseBoolValue(value, &b)){ \
            WarnConfigLine(path, line_no, "invalid bool for " + key); \
            return false; \
        } \
        config->FIELD = b; \
        return true; \
    }

    SET_DOUBLE("default_modulation_freq_hz", default_modulation_freq_hz)
    if(key == "default_amplitude_mode"){
        config->default_amplitude_mode = value;
        return true;
    }
    SET_DOUBLE("default_fixed_amplitude_mbps", default_fixed_amplitude_mbps)
    SET_DOUBLE("stability.single_round_exit_threshold", stability_single_round_exit_threshold)
    SET_DOUBLE("stability.consecutive_exit_threshold", stability_consecutive_exit_threshold)
    SET_U32("stability.stable_rounds", stability_stable_rounds)
    SET_DOUBLE("stability.full_pipe_growth_threshold", stability_full_pipe_growth_threshold)
    SET_DOUBLE("spectral.drate_integrity_threshold", spectral_drate_integrity_threshold)
    SET_DOUBLE("spectral.srtt_integrity_threshold", spectral_srtt_integrity_threshold)
    SET_DOUBLE("spectral.min_drate_snr", spectral_min_drate_snr)
    SET_DOUBLE("spectral.min_srtt_snr", spectral_min_srtt_snr)
    SET_DOUBLE("spectral.max_drate_width_ratio", spectral_max_drate_width_ratio)
    SET_DOUBLE("spectral.max_srtt_width_ratio", spectral_max_srtt_width_ratio)
    SET_DOUBLE("spectral.min_drate_phase_coherence", spectral_min_drate_phase_coherence)
    SET_DOUBLE("spectral.min_srtt_phase_coherence", spectral_min_srtt_phase_coherence)
    SET_DOUBLE("spectral.freq_sigma_ratio", spectral_freq_sigma_ratio)
    SET_DOUBLE("spectral.snr_slope", spectral_snr_slope)
    SET_DOUBLE("spectral.energy_threshold", spectral_energy_threshold)
    SET_DOUBLE("spectral.energy_slope", spectral_energy_slope)
    SET_DOUBLE("spectral.width_r0_drate", spectral_width_r0_drate)
    SET_DOUBLE("spectral.width_r0_srtt", spectral_width_r0_srtt)
    SET_DOUBLE("spectral.width_sigma", spectral_width_sigma)
    SET_BOOL("merged_rescue.enable", merged_rescue_enable)
    SET_DOUBLE("merged_rescue.window_multiplier", merged_rescue_window_multiplier)
    SET_U32("merged_rescue.max_passes", merged_rescue_max_passes)
    SET_DOUBLE("merged_rescue.max_trend_ratio", merged_rescue_max_trend_ratio)
    SET_DOUBLE("merged_rescue.confidence_discount", merged_rescue_confidence_discount)
    SET_BOOL("trusted_bw.clear_on_cruise_start", trusted_bw_clear_on_cruise_start)
    if(key == "trace.gate_trace_mode"){
        config->trace_gate_trace_mode = value;
        return true;
    }
    SET_U64("trace.gate_trace_sample_interval_us", trace_gate_trace_sample_interval_us)
    SET_BOOL("trace.enable_cruise_window_trace", trace_enable_cruise_window_trace)
    SET_BOOL("trace.enable_trusted_bw_selection_trace", trace_enable_trusted_bw_selection_trace)

#undef SET_DOUBLE
#undef SET_U32
#undef SET_U64
#undef SET_BOOL

    WarnConfigLine(path, line_no, "unknown key: " + key);
    return false;
}

bool LoadFreqBbrConfig(const std::string& path, FreqBbrConfig* config)
{
    std::ifstream in(path.c_str());
    if(!in.is_open()){
        std::cerr << "[freqBbrConfig warning] unable to open config: "
                  << path << "; using built-in defaults" << std::endl;
        return false;
    }
    std::string line;
    uint32_t line_no = 0;
    while(std::getline(in, line)){
        ++line_no;
        const size_t comment = line.find('#');
        if(comment != std::string::npos){
            line = line.substr(0, comment);
        }
        line = Trim(line);
        if(line.empty()){
            continue;
        }
        const size_t eq = line.find('=');
        if(eq == std::string::npos){
            WarnConfigLine(path, line_no, "expected key=value");
            continue;
        }
        const std::string key = Trim(line.substr(0, eq));
        const std::string value = Trim(line.substr(eq + 1));
        if(key.empty()){
            WarnConfigLine(path, line_no, "empty key");
            continue;
        }
        SetFreqBbrConfigValue(config, key, value, path, line_no);
    }
    std::cout << "[freqBbrConfig] loaded " << path << std::endl;
    return true;
}

void ApplyFreqBbrConfigToGlobals(const FreqBbrConfig& config)
{
    for(uint32_t i = 0; i < NUM_FLOWS; ++i){
        g_freq_hz[i] = config.default_modulation_freq_hz;
        g_fixed_mbps[i] = config.default_fixed_amplitude_mbps;
        auto it = config.flow.find(i);
        if(it != config.flow.end()){
            if(it->second.has_modulation_freq_hz){
                g_freq_hz[i] = it->second.modulation_freq_hz;
            }
            if(it->second.has_fixed_amplitude_mbps){
                g_fixed_mbps[i] = it->second.fixed_amplitude_mbps;
            }
        }
        g_amp_mode[i] = config.default_amplitude_mode;
    }
    g_gate_trace_mode = config.trace_gate_trace_mode;
    g_gate_trace_sample_interval_us =
        config.trace_gate_trace_sample_interval_us;
    g_enable_cruise_window_trace =
        config.trace_enable_cruise_window_trace;
    g_enable_convergence_gate_trace =
        config.trace_enable_trusted_bw_selection_trace;
}

void ApplyGlobalsToFreqBbrConfig(FreqBbrConfig* config)
{
    config->default_amplitude_mode = g_amp_mode[0];
    config->default_modulation_freq_hz = g_freq_hz[0];
    config->default_fixed_amplitude_mbps = g_fixed_mbps[0];
    for(uint32_t i = 0; i < NUM_FLOWS; ++i){
        FreqBbrFlowConfig& flow = config->flow[i];
        flow.modulation_freq_hz = g_freq_hz[i];
        flow.has_modulation_freq_hz = true;
        flow.fixed_amplitude_mbps = g_fixed_mbps[i];
        flow.has_fixed_amplitude_mbps = true;
    }
    config->trace_gate_trace_mode = g_gate_trace_mode;
    config->trace_gate_trace_sample_interval_us =
        g_gate_trace_sample_interval_us;
    config->trace_enable_cruise_window_trace =
        g_enable_cruise_window_trace;
    config->trace_enable_trusted_bw_selection_trace =
        g_enable_convergence_gate_trace;
}

std::string PreScanFreqBbrConfigPath(int argc, char* argv[],
                                     const std::string& default_path)
{
    const std::string prefix = "--freqBbrConfig=";
    for(int i = 1; i < argc; ++i){
        const std::string arg = argv[i];
        if(arg.rfind(prefix, 0) == 0){
            return arg.substr(prefix.size());
        }
        if(arg == "--freqBbrConfig" && i + 1 < argc){
            return argv[i + 1];
        }
    }
    return default_path;
}

Ptr<PointToPointChannel>
GetPointToPointChannel(const NetDeviceContainer& devices)
{
    Ptr<PointToPointNetDevice> device = DynamicCast<PointToPointNetDevice>(devices.Get(0));
    NS_ABORT_MSG_IF(device == nullptr, "Expected PointToPointNetDevice");
    Ptr<PointToPointChannel> channel = DynamicCast<PointToPointChannel>(device->GetChannel());
    NS_ABORT_MSG_IF(channel == nullptr, "Expected PointToPointChannel");
    return channel;
}

void
ApplyPropagationDelayStep(const std::vector<Ptr<PointToPointChannel>>& edge_channels,
                          const Ptr<PointToPointChannel>& bottleneck_channel,
                          double edge_delay_ms,
                          double bottleneck_delay_ms,
                          uint32_t step_index)
{
    for (const Ptr<PointToPointChannel>& channel : edge_channels)
    {
        channel->SetPropagationDelay(MilliSeconds(edge_delay_ms));
    }
    bottleneck_channel->SetPropagationDelay(MilliSeconds(bottleneck_delay_ms));

    std::cout << "[delay-step] t=" << Simulator::Now().GetSeconds()
              << "s step=" << step_index
              << " edge_delay_ms=" << edge_delay_ms
              << " bottleneck_delay_ms=" << bottleneck_delay_ms
              << " path_oneway_ms=" << (2.0 * edge_delay_ms + bottleneck_delay_ms)
              << " path_rtt_ms=" << (2.0 * (2.0 * edge_delay_ms + bottleneck_delay_ms))
              << std::endl;
}

void
ConfigurePropagationDelaySchedule(const std::vector<Ptr<PointToPointChannel>>& edge_channels,
                                  const Ptr<PointToPointChannel>& bottleneck_channel)
{
    if (!g_dynamic_delay_enable)
    {
        return;
    }

    const std::vector<double> change_times_s = {4.0, 7.0, 12.0, 18.0, 22.0, 27.0};
    const std::vector<double> edge_delays_ms = {1.0, 5.0, 1.0, 7.0, 2.0, 6.0};
    const std::vector<double> bottleneck_delays_ms = {8.0, 55.0, 12.0, 45.0, 6.0, 60.0};

    for (size_t i = 0; i < change_times_s.size(); ++i)
    {
        Simulator::Schedule(Seconds(change_times_s[i]),
                            &ApplyPropagationDelayStep,
                            edge_channels,
                            bottleneck_channel,
                            edge_delays_ms[i],
                            bottleneck_delays_ms[i],
                            static_cast<uint32_t>(i + 1));
    }
}

static Ptr<DqcSender> InstallDqc( dqc::CongestionControlType cc_type,
                        Ptr<Node> sender,Ptr<Node> receiver,
                        uint16_t send_port,uint16_t recv_port,
                        double startTime,double stopTime,
                        DqcTrace *trace, DqcTraceState *stat,
                        double freq_hz, const std::string& amp_mode, double fixed_mbps,
                        uint32_t max_bps=0,uint32_t cid=0,bool ecn=false,uint32_t emucons=1,
	                        uint64_t fair_share_bps=0,
	                        uint64_t flow_size_bytes=0,
	                        uint32_t trace_enable=0,
	                        uint32_t flow_index=0)
{
    Ptr<DqcSender> sendApp = CreateObject<DqcSender> (cc_type,ecn,g_use_engine_timer);
    Ptr<DqcReceiver> recvApp = CreateObject<DqcReceiver>(100);  // 100ms goodput统计间隔，更实时
    sender->AddApplication (sendApp);
    receiver->AddApplication (recvApp);
    sendApp->SetProcessIntervalUs(g_process_interval_us);
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
    if(flow_size_bytes > 0){
        sendApp->SetPacketLimitBytes(flow_size_bytes);
    }
    if(max_bps>0){
        sendApp->SetMaxBandwidth(max_bps);
    }
    if(cid){
       sendApp->SetSenderId(cid);
        sendApp->SetCongestionId(cid);
    }
    if(trace){
        if(trace_enable & DqcTraceEnable::E_DQC_BW){
        sendApp->SetBwTraceFuc(MakeCallback(&DqcTrace::OnBw,trace));
        }
        if(trace_enable & DqcTraceEnable::E_DQC_RTT){
        sendApp->SetRttTraceFuc(MakeCallback(&DqcTrace::OnRtt,trace));
        }
        if(trace_enable & DqcTraceEnable::E_DQC_QUEUE_DELAY){
        sendApp->SetQueueDelayTraceFuc(MakeCallback(&DqcTrace::OnQueueDelay,trace));
        }
        if(trace_enable & DqcTraceEnable::E_DQC_SEND_RATE){
        sendApp->SetSendRateTraceFuc(MakeCallback(&DqcTrace::OnSendRate,trace));
        }
        if(trace_enable & DqcTraceEnable::E_DQC_RECV_RATE){
        sendApp->SetRecvRateTraceFuc(MakeCallback(&DqcTrace::OnRecvRate,trace));
        }
        if(trace_enable & DqcTraceEnable::E_DQC_RECV_RATE_RAW){
        sendApp->SetRecvRateRawTraceFuc(MakeCallback(&DqcTrace::OnRecvRateRaw,trace));
        }
        if(trace_enable & DqcTraceEnable::E_DQC_INFLIGHT){
        sendApp->SetInflightTraceFuc(MakeCallback(&DqcTrace::OnInflight,trace));
        }
        if(trace_enable & DqcTraceEnable::E_DQC_BBR_MODE){
        sendApp->SetBbrModeTraceFuc(MakeCallback(&DqcTrace::OnBbrMode,trace));
        }
        if(trace_enable & DqcTraceEnable::E_DQC_UP_PHASE){
        sendApp->SetUpPhaseTraceFuc(MakeCallback(&DqcTrace::OnUpPhase,trace));
        }
        if(trace_enable & DqcTraceEnable::E_DQC_FREQ_ANALYSIS){
        sendApp->SetFreqAnalysisTraceFuc(MakeCallback(&DqcTrace::OnFreqAnalysis,trace));
        sendApp->SetRttFreqAnalysisTraceFuc(MakeCallback(&DqcTrace::OnRttFreqAnalysis,trace));
        }
        if(trace_enable & (DqcTraceEnable::E_DQC_FREQCCV4_LOAD |
                           DqcTraceEnable::E_DQC_FREQCCV4_GATE)){
        sendApp->SetFreqCCv4LoadTraceFuc(MakeCallback(&DqcTrace::OnFreqCCv4Load,trace));
        }
        if(trace_enable & DqcTraceEnable::E_DQC_GOODPUT){
        recvApp->SetGoodputTraceFuc(MakeCallback(&DqcTrace::OnGoodput,trace));
        }
        if(trace_enable & DqcTraceEnable::E_DQC_LOSS_RATE){
        sendApp->SetLossRateTraceFuc(MakeCallback(&DqcTrace::OnLossRate,trace));
        }
    }
	    // Configure FreqCCv4 oscillation parameters
	    if(cc_type == kFreqCCv4){
	        sendApp->ConfigureFreqBbr(g_freq_bbr_config, flow_index);
	        sendApp->ConfigureFreqCC(freq_hz, amp_mode, fixed_mbps);
	        sendApp->ConfigureFreqCCv4ConvergenceGate(g_enable_convergence_gate_trace,
	                                                  g_enable_convergence_gate_control,
	                                                  g_gate_trace_mode,
	                                                  g_gate_trace_sample_interval_us);
        sendApp->SetFreqCCIntervalWindowMultiplier(g_interval_window_rtt_mult);
        sendApp->SetFreqCCFairShareBandwidth(fair_share_bps);
    }
    return sendApp;
}

void ns3_freqccv4(int ins, std::string algo, DqcTraceState *stat, double sim_time=60.0, int loss_integer=0){
    std::string instance=DQC_SCENARIO_INSTANCE;
    uint64_t linkBw   = TOPO_BOTTLE_BW;
    uint16_t sendPort=1000;
    uint16_t recvPort=5000;

    double sim_dur=sim_time;
    double end_time=sim_time;
    double appStop=end_time;

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

    std::vector<Ptr<PointToPointChannel>> edge_channels = {
        GetPointToPointChannel(devn0n4),
        GetPointToPointChannel(devn1n4),
        GetPointToPointChannel(devn2n4),
        GetPointToPointChannel(devn3n4),
        GetPointToPointChannel(devn5n6),
        GetPointToPointChannel(devn5n7),
        GetPointToPointChannel(devn5n8),
        GetPointToPointChannel(devn5n9)};
    ConfigurePropagationDelaySchedule(edge_channels, GetPointToPointChannel(devn4n5));

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

    dqc::CongestionControlType cc = kFreqCCv4;
    if(algo == "bbrv2" || algo == "BBRv2" || algo == "bbr2"){
        cc = kBBRv2;
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
	    uint32_t trace_enable =
	        DqcTraceEnable::E_DQC_GOODPUT |
	        DqcTraceEnable::E_DQC_BBR_MODE |
	        DqcTraceEnable::E_DQC_LOSS_RATE;
	    if(g_enable_cruise_window_trace){
	        trace_enable |= DqcTraceEnable::E_DQC_FREQCCV4_LOAD;
	    }
    if(g_enable_heavy_trace){
        trace_enable |=
            DqcTraceEnable::E_DQC_BW |
            DqcTraceEnable::E_DQC_SEND_RATE |
            DqcTraceEnable::E_DQC_RECV_RATE |
            DqcTraceEnable::E_DQC_RECV_RATE_RAW |
            DqcTraceEnable::E_DQC_QUEUE_DELAY |
            DqcTraceEnable::E_DQC_INFLIGHT;
    }
    if(g_enable_convergence_gate_trace){
        trace_enable |= DqcTraceEnable::E_DQC_FREQCCV4_GATE;
    }

    float flow_start_times[NUM_FLOWS] = {0.0, 0.0, 0.0, 0.0};
    if(g_flow_start_mode == "staggered_start"){
        flow_start_times[0] = 0.000;
        flow_start_times[1] = 0.020;
        flow_start_times[2] = 0.040;
        flow_start_times[3] = 0.060;
    }
    const uint64_t fair_share_bps = TOPO_BOTTLE_BW / NUM_FLOWS;

    // Flow 1: n0 -> n6
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    stat->RegisterCongestionType(test_pair);
    log="flow"+std::to_string(test_pair);
    trace->Log(log, trace_enable);
	    Ptr<DqcSender> sender1 = InstallDqc(cc,c.Get(0),c.Get(6),sendPort,recvPort,flow_start_times[0],appStop,trace.get(),stat,g_freq_hz[0],g_amp_mode[0],g_fixed_mbps[0],max_bps,sender_id,false,1,fair_share_bps,g_flow_size_bytes,trace_enable,0);
    if(g_enable_equivalence_audit_trace && !g_trace_output_dir.empty()){
        sender1->SetEquivalenceAuditTracePrefix(g_trace_output_dir+"flow1");
    }
    senders.push_back(sender1);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 2: n1 -> n7
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log="flow"+std::to_string(test_pair);
    trace->Log(log, trace_enable);
	    Ptr<DqcSender> sender2 = InstallDqc(cc,c.Get(1),c.Get(7),sendPort,recvPort,flow_start_times[1],appStop,trace.get(),stat,g_freq_hz[1],g_amp_mode[1],g_fixed_mbps[1],max_bps,sender_id,false,1,fair_share_bps,g_flow_size_bytes,trace_enable,1);
    if(g_enable_equivalence_audit_trace && !g_trace_output_dir.empty()){
        sender2->SetEquivalenceAuditTracePrefix(g_trace_output_dir+"flow2");
    }
    senders.push_back(sender2);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 3: n2 -> n8
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log="flow"+std::to_string(test_pair);
    trace->Log(log, trace_enable);
	    Ptr<DqcSender> sender3 = InstallDqc(cc,c.Get(2),c.Get(8),sendPort,recvPort,flow_start_times[2],appStop,trace.get(),stat,g_freq_hz[2],g_amp_mode[2],g_fixed_mbps[2],max_bps,sender_id,false,1,fair_share_bps,g_flow_size_bytes,trace_enable,2);
    if(g_enable_equivalence_audit_trace && !g_trace_output_dir.empty()){
        sender3->SetEquivalenceAuditTracePrefix(g_trace_output_dir+"flow3");
    }
    senders.push_back(sender3);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    // Flow 4: n3 -> n9
    trace.reset(new DqcTrace(test_pair));
    stat->ReisterAvgDelayId(test_pair);
    log="flow"+std::to_string(test_pair);
    trace->Log(log, trace_enable);
	    Ptr<DqcSender> sender4 = InstallDqc(cc,c.Get(3),c.Get(9),sendPort,recvPort,flow_start_times[3],appStop,trace.get(),stat,g_freq_hz[3],g_amp_mode[3],g_fixed_mbps[3],max_bps,sender_id,false,1,fair_share_bps,g_flow_size_bytes,trace_enable,3);
    if(g_enable_equivalence_audit_trace && !g_trace_output_dir.empty()){
        sender4->SetEquivalenceAuditTracePrefix(g_trace_output_dir+"flow4");
    }
    senders.push_back(sender4);
    sender_id++;
    test_pair++;
    sendPort++;
    recvPort++;
    traces.push_back(std::move(trace));

    Simulator::Stop (Seconds(sim_dur));
    auto run_wall_start = std::chrono::high_resolution_clock::now();
    Simulator::Run ();
    auto run_wall_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> run_wall = run_wall_end - run_wall_start;
    std::cout << "[runtime-diagnosis] sim_stop_s=" << sim_dur
              << " simulator_now_s=" << Simulator::Now().GetSeconds()
              << " event_count=unavailable_in_this_ns3_tree"
              << " wall_seconds=" << run_wall.count()
              << " process_interval_us=" << g_process_interval_us
              << " gate_trace_mode=" << g_gate_trace_mode
              << " heavy_trace=" << g_enable_heavy_trace
              << std::endl;
    Simulator::Destroy();
    stat->Flush(linkBw,sim_dur);
}

int main (int argc, char *argv[]){
    double sim_time=30.0;
    int ins[]={1};
    std::string trace_path="";
    std::string output_dir="";
    std::string algo="freqccv4";
	    uint32_t run_id=1;
	    uint32_t seed=1;
	    g_freq_bbr_config_path =
	        PreScanFreqBbrConfigPath(argc, argv, g_freq_bbr_config_path);
	    LoadFreqBbrConfig(g_freq_bbr_config_path, &g_freq_bbr_config);
	    ApplyFreqBbrConfigToGlobals(g_freq_bbr_config);

	    // Command line arguments
	    CommandLine cmd;
	    cmd.AddValue("freqBbrConfig", "FreqBBR key=value config file path", g_freq_bbr_config_path);
	    cmd.AddValue("sim_time", "Simulation time in seconds", sim_time);
    cmd.AddValue("trace_path", "Output trace directory path", trace_path);
    cmd.AddValue("outputDir", "Output trace directory path", output_dir);
    cmd.AddValue("algo", "Congestion control: freqccv4 or bbrv2", algo);
    cmd.AddValue("runId", "ns-3 RNG run id", run_id);
    cmd.AddValue("seed", "ns-3 RNG seed", seed);
	    cmd.AddValue("enableConvergenceGateTrace", "Enable FreqCCv4 convergence-gate CSV trace", g_enable_convergence_gate_trace);
	    cmd.AddValue("enableConvergenceGateControl", "Gate CRUISE modulation by BBR stability", g_enable_convergence_gate_control);
	    cmd.AddValue("enableCruiseWindowTrace", "Enable FreqCCv4 CRUISE window trace", g_enable_cruise_window_trace);
    cmd.AddValue("flowStartMode", "Flow start mode: same_start or staggered_start", g_flow_start_mode);
    cmd.AddValue("flowSizeBytes", "Per-flow send limit in bytes; 0 keeps unlimited/default behavior", g_flow_size_bytes);
    cmd.AddValue("processIntervalUs", "DqcSender process timer interval in microseconds", g_process_interval_us);
    cmd.AddValue("smokeMode", "Use tiny, fast packet-level smoke defaults", g_smoke_mode);
    cmd.AddValue("enableHeavyTrace", "Enable per-packet/per-pacing heavy trace callbacks", g_enable_heavy_trace);
    cmd.AddValue("gateTraceMode", "FreqCCv4 gate trace mode: off, round_only, sampled_pacing, full", g_gate_trace_mode);
	    cmd.AddValue("gateTraceSampleIntervalUs", "Minimum interval for sampled_pacing gate trace rows", g_gate_trace_sample_interval_us);
	    cmd.AddValue("gateStateMachineSelfTest", "Run synthetic convergence-gate state-machine self-test and exit", g_gate_state_machine_self_test);
	    cmd.AddValue("trustedBwSelectionSelfTest", "Run TrustedBw dual-signal selection self-test and exit", g_trusted_bw_selection_self_test);
	    cmd.AddValue("trustedBwPacingSelfTest", "Run TrustedBw pacing-baseline self-test and exit", g_trusted_bw_pacing_self_test);
	    cmd.AddValue("useEngineTimer", "Use DQC engine alarm timer; false uses processIntervalUs polling", g_use_engine_timer);
	    cmd.AddValue("enableEquivalenceAuditTrace", "Enable packet/ACK/pacing audit traces for B/C equivalence checks", g_enable_equivalence_audit_trace);
	    cmd.AddValue("stabilitySingleRoundExitThreshold", "TrustedBw selection MaxDRate single-round exit threshold", g_freq_bbr_config.stability_single_round_exit_threshold);
	    cmd.AddValue("stabilityConsecutiveExitThreshold", "TrustedBw selection MaxDRate consecutive exit threshold", g_freq_bbr_config.stability_consecutive_exit_threshold);
	    cmd.AddValue("stabilityStableRounds", "TrustedBw selection stable rounds", g_freq_bbr_config.stability_stable_rounds);
	    cmd.AddValue("stabilityFullPipeGrowthThreshold", "TrustedBw selection full-pipe growth threshold", g_freq_bbr_config.stability_full_pipe_growth_threshold);
	    cmd.AddValue("spectralDrateIntegrityThreshold", "Delivery Rate spectral integrity threshold", g_freq_bbr_config.spectral_drate_integrity_threshold);
	    cmd.AddValue("spectralSrttIntegrityThreshold", "SRTT spectral integrity threshold", g_freq_bbr_config.spectral_srtt_integrity_threshold);
	    cmd.AddValue("spectralMinDrateSnr", "Spectral Validity Gate V2 minimum DRate SNR", g_freq_bbr_config.spectral_min_drate_snr);
	    cmd.AddValue("spectralMinSrttSnr", "Spectral Validity Gate V2 minimum SRTT SNR", g_freq_bbr_config.spectral_min_srtt_snr);
	    cmd.AddValue("spectralMaxDrateWidthRatio", "Spectral Validity Gate V2 maximum DRate width ratio", g_freq_bbr_config.spectral_max_drate_width_ratio);
	    cmd.AddValue("spectralMaxSrttWidthRatio", "Spectral Validity Gate V2 maximum SRTT width ratio", g_freq_bbr_config.spectral_max_srtt_width_ratio);
	    cmd.AddValue("spectralMinDratePhaseCoherence", "Spectral Validity Gate V2 minimum DRate phase coherence", g_freq_bbr_config.spectral_min_drate_phase_coherence);
	    cmd.AddValue("spectralMinSrttPhaseCoherence", "Spectral Validity Gate V2 minimum SRTT phase coherence", g_freq_bbr_config.spectral_min_srtt_phase_coherence);
	    cmd.AddValue("mergedRescueEnable", "Enable merged-window rescue", g_freq_bbr_config.merged_rescue_enable);
	    cmd.AddValue("mergedRescueWindowMultiplier", "Merged-window rescue duration multiplier", g_freq_bbr_config.merged_rescue_window_multiplier);
	    cmd.AddValue("mergedRescueMaxPasses", "Merged-window rescue max passes", g_freq_bbr_config.merged_rescue_max_passes);
	    cmd.AddValue("mergedRescueMaxTrendRatio", "Merged-window rescue max trend ratio", g_freq_bbr_config.merged_rescue_max_trend_ratio);
	    // Per-flow parameters
    cmd.AddValue("freq1", "Flow 1 oscillation frequency (Hz)", g_freq_hz[0]);
    cmd.AddValue("freq2", "Flow 2 oscillation frequency (Hz)", g_freq_hz[1]);
    cmd.AddValue("freq3", "Flow 3 oscillation frequency (Hz)", g_freq_hz[2]);
    cmd.AddValue("freq4", "Flow 4 oscillation frequency (Hz)", g_freq_hz[3]);
    cmd.AddValue("amp1", "Flow 1 amplitude mode", g_amp_mode[0]);
    cmd.AddValue("amp2", "Flow 2 amplitude mode", g_amp_mode[1]);
    cmd.AddValue("amp3", "Flow 3 amplitude mode", g_amp_mode[2]);
    cmd.AddValue("amp4", "Flow 4 amplitude mode", g_amp_mode[3]);
    cmd.AddValue("fixed1", "Flow 1 fixed amplitude (Mbps)", g_fixed_mbps[0]);
    cmd.AddValue("fixed2", "Flow 2 fixed amplitude (Mbps)", g_fixed_mbps[1]);
    cmd.AddValue("fixed3", "Flow 3 fixed amplitude (Mbps)", g_fixed_mbps[2]);
    cmd.AddValue("fixed4", "Flow 4 fixed amplitude (Mbps)", g_fixed_mbps[3]);
    cmd.AddValue("interval_win_rtt_mult", "Interval-phase STFT window multiplier on min_rtt", g_interval_window_rtt_mult);
	    cmd.AddValue("dynamic_delay_enable", "Enable scheduled propagation delay changes at 4,7,12,18,22,27s", g_dynamic_delay_enable);
	    cmd.Parse(argc, argv);
	    ApplyGlobalsToFreqBbrConfig(&g_freq_bbr_config);
	    if(g_gate_state_machine_self_test){
	        return FreqCCv4Sender::RunConvergenceGateStateMachineSelfTest(std::cout) ? 0 : 1;
	    }
	    if(g_trusted_bw_selection_self_test){
	        return FreqCCv4Sender::RunTrustedBwSelectionSelfTest(std::cout) ? 0 : 1;
	    }
	    if(g_trusted_bw_pacing_self_test){
	        return FreqCCv4Sender::RunTrustedBwPacingSelfTest(std::cout) ? 0 : 1;
	    }
    if(g_smoke_mode){
        sim_time = std::min(sim_time, 0.5);
        if(g_flow_size_bytes == 0 || g_flow_size_bytes > 20000ULL){
            g_flow_size_bytes = 20000ULL;
        }
        g_process_interval_us = std::max<int64_t>(g_process_interval_us, 20000);
        g_dynamic_delay_enable = false;
        g_enable_heavy_trace = false;
        g_use_engine_timer = false;
        if(g_gate_trace_mode == "full" || g_gate_trace_mode == "sampled_pacing"){
            g_gate_trace_mode = "round_only";
        }
    }
    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(run_id);
    SendPacketManager::SetDeterministicRandomSeed(seed, run_id);
    if(!output_dir.empty()){
        trace_path = output_dir;
    }
    if(!trace_path.empty()){
        if(trace_path.back() != '/'){
            trace_path.push_back('/');
        }
        EnsureDirectoryExists(trace_path);
        set_dqc_trace_folder(trace_path);
    }
    g_trace_output_dir = trace_path;
    SetQueueOccupancyTraceFolder(trace_path);

    // Print configuration
    std::cout << "=== " << DQC_SCENARIO_TITLE << " Configuration ===" << std::endl;
    std::cout << "Number of flows: " << NUM_FLOWS << std::endl;
    std::cout << "Congestion control: " << algo << std::endl;
	    std::cout << "Run id / seed: " << run_id << " / " << seed << std::endl;
	    std::cout << "FreqBBR config: " << g_freq_bbr_config_path << std::endl;
	    std::cout << "Flow start mode: " << g_flow_start_mode << std::endl;
    std::cout << "Flow size bytes: " << g_flow_size_bytes << std::endl;
    std::cout << "Process interval us: " << g_process_interval_us << std::endl;
    std::cout << "Use engine timer: " << g_use_engine_timer << std::endl;
    std::cout << "Smoke mode: " << g_smoke_mode << std::endl;
	    std::cout << "Heavy trace: " << g_enable_heavy_trace << std::endl;
	    std::cout << "Cruise window trace: " << g_enable_cruise_window_trace << std::endl;
    std::cout << "Gate trace mode/sample interval us: "
              << g_gate_trace_mode << "/"
              << g_gate_trace_sample_interval_us << std::endl;
    std::cout << "Equivalence audit trace: "
              << g_enable_equivalence_audit_trace << std::endl;
    std::cout << "Convergence gate trace/control: "
              << g_enable_convergence_gate_trace << "/"
              << g_enable_convergence_gate_control << std::endl;
    std::cout << "Bottleneck bandwidth: "<<TOPO_BOTTLE_BW/1000000<<" Mbps" << std::endl;
    std::cout << "Bottleneck delay: "<<TOPO_BOTTLE_PDELAY<<" ms" << std::endl;
    std::cout << "Simulation time: " << sim_time << " seconds" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Per-flow parameters:" << std::endl;
    for(int i = 0; i < NUM_FLOWS; i++){
        std::cout << "  Flow " << (i+1) << ": freq=" << g_freq_hz[i] << "Hz, amp=" << g_amp_mode[i] << ", fixed=" << g_fixed_mbps[i] << "Mbps" << std::endl;
    }
    std::cout << "Interval window multiplier: " << g_interval_window_rtt_mult << " * min_rtt" << std::endl;
    if (g_dynamic_delay_enable) {
        std::cout << "Dynamic propagation delay schedule:" << std::endl;
        std::cout << "  t=4s:  edge=1ms, bottleneck=8ms, path_rtt=20ms" << std::endl;
        std::cout << "  t=7s:  edge=5ms, bottleneck=55ms, path_rtt=130ms" << std::endl;
        std::cout << "  t=12s: edge=1ms, bottleneck=12ms, path_rtt=28ms" << std::endl;
        std::cout << "  t=18s: edge=7ms, bottleneck=45ms, path_rtt=118ms" << std::endl;
        std::cout << "  t=22s: edge=2ms, bottleneck=6ms, path_rtt=20ms" << std::endl;
        std::cout << "  t=27s: edge=6ms, bottleneck=60ms, path_rtt=144ms" << std::endl;
    }
    std::cout << "====================================" << std::endl;

    const char *algos[]={""};
    for (size_t c = 0; c < sizeof(algos) / sizeof(algos[0]); ++c){
        std::string cong=algo;
        std::string name=cong;
        std::unique_ptr<DqcTraceState> stat;
        stat.reset(new DqcTraceState(name));
        auto inner_start = std::chrono::high_resolution_clock::now();
        for (size_t i = 0; i < sizeof(ins) / sizeof(ins[0]); ++i){
            ns3_freqccv4(ins[i],cong,stat.get(),sim_time);
        }
        auto inner_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> tm = inner_end - inner_start;
        std::chrono::duration<double, std::ratio<60>> minutes =inner_end- inner_start;
        stat->RecordRuningTime(tm.count(),minutes.count());
    }
    return 0;
}
