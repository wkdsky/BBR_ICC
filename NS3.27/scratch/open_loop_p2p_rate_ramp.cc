/**
 * Open-loop triangle-probe experiment with smooth paced background traffic.
 *
 * The probe baseline rises linearly from 180 to about 293 Mbps through the
 * first two Regimes, then follows a slope-continuous curve to 520 Mbps in
 * Regime III.  The three equal Regimes are analysis windows rather than
 * traffic-control steps.  A continuously active UDP background flow grows
 * smoothly from zero to about 163 Mbps in Regime I, follows a smooth
 * hump-shaped aggregate-load envelope in Regime II, then fades to zero under
 * a roughly 402 Mbps aggregate-rate target in Regime III.  Its smoothly changing
 * inverse-triangle component preserves a visible delivered-rate response
 * while residual aggregate modulation drives the requested SRTT regimes.
 *
 * Probe target rate:
 *   triangle modulation: same waveform as FreqCCv3 CalculateOscillationOffset()
 *   modulation frequency: 5 Hz
 *   send-rate modulation: constant 200 Mbps peak-to-peak
 *
 * Topology:
 *   n0 ---- 400 Mbps, 20 ms one-way delay ---- n1
 *   device queue on n0 is four BDP by default, using RTT = 2 * one-way delay.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-id-tag.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"

#include "queue_occupancy_trace_helper.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("open-loop-p2p-rate-ramp");

namespace {

double g_sender_start_time_s = 0.0;
double g_sender_duration_s = 9.0;
double g_drain_time_s = 0.5;

double g_link_bw_mbps = 400.0;
double g_one_way_delay_ms = 20.0;
double g_buffer_bdp = 4.0;
bool g_bdp_uses_rtt = true;

double g_start_rate_mbps = 180.0;
double g_regime_iii_start_rate_mbps = 293.333333;
double g_end_rate_mbps = 520.0;
double g_regime_iii_knee_fraction = 0.266667;
double g_regime_iii_knee_rate_mbps = 465.0;
double g_regime_iii_knee_slope_mbps_per_s = 40.0;
double g_triangle_freq_hz = 5.0;
double g_triangle_amp_mbps = 100.0;

double g_cross_regime_i_rate_mbps = 163.333;
double g_cross_regime_ii_control_start_mbps = 163.333;
double g_cross_regime_ii_end_rate_mbps = 106.667;
double g_cross_regime_ii_hump_mbps = 20.0;
double g_cross_regime_i_antiphase_start_fraction = 0.75;
double g_cross_regime_ii_antiphase_gain = 0.40;
double g_cross_end_rate_mbps = 0.0;
double g_cross_regime_iii_antiphase_gain = 0.40;
double g_cross_regime_iii_transition_fraction = 0.10;
double g_cross_regime_iii_target_aggregate_mbps = 402.0;
uint32_t g_cross_packet_size_bytes = 1448;

uint32_t g_packet_size_bytes = 1200;
uint32_t g_flow_id = 1;
uint16_t g_udp_port = 5000;
uint16_t g_cross_port = 5001;

double g_trace_interval_ms = 10.0;
std::string g_trace_path = "traces/open_loop_p2p_rate_ramp";
std::string g_trace_name = "open_loop_p2p_rate_ramp";

uint64_t
ToBps(double mbps)
{
    if (mbps <= 0.0)
    {
        return 0;
    }
    return static_cast<uint64_t>(mbps * 1000000.0 + 0.5);
}

uint32_t
ComputeBdpQueueBytes(uint64_t link_bps,
                     double one_way_delay_ms,
                     double buffer_bdp,
                     bool bdp_uses_rtt)
{
    const double bdp_delay_ms = bdp_uses_rtt ? 2.0 * one_way_delay_ms : one_way_delay_ms;
    const double queue_bytes = static_cast<double>(link_bps) * bdp_delay_ms * buffer_bdp / 8000.0;
    return static_cast<uint32_t>(std::max(1.0, queue_bytes));
}

double
ClampNonNegative(double value)
{
    return value > 0.0 ? value : 0.0;
}

class OpenLoopRampSender : public Application
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::OpenLoopRampSender")
                                .SetParent<Application>()
                                .SetGroupName("Applications")
                                .AddConstructor<OpenLoopRampSender>();
        return tid;
    }

    void Configure(Address peer,
                   uint32_t packet_size_bytes,
                   double start_rate_mbps,
                   double regime_iii_start_rate_mbps,
                   double end_rate_mbps,
                   double regime_iii_knee_fraction,
                   double regime_iii_knee_rate_mbps,
                   double regime_iii_knee_slope_mbps_per_s,
                   double start_time_s,
                   double duration_s,
                   double triangle_freq_hz,
                   double triangle_amp_mbps,
                   uint32_t flow_id)
    {
        m_peer = peer;
        m_packetSizeBytes = packet_size_bytes;
        m_startRateMbps = start_rate_mbps;
        m_regimeIIIStartRateMbps = regime_iii_start_rate_mbps;
        m_endRateMbps = end_rate_mbps;
        m_regimeIIIKneeFraction =
            std::max(0.05, std::min(0.95, regime_iii_knee_fraction));
        m_regimeIIIKneeRateMbps = regime_iii_knee_rate_mbps;
        m_regimeIIIKneeSlopeMbpsPerS =
            std::max(0.0, regime_iii_knee_slope_mbps_per_s);
        m_startTimeS = start_time_s;
        m_durationS = duration_s;
        m_triangleFreqHz = triangle_freq_hz;
        m_triangleAmpMbps = triangle_amp_mbps;
        m_flowId = flow_id;
    }

    uint64_t GetTotalTxBytes() const
    {
        return m_totalTxBytes;
    }

    double GetStartTimeSeconds() const
    {
        return m_startTimeS;
    }

    double GetStopTimeSeconds() const
    {
        return m_startTimeS + m_durationS;
    }

    double GetTriangleAmplitudeMbps() const
    {
        return m_triangleAmpMbps;
    }

    double GetElapsedSeconds(double now_s) const
    {
        return std::max(0.0, now_s - m_startTimeS);
    }

    double GetBaselineRateMbpsAt(double now_s) const
    {
        if (now_s < m_startTimeS || now_s > GetStopTimeSeconds())
        {
            return 0.0;
        }
        const double elapsed_s = std::min(GetElapsedSeconds(now_s), m_durationS);
        if (m_durationS <= 0.0)
        {
            return m_endRateMbps;
        }

        const double regime_duration_s = m_durationS / 3.0;
        const double regime_iii_start_s = 2.0 * regime_duration_s;
        if (elapsed_s <= regime_iii_start_s || regime_duration_s <= 0.0)
        {
            const double progress = elapsed_s / std::max(regime_iii_start_s, 1e-9);
            return m_startRateMbps +
                   (m_regimeIIIStartRateMbps - m_startRateMbps) * progress;
        }

        // Two slope-continuous cubic Hermite segments rapidly but smoothly
        // finish the upper-clipping transition.  The knee is aligned with the
        // desired right-hand visible-oscillation margin, while the long tail
        // remains monotone and ends with zero slope.
        const double x = std::min(1.0, (elapsed_s - regime_iii_start_s) / regime_duration_s);
        const double pre_regime_slope_mbps_per_s =
            (m_regimeIIIStartRateMbps - m_startRateMbps) /
            std::max(regime_iii_start_s, 1e-9);
        const auto hermite = [](double local_x,
                                double start_value,
                                double end_value,
                                double start_tangent,
                                double end_tangent) {
            const double x2 = local_x * local_x;
            const double x3 = x2 * local_x;
            const double h00 = 2.0 * x3 - 3.0 * x2 + 1.0;
            const double h10 = x3 - 2.0 * x2 + local_x;
            const double h01 = -2.0 * x3 + 3.0 * x2;
            const double h11 = x3 - x2;
            return h00 * start_value + h10 * start_tangent +
                   h01 * end_value + h11 * end_tangent;
        };

        if (x <= m_regimeIIIKneeFraction)
        {
            const double segment_fraction = m_regimeIIIKneeFraction;
            const double local_x = x / segment_fraction;
            const double segment_duration_s = regime_duration_s * segment_fraction;
            return hermite(local_x,
                           m_regimeIIIStartRateMbps,
                           m_regimeIIIKneeRateMbps,
                           pre_regime_slope_mbps_per_s * segment_duration_s,
                           m_regimeIIIKneeSlopeMbpsPerS * segment_duration_s);
        }

        const double segment_fraction = 1.0 - m_regimeIIIKneeFraction;
        const double local_x = (x - m_regimeIIIKneeFraction) / segment_fraction;
        const double segment_duration_s = regime_duration_s * segment_fraction;
        return hermite(local_x,
                       m_regimeIIIKneeRateMbps,
                       m_endRateMbps,
                       m_regimeIIIKneeSlopeMbpsPerS * segment_duration_s,
                       0.0);
    }

    double GetTriangleValueAt(double now_s) const
    {
        if (now_s < m_startTimeS || now_s > GetStopTimeSeconds() || m_triangleFreqHz <= 0.0)
        {
            return 0.0;
        }

        const double elapsed_s = GetElapsedSeconds(now_s);
        const double period_s = 1.0 / m_triangleFreqHz;
        const double phase = std::fmod(elapsed_s, period_s) / period_s;

        if (phase < 0.25)
        {
            return phase * 4.0;
        }
        if (phase < 0.75)
        {
            return 2.0 - phase * 4.0;
        }
        return phase * 4.0 - 4.0;
    }

    double GetTargetRateMbpsAt(double now_s) const
    {
        const double baseline_mbps = GetBaselineRateMbpsAt(now_s);
        if (baseline_mbps <= 0.0)
        {
            return 0.0;
        }
        return ClampNonNegative(baseline_mbps +
                                m_triangleAmpMbps * GetTriangleValueAt(now_s));
    }

  private:
    void StartApplication() override
    {
        m_running = true;
        m_startTimeS = Simulator::Now().GetSeconds();
        m_totalTxBytes = 0;

        m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_socket->Connect(m_peer);

        Simulator::ScheduleNow(&OpenLoopRampSender::SendPacket, this);
    }

    void StopApplication() override
    {
        m_running = false;
        if (m_sendEvent.IsRunning())
        {
            Simulator::Cancel(m_sendEvent);
        }
        if (m_socket != nullptr)
        {
            m_socket->Close();
            m_socket = nullptr;
        }
    }

    void SendPacket()
    {
        if (!m_running || m_socket == nullptr)
        {
            return;
        }

        const double now_s = Simulator::Now().GetSeconds();
        if (now_s >= GetStopTimeSeconds())
        {
            return;
        }

        Ptr<Packet> packet = Create<Packet>(m_packetSizeBytes);
        packet->AddPacketTag(FlowIdTag(m_flowId));

        const int sent_bytes = m_socket->Send(packet);
        if (sent_bytes > 0)
        {
            m_totalTxBytes += static_cast<uint64_t>(sent_bytes);
        }

        ScheduleNextSend();
    }

    void ScheduleNextSend()
    {
        if (!m_running)
        {
            return;
        }

        const double now_s = Simulator::Now().GetSeconds();
        const double rate_mbps = GetTargetRateMbpsAt(now_s);
        if (rate_mbps <= 0.0)
        {
            m_sendEvent = Simulator::Schedule(MilliSeconds(1), &OpenLoopRampSender::SendPacket, this);
            return;
        }

        const double interval_s = (static_cast<double>(m_packetSizeBytes) * 8.0) /
                                  (rate_mbps * 1000000.0);
        m_sendEvent = Simulator::Schedule(Seconds(interval_s), &OpenLoopRampSender::SendPacket, this);
    }

    Ptr<Socket> m_socket;
    Address m_peer;
    EventId m_sendEvent;
    bool m_running{false};

    uint32_t m_packetSizeBytes{1200};
    double m_startRateMbps{100.0};
    double m_regimeIIIStartRateMbps{293.333333};
    double m_endRateMbps{450.0};
    double m_regimeIIIKneeFraction{0.266667};
    double m_regimeIIIKneeRateMbps{465.0};
    double m_regimeIIIKneeSlopeMbpsPerS{40.0};
    double m_durationS{30.0};
    double m_triangleFreqHz{5.0};
    double m_triangleAmpMbps{100.0};
    uint32_t m_flowId{1};

    double m_startTimeS{0.0};
    uint64_t m_totalTxBytes{0};
};

NS_OBJECT_ENSURE_REGISTERED(OpenLoopRampSender);

class SmoothCrossTrafficSender : public Application
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::SmoothCrossTrafficSender")
                                .SetParent<Application>()
                                .SetGroupName("Applications")
                                .AddConstructor<SmoothCrossTrafficSender>();
        return tid;
    }

    void Configure(Address peer,
                   Ptr<OpenLoopRampSender> probe_sender,
                   uint32_t packet_size_bytes,
                   double regime_i_rate_mbps,
                   double regime_ii_control_start_mbps,
                   double regime_ii_end_rate_mbps,
                   double regime_ii_hump_mbps,
                   double regime_ii_antiphase_gain,
                   double end_rate_mbps,
                   double regime_iii_antiphase_gain,
                   double regime_iii_transition_fraction,
                   double regime_iii_target_aggregate_mbps,
                   double data_start_time_s)
    {
        m_peer = peer;
        m_probeSender = probe_sender;
        m_packetSizeBytes = packet_size_bytes;
        m_regimeIRateMbps = regime_i_rate_mbps;
        m_regimeIIControlStartMbps = regime_ii_control_start_mbps;
        m_regimeIIEndRateMbps = regime_ii_end_rate_mbps;
        m_regimeIIHumpMbps = std::max(0.0, regime_ii_hump_mbps);
        m_regimeIIAntiphaseGain = std::max(0.0, regime_ii_antiphase_gain);
        m_endRateMbps = end_rate_mbps;
        m_regimeIIIAntiphaseGain = std::max(0.0, regime_iii_antiphase_gain);
        m_regimeIIITransitionFraction =
            std::max(0.01, std::min(1.0, regime_iii_transition_fraction));
        m_regimeIIITargetAggregateMbps = regime_iii_target_aggregate_mbps;
        m_dataStartTimeS = data_start_time_s;
    }

    uint64_t GetTotalTxBytes() const
    {
        return m_totalTxBytes;
    }

    double GetTargetRateMbpsAt(double now_s) const
    {
        if (m_probeSender == nullptr)
        {
            return 0.0;
        }
        const double start_s = m_probeSender->GetStartTimeSeconds();
        const double duration_s = m_probeSender->GetStopTimeSeconds() - start_s;
        if (now_s < start_s || duration_s <= 0.0)
        {
            return 0.0;
        }
        const double elapsed_s = std::min(now_s - start_s, duration_s);
        const double regime_duration_s = duration_s / 3.0;
        double mean_rate_mbps = m_regimeIIEndRateMbps;
        if (elapsed_s < regime_duration_s)
        {
            const double x = std::max(0.0, elapsed_s / regime_duration_s);
            // A fourth-order terminal Hermite curve stays below the link
            // threshold until the final Regime-I cycles while retaining zero
            // slope at the 3 s boundary.  Thus every cycle before the boundary
            // still touches the queue floor, but the first cycle after it does
            // not.
            const double x2 = x * x;
            const double x4 = x2 * x2;
            const double late_rise = x4 * (5.0 - 4.0 * x);
            mean_rate_mbps = m_regimeIRateMbps * late_rise;

            // Introduce the inverse component only as the bottleneck is first
            // reached.  This keeps the delivered probe modulation constant
            // while retaining a large aggregate triangle that drives SRTT.
            const double transition_span = std::max(
                0.01,
                1.0 - g_cross_regime_i_antiphase_start_fraction);
            const double antiphase_x = std::max(
                0.0,
                std::min(1.0,
                         (x - g_cross_regime_i_antiphase_start_fraction) /
                             transition_span));
            const double antiphase_smoothstep =
                antiphase_x * antiphase_x * (3.0 - 2.0 * antiphase_x);
            const double probe_triangle_offset_mbps =
                m_probeSender->GetTargetRateMbpsAt(now_s) -
                m_probeSender->GetBaselineRateMbpsAt(now_s);
            mean_rate_mbps -= m_regimeIIAntiphaseGain * antiphase_smoothstep *
                              probe_triangle_offset_mbps;
        }
        else if (elapsed_s < 2.0 * regime_duration_s)
        {
            const double x = (elapsed_s - regime_duration_s) / regime_duration_s;
            // The linear part exactly cancels the probe-baseline slope.  The
            // sin^2 hump fills the 4-BDP queue with zero extra-load slope at
            // both boundaries, avoiding the former kink at 6 s.
            const double linear_control_rate_mbps =
                m_regimeIIControlStartMbps +
                (m_regimeIIEndRateMbps - m_regimeIIControlStartMbps) * x;
            const double hump = std::sin(M_PI * x);
            mean_rate_mbps = linear_control_rate_mbps +
                             m_regimeIIHumpMbps * hump * hump;

            const double probe_triangle_offset_mbps =
                m_probeSender->GetTargetRateMbpsAt(now_s) -
                m_probeSender->GetBaselineRateMbpsAt(now_s);
            mean_rate_mbps -=
                m_regimeIIAntiphaseGain * probe_triangle_offset_mbps;
        }
        else
        {
            const double x = (elapsed_s - 2.0 * regime_duration_s) / regime_duration_s;
            const double join_x = std::min(1.0, x / m_regimeIIITransitionFraction);
            const double join_smoothstep = join_x * join_x * (3.0 - 2.0 * join_x);
            const double probe_baseline_mbps =
                m_probeSender->GetBaselineRateMbpsAt(now_s);
            const double controlled_rate_mbps = std::max(
                m_endRateMbps,
                m_regimeIIITargetAggregateMbps - probe_baseline_mbps);
            mean_rate_mbps = m_regimeIIEndRateMbps +
                             (controlled_rate_mbps - m_regimeIIEndRateMbps) *
                                 join_smoothstep;

            const double probe_triangle_offset_mbps =
                m_probeSender->GetTargetRateMbpsAt(now_s) -
                m_probeSender->GetBaselineRateMbpsAt(now_s);
            double effective_antiphase_gain =
                m_regimeIIAntiphaseGain +
                (m_regimeIIIAntiphaseGain - m_regimeIIAntiphaseGain) *
                    join_smoothstep;
            const double maximum_inverse_offset_mbps =
                std::abs(effective_antiphase_gain) *
                m_probeSender->GetTriangleAmplitudeMbps();
            if (maximum_inverse_offset_mbps > 0.0)
            {
                const double availability_scale =
                    std::min(1.0, mean_rate_mbps / maximum_inverse_offset_mbps);
                effective_antiphase_gain *= availability_scale;
            }
            mean_rate_mbps -= effective_antiphase_gain * probe_triangle_offset_mbps;
        }
        return ClampNonNegative(mean_rate_mbps);
    }

  private:
    void StartApplication() override
    {
        m_running = true;
        m_totalTxBytes = 0;
        m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_socket->Connect(m_peer);
        const double delay_s = std::max(0.0, m_dataStartTimeS - Simulator::Now().GetSeconds());
        m_sendEvent = Simulator::Schedule(Seconds(delay_s),
                                          &SmoothCrossTrafficSender::SendPacket,
                                          this);
    }

    void StopApplication() override
    {
        m_running = false;
        if (m_sendEvent.IsRunning())
        {
            Simulator::Cancel(m_sendEvent);
        }
        if (m_socket != nullptr)
        {
            m_socket->Close();
            m_socket = nullptr;
        }
    }

    void SendPacket()
    {
        if (!m_running || m_socket == nullptr)
        {
            return;
        }

        const double rate_mbps = GetTargetRateMbpsAt(Simulator::Now().GetSeconds());
        if (rate_mbps < 0.1)
        {
            m_sendEvent = Simulator::Schedule(MilliSeconds(1),
                                               &SmoothCrossTrafficSender::SendPacket,
                                               this);
            return;
        }

        const int sent_bytes = m_socket->Send(Create<Packet>(m_packetSizeBytes));
        if (sent_bytes > 0)
        {
            m_totalTxBytes += static_cast<uint64_t>(sent_bytes);
        }

        if (sent_bytes <= 0)
        {
            m_sendEvent = Simulator::Schedule(MicroSeconds(100),
                                               &SmoothCrossTrafficSender::SendPacket,
                                               this);
            return;
        }

        const double interval_s =
            static_cast<double>(m_packetSizeBytes) * 8.0 / (rate_mbps * 1000000.0);
        m_sendEvent = Simulator::Schedule(Seconds(interval_s),
                                           &SmoothCrossTrafficSender::SendPacket,
                                           this);
    }

    Ptr<Socket> m_socket;
    Ptr<OpenLoopRampSender> m_probeSender;
    Address m_peer;
    EventId m_sendEvent;
    bool m_running{false};
    uint32_t m_packetSizeBytes{1448};
    double m_regimeIRateMbps{100.0};
    double m_regimeIIControlStartMbps{138.0};
    double m_regimeIIEndRateMbps{52.0};
    double m_regimeIIHumpMbps{20.0};
    double m_regimeIIAntiphaseGain{0.20};
    double m_endRateMbps{40.0};
    double m_regimeIIIAntiphaseGain{0.40};
    double m_regimeIIITransitionFraction{0.10};
    double m_regimeIIITargetAggregateMbps{402.0};
    double m_dataStartTimeS{0.0};
    uint64_t m_totalTxBytes{0};
};

NS_OBJECT_ENSURE_REGISTERED(SmoothCrossTrafficSender);

class OpenLoopRateTracer
{
  public:
    OpenLoopRateTracer(Ptr<OpenLoopRampSender> sender,
                       Ptr<PacketSink> probe_sink,
                       Ptr<PacketSink> cross_sink,
                       const std::string& folder,
                       const std::string& trace_name,
                       double interval_s,
                       double stop_time_s)
        : m_sender(sender),
          m_probeSink(probe_sink),
          m_crossSink(cross_sink),
          m_intervalS(interval_s),
          m_stopTimeS(stop_time_s)
    {
        EnsureDirectoryExists(folder);
        m_stream.open((folder + trace_name + "_rate_trace.txt").c_str(), std::fstream::out);
        if (m_stream.is_open())
        {
            m_stream << "#time_s\tbaseline_mbps\ttriangle_value\ttarget_mbps"
                     << "\ttx_mbps\trx_mbps\ttotal_tx_bytes\ttotal_rx_bytes"
                     << "\tcross_rx_mbps\taggregate_rx_mbps\ttotal_cross_rx_bytes"
                     << std::endl;
        }
    }

    void Start()
    {
        m_lastSampleS = Simulator::Now().GetSeconds();
        m_lastTxBytes = m_sender != nullptr ? m_sender->GetTotalTxBytes() : 0;
        m_lastProbeRxBytes = m_probeSink != nullptr ? m_probeSink->GetTotalRx() : 0;
        m_lastCrossRxBytes = m_crossSink != nullptr ? m_crossSink->GetTotalRx() : 0;
        WriteSample();
    }

  private:
    void WriteSample()
    {
        if (!m_stream.is_open() || m_sender == nullptr || m_probeSink == nullptr ||
            m_crossSink == nullptr)
        {
            return;
        }

        const double now_s = Simulator::Now().GetSeconds();
        const uint64_t total_tx_bytes = m_sender->GetTotalTxBytes();
        const uint64_t total_probe_rx_bytes = m_probeSink->GetTotalRx();
        const uint64_t total_cross_rx_bytes = m_crossSink->GetTotalRx();
        const double delta_s = std::max(1e-9, now_s - m_lastSampleS);
        const double tx_mbps =
            static_cast<double>(total_tx_bytes - m_lastTxBytes) * 8.0 / delta_s / 1000000.0;
        const double probe_rx_mbps =
            static_cast<double>(total_probe_rx_bytes - m_lastProbeRxBytes) * 8.0 /
            delta_s / 1000000.0;
        const double cross_rx_mbps =
            static_cast<double>(total_cross_rx_bytes - m_lastCrossRxBytes) * 8.0 /
            delta_s / 1000000.0;

        m_stream << std::fixed << std::setprecision(6)
                 << now_s << "\t"
                 << m_sender->GetBaselineRateMbpsAt(now_s) << "\t"
                 << m_sender->GetTriangleValueAt(now_s) << "\t"
                 << m_sender->GetTargetRateMbpsAt(now_s) << "\t"
                 << tx_mbps << "\t"
                 << probe_rx_mbps << "\t"
                 << total_tx_bytes << "\t"
                 << total_probe_rx_bytes << "\t"
                 << cross_rx_mbps << "\t"
                 << (probe_rx_mbps + cross_rx_mbps) << "\t"
                 << total_cross_rx_bytes << std::endl;

        m_lastSampleS = now_s;
        m_lastTxBytes = total_tx_bytes;
        m_lastProbeRxBytes = total_probe_rx_bytes;
        m_lastCrossRxBytes = total_cross_rx_bytes;

        if (now_s + m_intervalS <= m_stopTimeS)
        {
            Simulator::Schedule(Seconds(m_intervalS), &OpenLoopRateTracer::WriteSample, this);
        }
    }

    Ptr<OpenLoopRampSender> m_sender;
    Ptr<PacketSink> m_probeSink;
    Ptr<PacketSink> m_crossSink;
    std::fstream m_stream;
    double m_intervalS{0.01};
    double m_stopTimeS{30.0};
    double m_lastSampleS{0.0};
    uint64_t m_lastTxBytes{0};
    uint64_t m_lastProbeRxBytes{0};
    uint64_t m_lastCrossRxBytes{0};
};

class QueueDropTracer
{
  public:
    QueueDropTracer(Ptr<NetDevice> device, const std::string& folder, const std::string& trace_name)
    {
        Ptr<PointToPointNetDevice> ptp_device = DynamicCast<PointToPointNetDevice>(device);
        if (ptp_device == nullptr)
        {
            return;
        }

        m_queue = ptp_device->GetQueue();
        if (m_queue == nullptr)
        {
            return;
        }

        EnsureDirectoryExists(folder);
        m_stream.open((folder + trace_name + "_drop_trace.txt").c_str(), std::fstream::out);
        if (m_stream.is_open())
        {
            m_stream << "#time_s\tpacket_bytes\tflow_id\tcum_drop_packets\tcum_drop_bytes" << std::endl;
        }

        m_queue->TraceConnectWithoutContext("Drop", MakeCallback(&QueueDropTracer::OnDrop, this));
    }

    uint64_t GetDropPackets() const
    {
        return m_dropPackets;
    }

    uint64_t GetDropBytes() const
    {
        return m_dropBytes;
    }

  private:
    void OnDrop(Ptr<const Packet> packet)
    {
        if (packet == nullptr)
        {
            return;
        }

        const uint32_t packet_bytes = packet->GetSize();
        uint32_t flow_id = 0;
        FlowIdTag flow_tag;
        if (packet->PeekPacketTag(flow_tag))
        {
            flow_id = flow_tag.GetFlowId();
        }

        ++m_dropPackets;
        m_dropBytes += packet_bytes;

        if (m_stream.is_open())
        {
            m_stream << std::fixed << std::setprecision(6)
                     << Simulator::Now().GetSeconds() << "\t"
                     << packet_bytes << "\t"
                     << flow_id << "\t"
                     << m_dropPackets << "\t"
                     << m_dropBytes << std::endl;
        }
    }

    Ptr<Queue<Packet>> m_queue;
    std::fstream m_stream;
    uint64_t m_dropPackets{0};
    uint64_t m_dropBytes{0};
};

void
WriteScenarioConfig(const std::string& folder,
                    const std::string& trace_name,
                    uint64_t link_bps,
                    uint32_t queue_bytes,
                    double sim_stop_s)
{
    EnsureDirectoryExists(folder);
    std::fstream stream((folder + trace_name + "_config.txt").c_str(), std::fstream::out);
    if (!stream.is_open())
    {
        return;
    }

    stream << std::fixed << std::setprecision(6)
           << "sender_start_time_s\t" << g_sender_start_time_s << std::endl
           << "sender_duration_s\t" << g_sender_duration_s << std::endl
           << "drain_time_s\t" << g_drain_time_s << std::endl
           << "sim_stop_s\t" << sim_stop_s << std::endl
           << "link_bw_mbps\t" << g_link_bw_mbps << std::endl
           << "link_bps\t" << link_bps << std::endl
           << "one_way_delay_ms\t" << g_one_way_delay_ms << std::endl
           << "buffer_bdp\t" << g_buffer_bdp << std::endl
           << "bdp_uses_rtt\t" << (g_bdp_uses_rtt ? 1 : 0) << std::endl
           << "queue_bytes\t" << queue_bytes << std::endl
           << "regime_duration_s\t" << (g_sender_duration_s / 3.0) << std::endl
           << "start_rate_mbps\t" << g_start_rate_mbps << std::endl
           << "regime_iii_start_rate_mbps\t" << g_regime_iii_start_rate_mbps << std::endl
           << "end_rate_mbps\t" << g_end_rate_mbps << std::endl
           << "regime_iii_knee_fraction\t" << g_regime_iii_knee_fraction << std::endl
           << "regime_iii_knee_rate_mbps\t" << g_regime_iii_knee_rate_mbps << std::endl
           << "regime_iii_knee_slope_mbps_per_s\t"
           << g_regime_iii_knee_slope_mbps_per_s << std::endl
           << "triangle_freq_hz\t" << g_triangle_freq_hz << std::endl
           << "triangle_amp_mbps\t" << g_triangle_amp_mbps << std::endl
           << "srate_peak_to_peak_mbps\t" << (2.0 * g_triangle_amp_mbps) << std::endl
           << "cross_start_time_s\t" << g_sender_start_time_s << std::endl
           << "cross_regime_i_rate_mbps\t" << g_cross_regime_i_rate_mbps << std::endl
           << "cross_regime_ii_control_start_mbps\t"
           << g_cross_regime_ii_control_start_mbps << std::endl
           << "cross_regime_ii_end_rate_mbps\t" << g_cross_regime_ii_end_rate_mbps << std::endl
           << "cross_regime_ii_hump_mbps\t" << g_cross_regime_ii_hump_mbps << std::endl
           << "cross_regime_i_antiphase_start_fraction\t"
           << g_cross_regime_i_antiphase_start_fraction << std::endl
           << "cross_regime_ii_antiphase_gain\t"
           << g_cross_regime_ii_antiphase_gain << std::endl
           << "cross_end_rate_mbps\t" << g_cross_end_rate_mbps << std::endl
           << "cross_regime_iii_antiphase_gain\t"
           << g_cross_regime_iii_antiphase_gain << std::endl
           << "cross_regime_iii_transition_fraction\t"
           << g_cross_regime_iii_transition_fraction << std::endl
           << "cross_regime_iii_target_aggregate_mbps\t"
           << g_cross_regime_iii_target_aggregate_mbps << std::endl
           << "cross_packet_size_bytes\t" << g_cross_packet_size_bytes << std::endl
           << "packet_size_bytes\t" << g_packet_size_bytes << std::endl
           << "trace_interval_ms\t" << g_trace_interval_ms << std::endl;
}

void
RunScenario()
{
    const uint64_t link_bps = ToBps(g_link_bw_mbps);
    const uint32_t queue_bytes =
        ComputeBdpQueueBytes(link_bps, g_one_way_delay_ms, g_buffer_bdp, g_bdp_uses_rtt);
    const double sim_stop_s = g_sender_start_time_s + g_sender_duration_s + g_drain_time_s;
    const double regime_duration_s = g_sender_duration_s / 3.0;
    const double cross_start_s = g_sender_start_time_s;
    const double cross_stop_s = g_sender_start_time_s + g_sender_duration_s;

    SetQueueOccupancyTraceFolder(g_trace_path);
    const std::string trace_folder = GetQueueOccupancyTraceFolder();
    WriteScenarioConfig(trace_folder, g_trace_name, link_bps, queue_bytes, sim_stop_s);

    NodeContainer nodes;
    nodes.Create(2);

    InternetStackHelper internet;
    internet.Install(nodes);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate(link_bps)));
    p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(g_one_way_delay_ms)));
    p2p.SetQueue("ns3::DropTailQueue",
                 "Mode",
                 StringValue("QUEUE_MODE_BYTES"),
                 "MaxBytes",
                 UintegerValue(queue_bytes));

    NetDeviceContainer devices = p2p.Install(nodes);

    std::shared_ptr<QueueOccupancyTracer> queue_tracer =
        InstallBottleneckQueueOccupancyTrace(devices.Get(0), g_trace_name, 1);
    std::shared_ptr<QueueDropTracer> drop_tracer =
        std::make_shared<QueueDropTracer>(devices.Get(0), trace_folder, g_trace_name);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);
    TrafficControlHelper tch;
    tch.Uninstall(devices);
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    PacketSinkHelper sink_helper("ns3::UdpSocketFactory",
                                 InetSocketAddress(Ipv4Address::GetAny(), g_udp_port));
    ApplicationContainer sink_apps = sink_helper.Install(nodes.Get(1));
    sink_apps.Start(Seconds(0.0));
    sink_apps.Stop(Seconds(sim_stop_s));
    Ptr<PacketSink> sink = DynamicCast<PacketSink>(sink_apps.Get(0));

    PacketSinkHelper cross_sink_helper(
        "ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), g_cross_port));
    ApplicationContainer cross_sink_apps = cross_sink_helper.Install(nodes.Get(1));
    cross_sink_apps.Start(Seconds(0.0));
    cross_sink_apps.Stop(Seconds(sim_stop_s));
    Ptr<PacketSink> cross_sink = DynamicCast<PacketSink>(cross_sink_apps.Get(0));

    Ptr<OpenLoopRampSender> sender = CreateObject<OpenLoopRampSender>();
    sender->Configure(InetSocketAddress(interfaces.GetAddress(1), g_udp_port),
                      g_packet_size_bytes,
                      g_start_rate_mbps,
                      g_regime_iii_start_rate_mbps,
                      g_end_rate_mbps,
                      g_regime_iii_knee_fraction,
                      g_regime_iii_knee_rate_mbps,
                      g_regime_iii_knee_slope_mbps_per_s,
                      g_sender_start_time_s,
                      g_sender_duration_s,
                      g_triangle_freq_hz,
                      g_triangle_amp_mbps,
                      g_flow_id);
    nodes.Get(0)->AddApplication(sender);
    sender->SetStartTime(Seconds(g_sender_start_time_s));
    sender->SetStopTime(Seconds(g_sender_start_time_s + g_sender_duration_s));

    Ptr<SmoothCrossTrafficSender> cross_sender = CreateObject<SmoothCrossTrafficSender>();
    cross_sender->Configure(InetSocketAddress(interfaces.GetAddress(1), g_cross_port),
                            sender,
                            g_cross_packet_size_bytes,
                            g_cross_regime_i_rate_mbps,
                            g_cross_regime_ii_control_start_mbps,
                            g_cross_regime_ii_end_rate_mbps,
                            g_cross_regime_ii_hump_mbps,
                            g_cross_regime_ii_antiphase_gain,
                            g_cross_end_rate_mbps,
                            g_cross_regime_iii_antiphase_gain,
                            g_cross_regime_iii_transition_fraction,
                            g_cross_regime_iii_target_aggregate_mbps,
                            cross_start_s);
    nodes.Get(0)->AddApplication(cross_sender);
    cross_sender->SetStartTime(Seconds(cross_start_s));
    cross_sender->SetStopTime(Seconds(cross_stop_s));

    std::shared_ptr<OpenLoopRateTracer> rate_tracer =
        std::make_shared<OpenLoopRateTracer>(sender,
                                             sink,
                                             cross_sink,
                                             trace_folder,
                                             g_trace_name,
                                             g_trace_interval_ms / 1000.0,
                                             sim_stop_s);
    Simulator::Schedule(Seconds(g_sender_start_time_s), &OpenLoopRateTracer::Start, rate_tracer.get());

    std::cout << "=== Continuous triangle probe plus smooth background traffic ===" << std::endl;
    std::cout << "sender_duration=" << g_sender_duration_s << "s, sim_stop=" << sim_stop_s
              << "s" << std::endl;
    std::cout << "link=" << g_link_bw_mbps << "Mbps, one_way_delay=" << g_one_way_delay_ms
              << "ms" << std::endl;
    std::cout << "queue=" << queue_bytes << " bytes (" << g_buffer_bdp
              << " BDP, basis=" << (g_bdp_uses_rtt ? "RTT" : "one-way") << ")" << std::endl;
    std::cout << "regimes=[0," << regime_duration_s << "),[" << regime_duration_s
              << "," << 2.0 * regime_duration_s << "),["
              << 2.0 * regime_duration_s << "," << g_sender_duration_s << ")s"
              << std::endl;
    std::cout << "probe_baseline_continuous=" << g_start_rate_mbps << "->"
              << g_regime_iii_start_rate_mbps << "@6s->" << g_end_rate_mbps
              << "Mbps (Regime-III knee=" << g_regime_iii_knee_rate_mbps << "Mbps at "
              << g_regime_iii_knee_fraction << ")"
              << ", triangle_freq=" << g_triangle_freq_hz
              << "Hz, constant_triangle_amp=" << g_triangle_amp_mbps
              << "Mbps (" << (2.0 * g_triangle_amp_mbps) << "Mbps peak-to-peak)"
              << std::endl;
    std::cout << "cross_active=" << cross_start_s << "->" << cross_stop_s
              << "s, cross_curve=0->" << g_cross_regime_i_rate_mbps << "->"
              << "control(" << g_cross_regime_ii_control_start_mbps << ")->"
              << g_cross_regime_ii_end_rate_mbps << "->" << g_cross_end_rate_mbps
              << "Mbps, regime_ii_hump=" << g_cross_regime_ii_hump_mbps
              << ", antiphase_gain=" << g_cross_regime_ii_antiphase_gain
              << ", regime_iii_antiphase_gain=" << g_cross_regime_iii_antiphase_gain
              << std::endl;
    std::cout << "packet_size=" << g_packet_size_bytes << " bytes" << std::endl;
    std::cout << "trace_folder=" << trace_folder << std::endl;
    std::cout << "=========================================" << std::endl;

    Simulator::Stop(Seconds(sim_stop_s));
    Simulator::Run();
    Simulator::Destroy();

    (void)queue_tracer;
    std::cout << "drop_packets=" << drop_tracer->GetDropPackets()
              << ", drop_bytes=" << drop_tracer->GetDropBytes() << std::endl;
    std::cout << "total_tx_bytes=" << sender->GetTotalTxBytes()
              << ", total_rx_bytes=" << sink->GetTotalRx()
              << ", total_cross_tx_bytes=" << cross_sender->GetTotalTxBytes()
              << ", total_cross_rx_bytes=" << cross_sink->GetTotalRx() << std::endl;
}

} // namespace

int
main(int argc, char* argv[])
{
    CommandLine cmd;
    cmd.AddValue("sender_start_time_s", "Sender start time in seconds", g_sender_start_time_s);
    cmd.AddValue("sender_duration_s",
                 "Active duration in seconds, split equally across three regimes",
                 g_sender_duration_s);
    cmd.AddValue("drain_time_s", "Extra simulation time after sender stops", g_drain_time_s);

    cmd.AddValue("link_bw_mbps", "P2P link bandwidth in Mbps", g_link_bw_mbps);
    cmd.AddValue("one_way_delay_ms", "P2P one-way propagation delay in ms", g_one_way_delay_ms);
    cmd.AddValue("buffer_bdp", "Sender-side device queue size in BDP units", g_buffer_bdp);
    cmd.AddValue("bdp_uses_rtt", "Use RTT rather than one-way delay for BDP queue sizing", g_bdp_uses_rtt);

    cmd.AddValue("start_rate_mbps",
                 "Initial triangle-probe baseline",
                 g_start_rate_mbps);
    cmd.AddValue("regime_iii_start_rate_mbps",
                 "Triangle-probe baseline at the second Regime boundary",
                 g_regime_iii_start_rate_mbps);
    cmd.AddValue("end_rate_mbps",
                 "Final triangle-probe baseline reached by a smooth Regime-III curve",
                 g_end_rate_mbps);
    cmd.AddValue("regime_iii_knee_fraction",
                 "Normalized Regime-III time of the smooth fast-rise knee",
                 g_regime_iii_knee_fraction);
    cmd.AddValue("regime_iii_knee_rate_mbps",
                 "Probe baseline at the Regime-III knee",
                 g_regime_iii_knee_rate_mbps);
    cmd.AddValue("regime_iii_knee_slope_mbps_per_s",
                 "Probe-baseline slope shared by both sides of the Regime-III knee",
                 g_regime_iii_knee_slope_mbps_per_s);
    cmd.AddValue("triangle_freq_hz", "Triangle modulation frequency in Hz", g_triangle_freq_hz);
    cmd.AddValue("triangle_amp_mbps", "Triangle modulation peak amplitude in Mbps", g_triangle_amp_mbps);

    cmd.AddValue("cross_regime_i_rate_mbps",
                 "Background-flow rate reached smoothly at the first Regime boundary",
                 g_cross_regime_i_rate_mbps);
    cmd.AddValue("cross_regime_ii_control_start_mbps",
                 "Initial Regime-II background-flow control-envelope rate",
                 g_cross_regime_ii_control_start_mbps);
    cmd.AddValue("cross_regime_ii_end_rate_mbps",
                 "Background-flow rate at the second Regime boundary",
                 g_cross_regime_ii_end_rate_mbps);
    cmd.AddValue("cross_regime_ii_hump_mbps",
                 "Smooth Regime-II aggregate-load hump amplitude",
                 g_cross_regime_ii_hump_mbps);
    cmd.AddValue("cross_regime_i_antiphase_start_fraction",
                 "Regime-I fraction where the inverse-triangle background component starts",
                 g_cross_regime_i_antiphase_start_fraction);
    cmd.AddValue("cross_regime_ii_antiphase_gain",
                 "Regime-II background-flow inverse-triangle gain",
                 g_cross_regime_ii_antiphase_gain);
    cmd.AddValue("cross_end_rate_mbps",
                 "Background-flow rate reached smoothly at the experiment end",
                 g_cross_end_rate_mbps);
    cmd.AddValue("cross_regime_iii_antiphase_gain",
                 "Regime-III background-flow inverse-triangle gain",
                 g_cross_regime_iii_antiphase_gain);
    cmd.AddValue("cross_regime_iii_transition_fraction",
                 "Fraction of Regime III used to join target aggregate-load control",
                 g_cross_regime_iii_transition_fraction);
    cmd.AddValue("cross_regime_iii_target_aggregate_mbps",
                 "Regime-III aggregate mean held while the background flow remains active",
                 g_cross_regime_iii_target_aggregate_mbps);
    cmd.AddValue("cross_packet_size_bytes",
                 "Background UDP flow packet size",
                 g_cross_packet_size_bytes);
    cmd.AddValue("cross_port", "Background UDP sink port", g_cross_port);

    cmd.AddValue("packet_size_bytes", "UDP payload size in bytes", g_packet_size_bytes);
    cmd.AddValue("flow_id", "FlowIdTag used by queue occupancy trace", g_flow_id);
    cmd.AddValue("udp_port", "UDP sink port", g_udp_port);

    cmd.AddValue("trace_interval_ms", "Rate trace sampling interval in ms", g_trace_interval_ms);
    cmd.AddValue("trace_path", "Output trace directory", g_trace_path);
    cmd.AddValue("trace_name", "Output trace file prefix", g_trace_name);
    cmd.Parse(argc, argv);

    RunScenario();
    return 0;
}
