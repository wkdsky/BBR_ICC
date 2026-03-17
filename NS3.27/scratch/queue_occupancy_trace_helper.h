#pragma once

#include "ns3/core-module.h"
#include "ns3/flow-id-tag.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"

#include <cerrno>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

namespace ns3 {

inline std::string&
QueueOccupancyTraceFolderStorage()
{
    static std::string folder;
    return folder;
}

inline std::string
NormalizeQueueTraceFolder(std::string folder)
{
    if (!folder.empty() && folder.back() != '/')
    {
        folder.push_back('/');
    }
    return folder;
}

inline std::string
DefaultQueueTraceFolder()
{
    char buf[FILENAME_MAX];
    memset(buf, 0, sizeof(buf));
    if (getcwd(buf, sizeof(buf)) == nullptr)
    {
        return "traces/";
    }
    return std::string(buf) + "/traces/";
}

inline void
EnsureDirectoryExists(const std::string& dir)
{
    if (dir.empty())
    {
        return;
    }

    std::string current = (dir[0] == '/') ? "/" : "";
    size_t pos = (dir[0] == '/') ? 1 : 0;

    while (pos < dir.size())
    {
        size_t next = dir.find('/', pos);
        std::string part = dir.substr(pos, next - pos);
        if (!part.empty())
        {
            if (!current.empty() && current.back() != '/')
            {
                current.push_back('/');
            }
            current += part;
            if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
            {
                return;
            }
        }
        if (next == std::string::npos)
        {
            break;
        }
        pos = next + 1;
    }
}

inline void
SetQueueOccupancyTraceFolder(const std::string& folder)
{
    QueueOccupancyTraceFolderStorage() = NormalizeQueueTraceFolder(folder);
}

inline std::string
GetQueueOccupancyTraceFolder()
{
    const std::string& folder = QueueOccupancyTraceFolderStorage();
    if (folder.empty())
    {
        return DefaultQueueTraceFolder();
    }
    return folder;
}

class QueueOccupancyTracer
{
  public:
    QueueOccupancyTracer(Ptr<Queue<Packet>> queue, const std::string& trace_name, uint32_t num_flows)
        : m_queue(queue),
          m_traceName(trace_name),
          m_numFlows(num_flows),
          m_flowBytes(num_flows + 1, 0),
          m_lastFlowBytes(num_flows + 1, 0)
    {
        if (m_queue == nullptr)
        {
            return;
        }

        std::string folder = GetQueueOccupancyTraceFolder();
        EnsureDirectoryExists(folder);

        m_stream.open((folder + m_traceName + "_bottleneck_queue.txt").c_str(), std::fstream::out);
        if (m_stream.is_open())
        {
            m_stream << "#time(s)\ttotal_bytes";
            for (uint32_t flowId = 1; flowId <= m_numFlows; ++flowId)
            {
                m_stream << "\tflow" << flowId << "_bytes_share";
            }
            m_stream << std::endl;
            WriteSample("init", true);
        }
    }

    void OnEnqueue(Ptr<const Packet> packet)
    {
        UpdateFlowBytes(packet, true);
        WriteSample("enqueue", false);
    }

    void OnDequeue(Ptr<const Packet> packet)
    {
        UpdateFlowBytes(packet, false);
        WriteSample("dequeue", false);
    }

  private:
    void UpdateFlowBytes(Ptr<const Packet> packet, bool enqueue)
    {
        if (packet == nullptr)
        {
            return;
        }

        FlowIdTag flowTag;
        if (!packet->PeekPacketTag(flowTag))
        {
            return;
        }

        uint32_t flowId = flowTag.GetFlowId();
        if (flowId == 0 || flowId > m_numFlows)
        {
            return;
        }

        uint32_t size = packet->GetSize();
        if (enqueue)
        {
            m_flowBytes[flowId] += size;
            return;
        }

        if (m_flowBytes[flowId] >= size)
        {
            m_flowBytes[flowId] -= size;
        }
        else
        {
            m_flowBytes[flowId] = 0;
        }
    }

    std::string FormatFlowBytesShare(uint32_t flowBytes, uint32_t totalBytes) const
    {
        double share = 0.0;
        if (totalBytes > 0)
        {
            share = 100.0 * static_cast<double>(flowBytes) / static_cast<double>(totalBytes);
        }

        std::ostringstream oss;
        oss << flowBytes << "(" << std::fixed << std::setprecision(2) << share << "%)";
        return oss.str();
    }

    void WriteSample(const char* event, bool force)
    {
        if (!m_stream.is_open() || m_queue == nullptr)
        {
            return;
        }

        uint32_t queueBytes = m_queue->GetNBytes();
        int64_t nowUs = Simulator::Now().GetMicroSeconds();

        if (!force && queueBytes == m_lastBytes && nowUs == m_lastTimeUs && m_flowBytes == m_lastFlowBytes)
        {
            return;
        }

        m_stream << Simulator::Now().GetSeconds() << "\t" << queueBytes;
        for (uint32_t flowId = 1; flowId <= m_numFlows; ++flowId)
        {
            m_stream << "\t" << FormatFlowBytesShare(m_flowBytes[flowId], queueBytes);
        }
        m_stream << std::endl;

        m_lastBytes = queueBytes;
        m_lastFlowBytes = m_flowBytes;
        m_lastTimeUs = nowUs;
    }

    Ptr<Queue<Packet>> m_queue;
    std::string m_traceName;
    uint32_t m_numFlows{0};
    std::fstream m_stream;
    uint32_t m_lastBytes{0};
    int64_t m_lastTimeUs{-1};
    std::vector<uint32_t> m_flowBytes;
    std::vector<uint32_t> m_lastFlowBytes;
};

inline std::shared_ptr<QueueOccupancyTracer>
InstallBottleneckQueueOccupancyTrace(Ptr<NetDevice> device, const std::string& trace_name, uint32_t num_flows)
{
    Ptr<PointToPointNetDevice> ptpDevice = DynamicCast<PointToPointNetDevice>(device);
    if (ptpDevice == nullptr)
    {
        return nullptr;
    }

    Ptr<Queue<Packet>> queue = ptpDevice->GetQueue();
    if (queue == nullptr)
    {
        return nullptr;
    }

    std::shared_ptr<QueueOccupancyTracer> tracer =
        std::make_shared<QueueOccupancyTracer>(queue, trace_name, num_flows);

    queue->TraceConnectWithoutContext("Enqueue",
                                      MakeCallback(&QueueOccupancyTracer::OnEnqueue, tracer.get()));
    queue->TraceConnectWithoutContext("Dequeue",
                                      MakeCallback(&QueueOccupancyTracer::OnDequeue, tracer.get()));
    return tracer;
}

} // namespace ns3
