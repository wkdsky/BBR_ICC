#pragma once
#include "ns3/proto_time.h"
#include "ns3/simulator.h"
namespace ns3{
class DqcSimuClock:public dqc::ProtoClock{
public:
    dqc::ProtoTime Now() const override{
        // ProtoTime is microsecond based; sub-ms BBRv2 probe-order bins need
        // the ns-3 simulator timestamp without millisecond truncation.
        int64_t us=Simulator::Now().GetMicroSeconds();
        dqc::ProtoTime current_ts=dqc::ProtoTime::Zero()+dqc::TimeDelta::FromMicroseconds(us);
        return current_ts;
    }
    dqc::ProtoTime ApproximateNow() const override{
        return Now();
    }   
};   
}
