
/*
 * tcp-PeriodicDC.h
 *
 *  Created on: 12 23 2018
 *      Author: root
 */


#ifndef TcpPeriodicDC_H
#define TcpPeriodicDC_H

#include "ns3/tcp-congestion-ops.h"
#include <vector>
#include <deque>
using namespace std;

namespace ns3 {

/**
 * \ingroup congestionOps
 *
 * \brief An implementation of TCP Vegas
 *
 * TCP Vegas is a pure delay-based congestion control algorithm implementing a proactive
 * scheme that tries to prevent packet drops by maintaining a small backlog at the
 * bottleneck queue.
 *
 * Vegas continuously measures the actual throughput a connection achieves as shown in
 * Equation (1) and compares it with the expected throughput calculated in Equation (2).
 * The difference between these 2 sending rates in Equation (3) reflects the amount of
 * extra packets being queued at the bottleneck.
 *
 *              actual = cwnd / RTT        (1)
 *              expected = cwnd / BaseRTT  (2)
 *              diff = expected - actual   (3)
 *
 * To avoid congestion, Vegas linearly increases/decreases its congestion window to ensure
 * the diff value fall between the 2 predefined thresholds, alpha and beta.
 * diff and another threshold, gamma, are used to determine when Vegas should change from
 * its slow-start mode to linear increase/decrease mode.
 *
 * Following the implementation of Vegas in Linux, we use 2, 4, and 1 as the default values
 * of alpha, beta, and gamma, respectively.
 *
 * More information: http://dx.doi.org/10.1109/49.464716
 */

class TcpPeriodicDC : public TcpNewReno
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * Create an unbound tcp socket.
   */
  TcpPeriodicDC (void);

  /**
   * \brief Copy constructor
   * \param sock the object to copy
   */
  TcpPeriodicDC (const TcpPeriodicDC& sock);
  virtual ~TcpPeriodicDC (void);

  virtual std::string GetName () const;

  /**
   * \brief Compute RTTs needed to execute Vegas algorithm
   *
   * The function filters RTT samples from the last RTT to find
   * the current smallest propagation delay + queueing delay (minRtt).
   * We take the minimum to avoid the effects of delayed ACKs.
   *
   * The function also min-filters all RTT measurements seen to find the
   * propagation delay (baseRtt).
   *
   * \param tcb internal congestion state
   * \param segmentsAcked count of segments ACKed
   * \param rtt last RTT
   *
   */
  virtual void Send(Ptr<TcpSocketBase> tsb, Ptr<TcpSocketState> tcb,
                      SequenceNumber32 seq, bool isRetrans);
  virtual void PktsAcked (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked,
                          const Time& rtt);
  virtual void PktsAckedAPeriodicDC (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked,
                            const Time& rtt);

  /**
   * \brief Enable/disable Vegas algorithm depending on the congestion state
   *
   * We only start a Vegas cycle when we are in normal congestion state (CA_OPEN state).
   *
   * \param tcb internal congestion state
   * \param newState new congestion state to which the TCP is going to switch
   */
  virtual void CongestionStateSet (Ptr<TcpSocketState> tcb,
                                   const TcpSocketState::TcpCongState_t newState);

  /**
   * \brief Adjust cwnd following Vegas linear increase/decrease algorithm
   *
   * \param tcb internal congestion state
   * \param segmentsAcked count of segments ACKed
   */
  virtual void IncreaseWindow (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked);
  virtual void ExitRecovery (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked) ;

  virtual void PeriodicDCModifyWindow (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked);

  /**
   * \brief Get slow start threshold following Vegas principle
   *
   * \param tcb internal congestion state
   * \param bytesInFlight bytes in flight
   *
   * \return the slow start threshold value
   */
  virtual uint32_t GetSsThresh (Ptr<const TcpSocketState> tcb,
                                uint32_t bytesInFlight);

  virtual Ptr<TcpCongestionOps> Fork ();
  
  void DecreaseWindow(Ptr<TcpSocketState> tcb);
  
  static void find_fm_row(double *fm,double *am,double *qavr,double T,vector<double> &pr);
  void find_fm(double *fm,double *am,double *f0am,vector<double> &s,double fs,vector<double> &c,double *simi);

protected:
private:
  /**
   * \brief Enable Vegas algorithm to start taking Vegas samples
   *
   * Vegas algorithm is enabled in the following situations:
   * 1. at the establishment of a connection
   * 2. after an RTO
   * 3. after fast recovery
   * 4. when an idle connection is restarted
   *
   * \param tcb internal congestion state
   */
  void EnablePeriodicDC (Ptr<TcpSocketState> tcb);

  /**
   * \brief Stop taking Vegas samples
   */
  void DisablePeriodicDC ();

private:
  double m_lamuda;                   //!< the target numble of packet queue in the buffer
  double default_lamuda,detectedLamda;                   //2020.7.04 by lhy
  double cur_Rate;					// current pacing rate (Mbps) 2020.7.14 by lhy
  double cwnd_gain;
  double ack_interval;				//Tau 2020.7.14 by lhy
  Time pre_ack_time;				
  double m_theta;                  //!< velocity parameter for increase
  Time m_srtt;	                      // smooth rtt
  uint32_t m_cntRtt;                 //!< Number of RTT measurements during last RTT
  bool m_doingPeriodicDCNow;              //!< If true, do PeriodicDC for this RTT
  SequenceNumber32 m_begSndNxt;      //!< Right edge during last RTT

  Time m_RTTmin;
  Time recordTimeMin;
  Time m_RTTstanding;
  Time m_oldRTTstanding;
  Time recordTimeStanding;

  double oldCwndpkts;
  double newCwndpkts;
  double oldAckTime;
  double oldAckSeq;

  double pra;

  std::map<SequenceNumber32,uint32_t> cwndHistory;
  std::map<SequenceNumber32,Time> ackTimeHistory;
  std::map<SequenceNumber32,SequenceNumber32> ackSeqHistory;
  std::vector<double> QdArray;
  std::vector<double> CwndArray;

  int QdInc,CwndInc;
  double old_Qd,old_Cwnd;//2020.8.14 by lhy
  
  Time oldRtt1;
  Time oldRtt2;

  bool oldDirect;
  bool newDirect;
  uint32_t times;
  bool isFirstTime;

  bool isPeriodicDCModify;
  bool stable_state;
  bool first_periodic;
  Time Qtarget;

  bool isSlowStart;

  Time m_RTTmax,glo_RTTmax;
  double RTTminop,Qop;
  bool ifcompete;
  bool tempEmpty;
  bool defaultMode;
  uint32_t RTTCountForEmpty;
  uint32_t RTTCountForMax;
  double RTTHistory[4]={0,0,0,0};
  bool emptyHistory[5]={false,false,false,false,false};
  Time tempMax;
  Time tempMin;
  bool nearEmpty,probe_ceil;
  double assitPra;
  double Epra;
  double oldEpra;
  double v0;
  uint32_t RTTCountFormin;
  double NEpra;
  uint32_t RTTCount;
  bool RTTdirect;
  bool isBiTheta;
  double ssTheta;
  double sstarget;
  double Smax;
  Time recordTimeEmpty;
  bool directChange;
  uint32_t RTTforCountDirectChange;
  uint32_t ForEmpty;
  double oldCwndMid;
  Time m_RTTq;
  uint32_t ackCount;
  double intervalPoint;
  double srttPoint;
  double sum;
  double targetRate;
  double Bd;
  double defaultBd;
  double Rc;
  double cycle;
  bool isDC,isCom;//2020.7.05 by lhy
  double default_cycle;//2020.7.04 by lhy
  double oldFm;
  double CwndList[3]={1,1,1};
  double UpDirectCount=0;
  std::deque<std::pair<int64_t, ns3::Time>> vals;
  double pre_Qdave;
  double curAm0,preAm0;
  Time pktLostTime;
};

} // namespace ns3

#endif // TcpPeriodicDC_H

