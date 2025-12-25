#ifndef periodicDC_HH
#define periodicDC_HH

#undef SIMULATION_MODE

#include "ccc.hh"
#ifdef SIMULATION_MODE
#include "random.h"
#else
#include "random.hh"
#endif
#include "estimators.hh"
#include "rtt-window.hh"

#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <vector>

#include <fftw3.h>

using namespace std;
class IntroCC : public CCC {
  typedef double Time;
  typedef int SeqNum;
  
  double lamda;
  double Bd,defaultBd;
  double Rc;
  
  // Some adjustable parameters
  // This factor is normalizes w.r.t expected Newreno window size for
  // TCP cooperation
  static constexpr double alpha_rtt = 1.0 / 1.0;
  static constexpr int num_probe_pkts = 2;
  
  struct PacketData {
    Time sent_time;
    Time intersend_time;
    Time intersend_time_vel;
    Time rtt;
    double prev_avg_sending_rate;
  };
  
  std::map<SeqNum, PacketData> unacknowledged_packets;
  
  Time min_rtt,max_rtt,gloabMinRtt,Qop,RTTminop;
  // If min rtt is supplied externally, preserve across flows.
  double external_min_rtt;
  PlainEwma rtt_unacked;
  RTTWindow rtt_window;
  Time prev_intersend_time;
  Time cur_intersend_time;
  double ack_interval;
  double latest_ack;
  Percentile interarrival;
  IsUniformDistr is_uniform;
  
  Time P_ave,P_ave_pre,P_max,P_min;
  int P_cnt;
  Time RTT_per_sec;
  Time probe_cycle_start;

  Time last_update_time,last_dec_time;
  int update_dir,pre_update_dir;
  int pkts_per_rtt;
  int theta;//^double update_amt;
  
  int rtt_num;
  
  Time intersend_time_vel;
  Time prev_intersend_time_vel;
  double prev_rtt; // Measured RTT one-rtt ago.
  double prev_rtt_update_time;
  double prev_avg_sending_rate;
  double cur_rate;
  
  bool nearEmpty,probe_ceil,nearEmpty2,is_periodic,first_periodic,first_uncoord;
  bool if_search;
  double cycle;
  double fm,pre_fm,phase;
  double update_lamda_inter,upper_search_bound;
  int sec_cnt;
  double preAm0;
  
  vector <double> P_qd;
  vector <double> P_cwnd;
  vector <double> th_cur;
  
  
#ifdef SIMULATION_MODE
  RNG rand_gen;
#else
  RandGen rand_gen;
#endif
  int num_pkts_lost;
  int num_pkts_acked;
  int num_pkts_sent;
  
  // Variables for expressing explicit utility
  enum {CONSTANT_LAMDA,AUTO_LAMDA} utility_mode;
  enum {CONSTANT_THETA,AUTO_THETA,AUTO_THETA_D} theta_mode;
  bool do_slow_start;
  bool if_compete;
  double default_lamda,detectedLamda;
  int flow_length;
  // To cooperate with TCP, measured in fraction of RTTs with loss
  // Behavior constant
  PlainEwma interarrival_ewma;
  double prev_ack_time;
  bool slow_start;
  double slow_start_threshold;
  
  // Find flow id
  static int flow_id_counter;
  int flow_id;
  
  Time cur_tick;
  
  int rttInc,cwndInc;
  Time preRTT,preCwnd;
  
  double current_timestamp();
  
  double randomize_intersend(double);
  
  void update_intersend_time(Time RTTinst);
  
  void update_lamda(double Qamp,double gain,Time rtt);
  
  int find_fm();
  
public:
  void interpret_config_str(std::string config,std::string Bd_config,std::string Rc_config);
  
  IntroCC(double lamda)
    : lamda(lamda),
      Bd(20),
	  defaultBd(20),
      Rc(12),
      unacknowledged_packets(),
      min_rtt(),
	  max_rtt(),
	  gloabMinRtt(),
	  Qop(),
	  RTTminop(),
      external_min_rtt(false),
      rtt_unacked(alpha_rtt),
      rtt_window(),
      prev_intersend_time(),
      cur_intersend_time(),
	  ack_interval(0),
	  latest_ack(0),
      interarrival(0.05),
      is_uniform(32),
      P_ave(),
	  P_ave_pre(),
      P_max(),
      P_min(),
	  P_cnt(),
	  RTT_per_sec(),
      probe_cycle_start(),
      last_update_time(),
	  last_dec_time(),
      update_dir(),
	  pre_update_dir(),
      pkts_per_rtt(),
      theta(),
	  rtt_num(0),
      intersend_time_vel(),
      prev_intersend_time_vel(),
      prev_rtt(),
      prev_rtt_update_time(),
      prev_avg_sending_rate(),
	  cur_rate(-1),
      nearEmpty(),
	  probe_ceil(),
	  nearEmpty2(),
	  is_periodic(),
	  first_periodic(),
	  first_uncoord(),
      if_search(),
      cycle(),
      fm(),
      pre_fm(),
	  phase(),
	  update_lamda_inter(),
	  upper_search_bound(),
	  sec_cnt(1),
	  preAm0(),
      P_qd(),
	  P_cwnd(),
	  th_cur(),
      rand_gen(),
      num_pkts_lost(),
      num_pkts_acked(),
      num_pkts_sent(),
      utility_mode(AUTO_LAMDA),
	  theta_mode(AUTO_THETA),
      do_slow_start(false),
      if_compete(false),
      default_lamda(),
	  detectedLamda(),
	  flow_length(),
      interarrival_ewma(1.0 / 32.0),
      prev_ack_time(),
      slow_start(),
      slow_start_threshold(),
      flow_id(++ flow_id_counter),
      cur_tick(),
	  rttInc(0),
	  cwndInc(0),
	  preRTT(0),
	  preCwnd(0)
  {}
  
  // callback functions for packet events
  virtual void init() override;
  virtual void onACK(int ack, double receiver_timestamp, 
		     double sent_time, int delta_class=-1);
  virtual void onTimeout() override;
  virtual void onDupACK() override;
  virtual void onPktSent(int seq_num) override;
  void onTinyPktSent() {num_pkts_acked ++;}
  virtual void close() override;
  
  bool send_tiny_pkt() {return false;}//num_pkts_acked < num_probe_pkts-1;}
  
  //#ifdef SIMULATION_MODE
  void set_timestamp(double s_cur_tick) {cur_tick = s_cur_tick;}
  //#endif
  
  void set_flow_length(int s_flow_length) {flow_length = s_flow_length;}
  void set_min_rtt(double x) {
    if (external_min_rtt == 0)
      external_min_rtt = x;
    else
      external_min_rtt = std::min(external_min_rtt, x);
    init();
    std::cout << "Set min. RTT to " << external_min_rtt << std::endl;
  }
  int get_delta_class() const {return 0;}
  double get_the_window() {return _the_window;}
};

#endif
