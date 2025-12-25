#undef NDEBUG // We want the assert statements to work

#include "ICC.hh"

#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
// #define LDEBUG

#define packet_size 1500

using namespace std;

int IntroCC::flow_id_counter = 0;

double IntroCC::current_timestamp( void ){
  return cur_tick;
}

void IntroCC::init() {
  if (num_pkts_acked != 0) {
	  external_min_rtt=min_rtt;
    #ifdef LDEBUG
    std::cerr << "% Packets Lost: " << (100.0 * num_pkts_lost) /
      (num_pkts_acked + num_pkts_lost) << " at " << current_timestamp() << " " << num_pkts_acked << " " << num_pkts_sent << " min_rtt= " << min_rtt << " " << num_pkts_acked << " " << num_pkts_lost << " if compete: " << if_compete << endl;
    #endif
    if (slow_start)
      std::cerr << "Finished while in slow-start at window " << _the_window << endl;
  }
  else
  {
	  P_max=-1;
	  P_min=0x3f3f3f3f;
  }
  cur_rate=-1;// initial Rate by lhy 2020.11.18
  RTT_per_sec=0;
  _intersend_time = 0;
  _the_window = num_probe_pkts;
  rtt_window.clear();
  if (external_min_rtt != 0)
    _intersend_time = external_min_rtt / _the_window;
  if(external_min_rtt){//if (keep_ext_min_rtt)//^^
    min_rtt = external_min_rtt;
	if(default_lamda/min_rtt<0.01)default_lamda=0.01*min_rtt;//Add minimum default lamda = 1Mb/100ms by lhy 2020.10.13
	//^^
	rtt_window.new_rtt_sample(min_rtt, current_timestamp());
	//^^
  }
  else
    min_rtt = numeric_limits<double>::max();
  _timeout = 1000;//^_^
  
  if (utility_mode != CONSTANT_LAMDA)
    lamda = default_lamda;
  
  unacknowledged_packets.clear();

  rtt_unacked.reset();
  
  
  prev_intersend_time = current_timestamp();// initial  by lhy 2020.11.18
  cur_intersend_time = 0;// initial  by lhy 2020.11.18
  interarrival.reset();
  is_uniform.reset();

  
  last_update_time = last_dec_time = current_timestamp();// initial  by lhy 2020.11.18
  update_dir = 0;
  pre_update_dir = 0;
  pkts_per_rtt = 0;
  theta = 1;
  
  intersend_time_vel = 0;
  prev_intersend_time_vel = 0;
  prev_rtt = 0;
  prev_rtt_update_time = 0;
  prev_avg_sending_rate = 0;

  num_pkts_acked = num_pkts_lost = num_pkts_sent = 0;
  interarrival_ewma.reset();
  prev_ack_time = 0.0;
  slow_start = true;
  slow_start_threshold = 1e10;
  
  P_ave=0;
  P_ave_pre=0;
  P_cnt=0;
  
  nearEmpty=false;
  probe_ceil=false;
  is_periodic=false;
  first_periodic=true;
  first_uncoord=true;
  probe_cycle_start=0;
  
  cycle=1.0;
  
  P_qd.clear();
  P_cwnd.clear();
  th_cur.clear();
  
  fm=0;
  pre_fm=-1;
  
  rttInc=0;
  cwndInc=0;
  preCwnd=0;
  preRTT=0;
  
  max_rtt=-1;
  gloabMinRtt=0x3f3f3f3f;
  Qop=0x3f3f3f3f;
  RTTminop=0x3f3f3f3f;
  detectedLamda = default_lamda;
  cout << "init: " << "min_rtt= " << min_rtt << endl;
}

int IntroCC::find_fm()
{	
	int n=P_qd.size();
	double fs=1.0*n/cycle;
	double temp,qst=-1;
	rtt_num++;
	fftw_complex *outa,*ina;
	outa = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n);
	ina = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n);
	double *cwnd_amp = new double [n];
	double similarity=0;
	//cwnd fft
	for(int i=0;i<n;i++)
	{
		ina[i][0]=P_cwnd[i];
        ina[i][1]=0;
	}
	fftw_plan ppp=fftw_plan_dft_1d(n,ina,outa,FFTW_FORWARD,FFTW_ESTIMATE);
	fftw_execute(ppp);
	fftw_destroy_plan(ppp);
	double PP_ave=sqrt(outa[0][0]*outa[0][0]+outa[0][1]*outa[0][1]);
	cwnd_amp[0]=1;
	for(int i=1;i<n/2;i++)
	{
		cwnd_amp[i]=sqrt(outa[i][0]*outa[i][0]+outa[i][1]*outa[i][1])*2/PP_ave;
	}
	//Qd fft
	for(int i=0;i<n;i++)
	{
		ina[i][0]=P_qd[i];
        ina[i][1]=0;
	}
	fftw_plan pp=fftw_plan_dft_1d(n,ina,outa,FFTW_FORWARD,FFTW_ESTIMATE);
	fftw_execute(pp);
	fftw_destroy_plan(pp);
	PP_ave=sqrt(outa[0][0]*outa[0][0]+outa[0][1]*outa[0][1]);
	for(int i=1;i<n/2;i++)
	{
		temp=sqrt(outa[i][0]*outa[i][0]+outa[i][1]*outa[i][1]);
		if(qst<temp)
		{
			qst=temp;
			fm=i;
		}
		temp=temp*2/PP_ave;
		similarity+=fabs(temp-cwnd_amp[i]);
	}
	fftw_free(outa);
	fftw_free(ina);
	delete cwnd_amp;
	similarity/=n;
	fm=fm*fs/n;
	double simiD=abs(double(1.0*(rttInc-cwndInc)))/n;
	#ifdef LDEBUG
	std::cerr<< "Interval arrive: " << current_timestamp() <<" P_max "<<P_max<<" P_min "<<P_min<<" similarity: "<< similarity << " simiD: " << simiD << " rttInc "<<rttInc<<" cwndInc "<<cwndInc;
	#endif
	double fm_th=1/(20*min_rtt/1000);
	//if(fm > 1.5 && pre_fm > 1.5 && abs(pre_fm-fm)<=2)return 1;
	if(similarity > 0.02 || simiD>0.7)return -1;// 0.01->0.02 by lhy 2020.12.24
	if(fm > 0 && pre_fm > 0 && abs(pre_fm-fm)<=std::min(5*fm_th,2.0) ){ // FSNB
		//preAm0=PP_ave;// restrain stable state by lhy 2021.9.3
		return 1;// dynamic fm threhold by lhy : 2020.10.13
	}
	//preAm0=PP_ave;
	return 0;
}

void IntroCC::update_lamda(double Qamp,double gain,Time rtt){//bool pkt_lost __attribute((unused)), double cur_rtt) {
	if (utility_mode==AUTO_LAMDA)
	{
		
		
		if(first_periodic)
		{
			first_periodic=false;
			return ;
		}
		
		//lamda=max(min(gain*(P_ave-min_rtt)/Qamp*lamda, 10*lamda),0.1*lamda);// 0901
		Qamp=Qamp;
		if(nearEmpty){
			detectedLamda=lamda;
			lamda/=2;
		}
		else lamda+=gain*0.2*detectedLamda;
		rtt=rtt;
		
	}
}

double IntroCC::randomize_intersend(double intersend) {
  //return rand_gen.exponential(intersend);
  //return rand_gen.uniform(0.99*intersend, 1.01*intersend);
  return intersend;
}


void IntroCC::update_intersend_time(Time RTTinst) {
  double cur_time __attribute((unused)) = current_timestamp();
  // if (external_min_rtt == 0) {
  //   exit(1);
  // }
  
  
  // First two RTTs are for probing
  /* if (num_pkts_acked < 2 * num_probe_pkts - 1)
    return; */

  // Calculate useful quantities
  double rtt = rtt_window.get_unjittered_rtt();//rtt_window.get_unjittered_rtt();//rtt_window.get_srtt();
  //double queuing_delay = rtt>1.05*min_rtt?rtt - min_rtt:0;//add threshold for Qd by lhy 12.22
  double queuing_delay = rtt - min_rtt;
  
  // Calculate ack interval
  ack_interval = cur_time-latest_ack;
  latest_ack = cur_time;
  
  if(cur_rate<0)
  {
	  cur_rate = _the_window * (1.0*packet_size*8/1000) / min_rtt; // Mbps // calculate with packet_size by lhy 12.14
  }
  
  double target_rate;
  target_rate = Rc*log(Bd/queuing_delay);

  if (last_update_time + rtt_window.get_latest_rtt() < cur_time)
  {
      cout << "new_RTT_at: " << cur_time << " cur_dir= " << update_dir << " prev_v= " << theta;//^_^
      if ( (theta_mode == AUTO_THETA_D && update_dir * pre_update_dir > 0) || (theta_mode == AUTO_THETA && update_dir > 0 && pre_update_dir >0) )//update_dir*pre_update_dir >0)//update_dir > 0)
	  {
          theta *= 2;//^_^
      }
      else 
	  {
        theta = 1.;
      }
	  //theta = min((double)theta, _the_window/lamda); // ^^ to be modify
	  if(theta < 1) theta = 1; // add compete by lhy 2021.09.01
      cout<< " cur_v= " << theta << " window= " << _the_window << " target= " << target_rate << " rtt= " << rtt << endl;//^_^
      last_update_time = cur_time;
	  pre_update_dir = update_dir;
      pkts_per_rtt = update_dir = 0;
    }
    ++ pkts_per_rtt;

    if (cur_rate <= target_rate) {
      ++ update_dir;
      cur_rate += min(theta * lamda * ack_interval /  RTTinst, 9*cur_rate);
    }
    else {
      -- update_dir;
	  if(theta_mode == AUTO_THETA || pre_update_dir > 0)theta=1;
      cur_rate -= min(theta * lamda * ack_interval /  RTTinst, 0.5*cur_rate);
    }
  if(Bd==defaultBd)_the_window = max(2.0, cur_rate * min_rtt / (1.0*packet_size*8/1000));//^limit the min cwnd // calculate with packet_size by lhy 12.14
  else _the_window = max(2.0, cur_rate * rtt / (1.0*packet_size*8/1000));
  cur_rate = max(cur_rate, 0.2 * (1.0*packet_size*8/1000) / min_rtt);//add lower boundary for rate by lhy 12.23
  #ifdef LDEBUG
  std::cerr << rtt_num << ":time= " << cur_time << " window= " << _the_window << " target= " << target_rate << " rtt_standing= " << rtt << " min_rtt= " << min_rtt <<" queuing_delay= "<<queuing_delay<< " theta= " << theta << " lamda= " << lamda ;//^_^
  #endif
  // Set intersend time and perform boundary checks.
  cur_intersend_time = 1000 / (cur_rate * 1000 / (1.0*packet_size*8/1000));// calculate with packet_size by lhy 12.14
  _intersend_time = randomize_intersend(cur_intersend_time);
}

void IntroCC::onACK(int ack, 
			double receiver_timestamp __attribute((unused)),
			double sent_time, int delta_class __attribute((unused))) {
  int seq_num = ack - 1;
  double cur_time = current_timestamp();
  assert(cur_time > sent_time);
  rtt_window.new_rtt_sample(cur_time - sent_time, cur_time);
  if(Bd == defaultBd)min_rtt = rtt_window.get_min_rtt(); //min(min_rtt, cur_time - sent_time); // add compete by lhy 2021.09.01
  else min_rtt = min(min_rtt,cur_time - sent_time); // FSNB
  gloabMinRtt=std::min(min_rtt,gloabMinRtt);
  assert(rtt_window.get_unjittered_rtt() >= min_rtt);


  if (prev_ack_time != 0) {
    interarrival_ewma.update(cur_time - prev_ack_time, cur_time / min_rtt);
    interarrival.push(cur_time - prev_ack_time);
  }
  prev_ack_time = cur_time;
  //2020.11.12 use srtt
  Time rtt_measured=rtt_window.get_srtt();//rtt_window.get_unjittered_rtt();// use srtt, rtt, rttstanding
  
  if (probe_cycle_start < cur_time)
  {
	  if(rtt_measured >= preRTT)rttInc++;
	  if(_the_window > preCwnd)cwndInc++;
	  preRTT=rtt_measured;
	  preCwnd=_the_window;
	  P_cwnd.push_back(_the_window);
	  P_qd.push_back(rtt_measured);
	  P_min=min(P_min,cur_time - sent_time);
    P_min=min(P_min,rtt_measured);
	  P_max=max(P_max,rtt_measured);
	  P_ave+=(rtt_measured);
	  P_cnt++;
	  if(P_min<=min_rtt+1)nearEmpty=true;
	  if(P_max>=max_rtt-1)
	  {
		probe_ceil=true;
		//max_rtt=std::max(P_max,max_rtt);
	  }
  }
  
  if(max_rtt<rtt_measured) // 0908
  {
	max_rtt = rtt_measured;
	if(Bd > defaultBd && Qop != 0x3f3f3f3f)
	{
		Bd = std::max (1.0*defaultBd * (max_rtt - RTTminop)/Qop, defaultBd);
	}
  }
  
  bool pkt_lost = false;
  int it_cnt=0,it_tot=unacknowledged_packets.count(seq_num);
  //std::cerr<<"debug1: lossDetect1: "<<seq_num<< " "<<unacknowledged_packets.count(seq_num)<<" "<<unacknowledged_packets.size()<<" "<< unacknowledged_packets.begin()->first<<" "<< unacknowledged_packets.end()->first<<endl;
  if (unacknowledged_packets.count(seq_num) != 0) {
    int tmp_seq_num = -1;
    for (auto x : unacknowledged_packets) {
      //std::cerr<<"debug1: lossDetect: "<<tmp_seq_num<< " "<<x.first<<" "<<seq_num<<endl;
	  it_cnt++;
	  if(it_cnt>it_tot)break;//safe break by lhy 2020.12.08
      assert(tmp_seq_num <= x.first);
      tmp_seq_num = x.first;
      if (x.first > seq_num)break;
      prev_intersend_time = x.second.intersend_time;
      prev_intersend_time_vel = x.second.intersend_time_vel;
      prev_rtt = x.second.rtt;
      prev_rtt_update_time = x.second.sent_time;
      prev_avg_sending_rate = x.second.prev_avg_sending_rate;
      if (x.first < seq_num) {
        ++ num_pkts_lost;
        pkt_lost = true;
      }
	  unacknowledged_packets.erase(x.first);
    }
  }
  
  /* bool pkt_lost = false;
  std::cerr<<"debug1: lossDetect1: "<<seq_num<< " "<<unacknowledged_packets.count(seq_num)<<" "<<unacknowledged_packets.size()<<" "<< unacknowledged_packets.begin()->first<<" "<< unacknowledged_packets.end()->first<<endl;
  if (unacknowledged_packets.count(seq_num) != 0) {
    int tmp_seq_num = -1;
    for (std::map<SeqNum, PacketData>::iterator x=unacknowledged_packets.begin();x!=unacknowledged_packets.end();x++) {
      std::cerr<<"debug1: lossDetect: "<<tmp_seq_num<< " "<<x->first<<" "<<seq_num<<endl;
      assert(tmp_seq_num <= x->first);
      tmp_seq_num = x->first;
      if (x->first > seq_num)break;
      prev_intersend_time = x->second.intersend_time;
      prev_intersend_time_vel = x->second.intersend_time_vel;
      prev_rtt = x->second.rtt;
      prev_rtt_update_time = x->second.sent_time;
      prev_avg_sending_rate = x->second.prev_avg_sending_rate;
      if (x->first < seq_num) {
        ++ num_pkts_lost;
        pkt_lost = true;
      }
	  std::map<SeqNum, PacketData>::iterator xx=x;
	  x++;
	  unacknowledged_packets.erase(xx);
    }
  } */
  
  //if(pkt_lost)Bd=defaultBd;
  
  if (probe_cycle_start+cycle*1000 < cur_time)
  {
	  probe_cycle_start=cur_time;
	  P_ave/=P_cnt;
	  int period_state=find_fm();
	  is_periodic = false;
	  if(!P_qd.empty() && period_state > 0 && P_ave>=0.9*preAm0 && P_ave<=1.1*preAm0)// FSNB
	  {
		if(first_uncoord){
			//update_lamda();
			is_periodic = true;
			cycle=min(20*min_rtt/1000,5/fm); // 0908 //10->2 2020.09.12
			if(Bd==defaultBd)
			{
				Qop = min(Qop,P_ave-min_rtt); // FSNB
				RTTminop = min_rtt;
				//RTTminop = m_RTTmax.GetSeconds();
			}
			Bd=defaultBd;
		}
		first_uncoord = true;
		//rtt_window.change_window(false);
	  }
	  else if(if_compete && (P_ave>P_min && P_ave-P_min>2*Bd) &&  period_state < 0 && P_min>2*Bd+gloabMinRtt)// compete
	  {
		  //rtt_window.change_window(true);
		  cycle=20*min_rtt/1000;//2020.09.12
		  //if(!first_uncoord)Bd = std::max (2 * (P_max - min_rtt), defaultBd);
		  
		  
		  // old scheme at 0907
		  if(Qop!=0x3f3f3f3f)Bd = std::max (1.0*defaultBd * (max_rtt - RTTminop)/Qop, defaultBd); //FSNB
		  else Bd = std::max (10.0* (max_rtt - min_rtt), defaultBd);
		  
		  // new scheme at 0907
		  //Bd = 0x3f3f3f3f;
		  
		  first_uncoord = false;
		  #ifdef LDEBUG
		  //std::cerr<<" debug:compete at "<< cur_time << " P_max " <<P_max << " RTTminop " << RTTminop << " Qop "<< Qop << " Bd " << Bd <<endl;
		  #endif
		  //cycle=1.0;
	  }
	  else
	  {
		  cycle=20*min_rtt/1000;//2020.09.12
		  //first_uncoord = true;
		  //Bd = defaultBd;
		  //cycle=1.0;
		  //rtt_window.change_window(false);
		  //lamda=default_lamda;
	  }
	  preAm0 = P_ave;
	  double Qamp=0;
	  if(!(nearEmpty || probe_ceil))P_ave_pre=P_ave;// by lhy 2020.10.13
	  Qamp=std::max(P_max-P_ave_pre,P_ave_pre-P_min);
	  if((P_max-P_min)/2>Qamp || P_ave_pre==0)
	  {
		  Qamp=(P_max-P_min)/2;
	  }
	  else if(is_periodic)
	  {
		  //if(nearEmpty || probe_ceil)P_ave=P_ave_pre;// update Qave more frequent in safe region by lhy 2020.09.30
		  P_ave=P_ave_pre;
	  }
	  
	  double gain=1;
	  //if(P_ave_pre!=0)gain=(P_ave_pre-min_rtt)/Qamp;
	  #ifdef LDEBUG
	  std::cerr<<" P_ave "<<P_ave<<" P_ave_pre "<<preAm0 << " Qamp " << Qamp << " gain "<< gain << " RTT_max " << max_rtt << " if_safe " << nearEmpty << probe_ceil << " C/N " << lamda*(cur_time - sent_time)/Qamp << " Bd " << Bd <<" first_uncoord "<<first_uncoord<<" fm "<<fm <<" lamda "<<lamda << " Qop " << Qop << " RTTminop " << RTTminop <<" ifstable " << is_periodic <<" detectedLamda " << detectedLamda <<endl;
	  #endif
	  if(is_periodic && !first_periodic)Bd=defaultBd;
	  if(is_periodic){// && std::abs(Qamp-(P_ave-min_rtt))>0.01*min_rtt){ // 0908
		  update_lamda(Qamp,gain,cur_time - sent_time);// add dead zone 2020.09.12
	  }
	  else
	  {
		cycle=20*min_rtt/1000;
		lamda = detectedLamda;
		//lamda=default_lamda;
		first_periodic=true;
	  }
	  P_ave_pre=P_ave;
	  pre_fm=fm;
	  P_qd.clear();
	  P_cwnd.clear();
	  P_min=0x3f3f3f3f;
	  P_max=-1;
	  P_ave=0;
	  //P_cnt=0;
	  rttInc=0;
	  cwndInc=0;
	  nearEmpty=false;
	  probe_ceil=false;
	  //slow down when serious loss by lhy 2020.12.09
	  //if((double)(1.0*num_pkts_lost/P_cnt)>0.1)cur_rate*=0.5;
	  num_pkts_lost=0;
	  P_cnt=0;
  }
  update_intersend_time(cur_time - sent_time);
  Time rttstanding = rtt_window.get_unjittered_rtt();
  if(Bd>defaultBd && pkt_lost && cur_time-last_dec_time > rttstanding)
  {
	    //theta=1;
		cur_rate*=0.7;
		_the_window = max(2.0, cur_rate * rttstanding / (1.0*packet_size*8/1000));
		cur_rate = max(cur_rate, 0.2 * (1.0*packet_size*8/1000) / min_rtt);
		cur_intersend_time = 1000 / (cur_rate * 1000 / (1.0*packet_size*8/1000));
		_intersend_time = randomize_intersend(cur_intersend_time);
		last_dec_time = cur_time;
  }
  
  #ifdef LDEBUG
  std::cerr << " RTT_true= " << cur_time - sent_time << " srtt= " << rtt_window.get_srtt() << " fm= " << pre_fm ;//^_^
  #endif

  
  
  RTT_per_sec+=(cur_time - sent_time);
  th_cur.push_back(cur_time);
  if(cur_time > sec_cnt*1000)
  {
	  cout<< "\ndebug1:sec: " << sec_cnt << " throughput: " << (1.0*th_cur.size()*packet_size*1000*8/(cur_time-th_cur[0])/1024/1024) << " RTT: " << RTT_per_sec/th_cur.size() << endl;
	  sec_cnt++;
	  RTT_per_sec=0;
  }
  while(th_cur[0] < cur_time-1000)
  {
	  th_cur.erase(th_cur.begin());
  }
  #ifdef LDEBUG
  std::cerr<< " throughput= "<< (double)(1.0*th_cur.size()*packet_size*1000*8/(cur_time-th_cur[0])/1024/1024) << " sendingrate= " << cur_rate << " acked= " << seq_num << " ack_interval= "<< ack_interval << " step= " << theta * lamda * ack_interval /  (cur_time - sent_time) <<endl;
  #endif
  ++ num_pkts_acked;
  // if (pkt_lost) {
  // }
}

void IntroCC::onPktSent(int seq_num) {
  double cur_time = current_timestamp();
  
  unacknowledged_packets[seq_num] = {cur_time,
                                     cur_intersend_time,
                                     intersend_time_vel,
                                     rtt_window.get_unjittered_rtt(),
                                     unacknowledged_packets.size() / (cur_time - prev_rtt_update_time)
  };

  rtt_unacked.force_set(rtt_window.get_unjittered_rtt(), cur_time / min_rtt);
  for (auto & x : unacknowledged_packets) {
    if (cur_time - x.second.sent_time > rtt_unacked) {
      rtt_unacked.update(cur_time - x.second.sent_time, cur_time / min_rtt);
      prev_intersend_time = x.second.intersend_time;
      prev_intersend_time_vel = x.second.intersend_time_vel;
      continue;
    }
    break;
  }
  //update_intersend_time();
  ++ num_pkts_sent;
  _intersend_time = randomize_intersend(cur_intersend_time);
  #ifdef LDEBUG
	std::cerr<<"debug1: PacingPkt: "<<seq_num<< " at "<<cur_time<< " intersend_time: "<< _intersend_time<< " cur_rate "<<cur_rate<<" cwnd "<<_the_window<<" rttmin "<<min_rtt<< endl;
  #endif
}

void IntroCC::close() {
}

void IntroCC::onDupACK() {

}

void IntroCC::onTimeout() {

}

void IntroCC::interpret_config_str(string config, string Bd_config, string Rc_config) { //config

  if (config.substr(0, 6) == "do_ss:") {
    do_slow_start = true;
    config = config.substr(6, string::npos);
    cout << "Will do slow start" << endl;
  }
  if (config.substr(0, 8) == "compete:") {
    if_compete = true;
    config = config.substr(8, string::npos);
    cout << "Using Compete Mode" << endl;
  }
  if (config.substr(0, 11) == "auto_theta:") {
    theta_mode = AUTO_THETA;
    config = config.substr(11, string::npos);
    cout << "Using Automatic Theta Mode" << endl;
  }
  else if (config.substr(0, 13) == "auto_theta_d:") {
    theta_mode = AUTO_THETA_D;
    config = config.substr(13, string::npos);
    cout << "Using Automatic Theta (Double direction) Mode" << endl;
  }
  else if(config.substr(0, 15) == "constant_theta:"){
	theta_mode = CONSTANT_THETA;
    config = config.substr(15, string::npos);
    cout << "Using Constant Theta Mode" << endl;
  }

  lamda = 1; 
  if (config.substr(0, 15) == "constant_lamda:") {
    utility_mode = CONSTANT_LAMDA;
    default_lamda = atof(config.substr(15, string::npos).c_str());
	lamda = default_lamda;
    cout << "Constant lamda mode with lamda = " << lamda << endl;
  }
  else if (config.substr(0, 5) == "auto:") {
    utility_mode = AUTO_LAMDA;
    default_lamda = atof(config.substr(5, string::npos).c_str());
    lamda = default_lamda;
    cout << "Using Automatic Mode with default lamda = " << default_lamda << endl;
  }
  else {
    utility_mode = CONSTANT_LAMDA;
    lamda = 1;
    cout << "Incorrect format of configuration string '" << config
	 << "'. Using constant lamda mode with lamda = 1 by default" << endl;
  }
  Bd = atof(Bd_config.substr(0, string::npos).c_str());
  defaultBd = Bd;
  Rc = atof(Rc_config.substr(0, string::npos).c_str());
  cout << "Set Bd= " << Bd << "ms" << endl;
  cout << "Set_Rc= " << Rc << "mbps" << endl;
  
}
