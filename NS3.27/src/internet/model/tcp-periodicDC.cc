/*
 * tcp-PeriodicDC.cc
 *
 *  Created on: 12 23 2018
 *      Author: root
 */
 
 //update 'RTT reaction' : 2020.6.15 by lhy
 //update default lamda and cycle, correct theta regulation : 2020.7.04 by lhy
 //add switch for running on datacenter : 2020.7.05 by lhy
 //ver 1.0	based on Current Rate ------ 2020.7.14 by lhy
 //ver 1.1	Modify RTT changes and no-PDCC flows detection with both time and frequency domain : 2020.8.14 by lhy
 /*
Update by lhy 2020.10.15
1. Add dynamic fm threshold
2. Add dynamic RTTmin time window
3. Move safe region judgement before Qamp updating
*/
 /*
Update by lhy 2020.11.11
1. Add adaptive Bd
*/

#include "tcp-periodicDC.h"
#include "ns3/tcp-socket-base.h"
#include "ns3/log.h"
#include <cmath>
#include <fftw3.h>
#include <chrono>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("TcpPeriodicDC");
NS_OBJECT_ENSURE_REGISTERED (TcpPeriodicDC);

TypeId
TcpPeriodicDC::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpPeriodicDC")
    .SetParent<TcpNewReno> ()
    .AddConstructor<TcpPeriodicDC> ()
    .SetGroupName ("Internet")
    .AddAttribute ("m_lamuda", "the target packt queue in the buffer",
                   DoubleValue (2),
                   MakeDoubleAccessor (&TcpPeriodicDC::m_lamuda),
                   MakeDoubleChecker<double> ())
	.AddAttribute ("cwnd_gain", "coverting gain of cwnd and rate",
                   DoubleValue (1),
                   MakeDoubleAccessor (&TcpPeriodicDC::cwnd_gain),
                   MakeDoubleChecker<double> ())
   .AddAttribute ("Bd", "the target packt queue in the buffer",
				  DoubleValue (10),
				  MakeDoubleAccessor (&TcpPeriodicDC::Bd),
				  MakeDoubleChecker<double> ())
  .AddAttribute ("Rc", "the target packt queue in the buffer",
				 DoubleValue (30),
				 MakeDoubleAccessor (&TcpPeriodicDC::Rc),
				 MakeDoubleChecker<double> ())
   .AddAttribute ("isPeriodicDCModify", "the mode of adjusting cwnd PeriodicDC or PeriodicDCModify",
		   	   	      BooleanValue (false),
					  MakeBooleanAccessor (&TcpPeriodicDC::isPeriodicDCModify),
					  MakeBooleanChecker ())
   .AddAttribute ("Qtarget", "the mode of adjusting cwnd PeriodicDC or PeriodicDCModify",
					  TimeValue (Seconds(0.0024)),
					  MakeTimeAccessor (&TcpPeriodicDC::Qtarget),
					  MakeTimeChecker ())
   .AddAttribute ("assitPra", "the asist pramete ",
					 DoubleValue (10),
					 MakeDoubleAccessor (&TcpPeriodicDC::assitPra),
					 MakeDoubleChecker<double> ())
   .AddAttribute ("cycle", "cycle ",
					 DoubleValue (3.0),
					 MakeDoubleAccessor (&TcpPeriodicDC::cycle),
					 MakeDoubleChecker<double> ())
	.AddAttribute ("isDC", "If works in datacenter ",
					 BooleanValue (false),
					 MakeBooleanAccessor (&TcpPeriodicDC::isDC),
					 MakeBooleanChecker())
	.AddAttribute ("isCom", "If works in datacenter ",
					 BooleanValue (false),
					 MakeBooleanAccessor (&TcpPeriodicDC::isCom),
					 MakeBooleanChecker ())
  ;
  return tid;
}

TcpPeriodicDC::TcpPeriodicDC (void)
  : TcpNewReno (),
    m_lamuda (1),
	default_lamuda (1),
	detectedLamda (1),
	cur_Rate(-1),
	cwnd_gain(1),
	ack_interval(0.0),
	pre_ack_time(Time (0.0)),
    m_theta(1),
	m_srtt (Time::Max ()),
    m_cntRtt (0),
    m_doingPeriodicDCNow (true),
    m_begSndNxt (0),
	m_RTTmin (Time::Max ()),
	recordTimeMin (Time (0.0)),
	m_RTTstanding (Time::Max ()),
	m_oldRTTstanding(Time::Max ()),
	recordTimeStanding (Time (0.0)),
	oldCwndpkts(1.0),
	newCwndpkts(1.0),
	oldAckTime(0),
	oldAckSeq(0),
	pra(1),
	QdInc(0),
	CwndInc(0),
	old_Qd(0),
	old_Cwnd(0),
	oldRtt1(Time::Min ()),
	oldRtt2(Time::Min ()),
	oldDirect(true),
	newDirect(true),
	times(0),
	isFirstTime(true),
	isPeriodicDCModify(false),
	stable_state(false),
	first_periodic(true),
	Qtarget(Time(Seconds(0.0024))),
	isSlowStart(true),
	m_RTTmax(Time (0.0)),
	glo_RTTmax(Time (0.0)),
	RTTminop(0),
	Qop(0),
	ifcompete(false),
	tempEmpty(false),
	defaultMode(true),
	RTTCountForEmpty(0),
 	RTTCountForMax(0),
	tempMax(Time (0.0)),
	tempMin(Time::Max ()),
	nearEmpty(true),
	probe_ceil(true),
	assitPra(0),
	Epra(2),
	oldEpra(1),
	v0(1),
	RTTCountFormin(0),
	NEpra(1),
	RTTCount(0),
   isBiTheta(false),
   ssTheta(1),
   sstarget(5),
   Smax(8),
   recordTimeEmpty (Time (0.0)),
   directChange(false),
   RTTforCountDirectChange(0),
   ForEmpty(2),
   oldCwndMid(1.0),
   m_RTTq(Time::Max ()),
   ackCount(0),
   intervalPoint(0.0),
   srttPoint(0.0),
   sum(0.0),
   targetRate(0),
    Bd(10),
	defaultBd(10),
    Rc(6),
    cycle(3.0),
	isDC(false),
	default_cycle(3.0),
	oldFm(-500),
	vals(),
	pre_Qdave(0),
	curAm0(0),
	preAm0(0),
        pktLostTime(Time (0.0))
{
  NS_LOG_FUNCTION (this);
}

TcpPeriodicDC::TcpPeriodicDC (const TcpPeriodicDC& sock)
  : TcpNewReno (sock),
	m_lamuda (sock.m_lamuda),
	default_lamuda (1),
	detectedLamda (1),
	cur_Rate(-1),
	cwnd_gain(1),
	ack_interval(0.0),
	pre_ack_time(Time (0.0)),
	m_theta (sock.m_theta),
	m_srtt (sock.m_srtt),
    m_cntRtt (sock.m_cntRtt),
	m_doingPeriodicDCNow (true),
    m_begSndNxt (0),
	m_RTTmin (Time::Max ()),
	recordTimeMin (Time (0.0)),
	m_RTTstanding (Time::Max ()),
	m_oldRTTstanding(Time::Max ()),
	recordTimeStanding (Time (0.0)),
	oldCwndpkts(1.0),
	newCwndpkts(1.0),
	oldAckTime(0),
	oldAckSeq(0),
	pra(1),
	QdInc(0),
	CwndInc(0),
	old_Qd(0),
	old_Cwnd(0),
	oldRtt1(Time::Min()),
	oldRtt2(Time::Min()),
	oldDirect(true),
	newDirect(true),
	times(0),
	isFirstTime(true),
	isPeriodicDCModify(false),
	stable_state(false),
	first_periodic(true),
	Qtarget(Time(Seconds(0.0024))),
	isSlowStart(true),
	m_RTTmax(Time (0.0)),
	glo_RTTmax(Time (0.0)),
	RTTminop(0),
	Qop(0),
	ifcompete(false),
	tempEmpty(false),
	defaultMode(true),
	RTTCountForEmpty(0),
 	RTTCountForMax(0),
	tempMax(Time (0.0)),
	nearEmpty(true),
	probe_ceil(true),
	assitPra(0),
	Epra(4),
	oldEpra(4),
	v0(1),
	RTTCountFormin(0),
	NEpra(1),
	RTTCount(0),
   isBiTheta(false),
   ssTheta(1),
   sstarget(5),
   Smax(4),
   recordTimeEmpty (Time (0.0)),
   directChange(false),
   RTTforCountDirectChange(0),
   ForEmpty(2),
   oldCwndMid(1.0),
   m_RTTq(Time::Max ()),
   ackCount(0),
   intervalPoint(0.0),
   srttPoint(0.0),
   sum(0.0),
   targetRate(0),
   cycle(3.0),
   isDC(false),
   default_cycle(3.0),
   oldFm(-500),
   vals(),
   pre_Qdave(0),
   curAm0(0),
	preAm0(0)
{
  NS_LOG_FUNCTION (this);
}

TcpPeriodicDC::~TcpPeriodicDC (void)
{
  NS_LOG_FUNCTION (this);
}

Ptr<TcpCongestionOps>
TcpPeriodicDC::Fork (void)
{
  return CopyObject<TcpPeriodicDC> (this);
}

void TcpPeriodicDC::Send(Ptr<TcpSocketBase> tsb, Ptr<TcpSocketState> tcb,
                  SequenceNumber32 seq, bool isRetrans) {

  NS_LOG_FUNCTION(this);



  // If retransmission, start sequence (PktsAcked() finds end of sequence).
  if (isRetrans) {
    NS_LOG_LOGIC(this << "  Starting retrans sequence: " << seq);
  }else {

	  cwndHistory.insert(std::pair<SequenceNumber32,uint32_t>(tcb->m_nextTxSequence,tcb->m_cWnd));
	  ackTimeHistory.insert(std::pair<SequenceNumber32,Time>(tcb->m_nextTxSequence,Simulator::Now ()));
	  ackSeqHistory.insert(std::pair<SequenceNumber32,SequenceNumber32>(tcb->m_nextTxSequence,tcb->m_lastAckedSeq));
  }
  NS_LOG_DEBUG("debug:Pacing "<< this <<" now: "<< (double)1.0*Simulator::Now ().GetInteger()/1000000000 << " send_seq " << seq << " Pacing_pkts "<< tsb->pacingQueueBytes()/tcb->m_segmentSize<<" Inflight "<<tsb->BytesInFlight()/tcb->m_segmentSize << " Pacing_rate "<< tcb->GetPacingRate() << " now(ns) " <<  Simulator::Now ().GetInteger() % 1000000 << " with "<< tcb->m_congState << " " << isRetrans);
}

void
TcpPeriodicDC::PktsAcked (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked,
                     const Time& rtt)
{
	NS_LOG_FUNCTION (this << tcb << segmentsAcked << rtt);
	//NS_LOG_INFO(" PktsAcked "<<tcb->m_cWnd );
//	std::chrono::high_resolution_clock::time_point beginTime = std::chrono::high_resolution_clock::now();



	if (rtt.IsZero ())
	{
		return;
	}
	if(Time::Min()==oldRtt2){
		//pktLostTime = Simulator::Now();
		oldRtt2=Simulator::Now ();
		oldRtt1=Simulator::Now ();
		nearEmpty=false;
		
	}
	Time RTT=this->GetrttInst();
	//Time RTT=min(this->GetrttInst(),rtt);
	//Time RTT=rtt;
	//init lamda 2020.7.04 by lhy
	if(oldFm==-500)
	{
		default_cycle=cycle;
		default_lamuda=m_lamuda;
		detectedLamda = default_lamuda;
		defaultBd=Bd;
		if(cur_Rate<0)// init Rate 2020.7.14 by lhy
		{
			cur_Rate = tcb->m_cWnd*8 / RTT.GetSeconds();// in bps
			cur_Rate /=1000000; // in Mbps
		}
	}
	//2020.7.04 by lhy

        double Bdelay=Bd*0.001;// convert to (s)
        double Bw=Rc*100/12;// convert to (Mbps)

	int64_t nowToInt = Simulator::Now ().GetInteger();
	//int64_t recordTimeMinToInt = recordTimeMin.GetInteger();

	//double cycle=3.0;
	//double Qd = m_RTTstanding.GetSeconds() - m_RTTmin.GetSeconds();
	double Qd = RTT.GetSeconds() - m_RTTmin.GetSeconds(); // use instantaneous RTT to caculate Qd
	Qd = std::max(0.0,Qd);

	//judge packet loss
	/*(bool pkt_loss=false;
	SequenceNumber32 L_seq=tcb->m_lastAckedSeq;
	if(ackTimeHistory.count(L_seq) != 0)
	{
		for(auto x : ackTimeHistory)
		{
			if (x.first > L_seq) break;
			if (x.first < L_seq) pkt_loss=true;
			ackTimeHistory.erase(x.first);
		}
	} */
	
	//update RTTmin
	Time tempRTT = RTT;
	while(!vals.empty() && vals.back().second > tempRTT)vals.pop_back();
	vals.push_back(make_pair(nowToInt,tempRTT));
	while(vals.size()>1 && (vals.front().first + std::max(Seconds (assitPra).GetInteger(),int64_t(100*m_RTTmin.GetSeconds()))) < nowToInt)vals.pop_front();// dynamic RTTmin time window by lhy 2020.10.15
	m_RTTmin=std::min(vals.front().second,m_RTTmin);
	
	
	/* if(nowToInt > recordTimeMinToInt + Seconds (assitPra).GetInteger()) {
		m_RTTmin = Time::Max ();
	}
	if (RTT <= m_RTTmin) {
		recordTimeMin = Simulator::Now ();

	} */
	
	
	//m_RTTmin = std::min (m_RTTmin, RTT);
   sum=sum+RTT.GetSeconds();

   ackCount++;
   /* if(tcb->m_lastAckedSeq.GetValue()/1448.0 <=2){
	   //m_lamuda=6/m_RTTmin.GetSeconds();
	   
   } */
	
	//update Tau
	ack_interval = std::min(Simulator::Now ().GetSeconds()-pre_ack_time.GetSeconds(),10*m_RTTmin.GetSeconds());// 0829
	pre_ack_time = Simulator::Now ();
	
   //record Qd, Cwnd information
   if(Simulator::Now ().GetSeconds()>=oldRtt2.GetSeconds()){
	   if(Qd>=old_Qd)QdInc++;
	   if(tcb->m_cWnd>old_Cwnd)CwndInc++;
	   old_Qd=Qd;
	   old_Cwnd=tcb->m_cWnd;
	   QdArray.push_back(Qd);
	   CwndArray.push_back(tcb->m_cWnd);
	   m_RTTmax=max(RTT,m_RTTmax);
	   tempMin=min(RTT,tempMin);
	   if(RTT<=m_RTTmin){
		   nearEmpty=true;
	   }
	   if(RTT>=glo_RTTmax)
	   {
		   probe_ceil=true;
		   glo_RTTmax=RTT;
	   }

   }
  // NS_LOG_UNCOND(this<<" now "<<Simulator::Now ().GetSeconds()<<" Record "<<oldRtt2.GetSeconds()<<" 2 "<<tempMin.GetSeconds()<<" "<<RTT.GetSeconds());


        double amplitudFm=0.0;
        double Qave=(sum/ackCount);
        double fm=5/cycle;
		curAm0 = Qave;

	double f0Am=0,simiFactor=0;
	double fmth=1/(20*m_RTTmin.GetSeconds());// fm threshold by lhy 2020.10.15
	//update lamda and cycle
	if(Simulator::Now ().GetSeconds() > oldRtt2.GetSeconds() + cycle){
		find_fm(&fm,&amplitudFm,&f0Am,QdArray,QdArray.size()/cycle,CwndArray,&simiFactor);
		amplitudFm*=2;
		double simiD=abs(double(1.0*(QdInc-CwndInc)))/QdArray.size();
		NS_LOG_DEBUG("debug:Interval_Arrive "<< this <<" now: "<< (double)1.0*nowToInt/1000000000<<" fm: "<< fm << " similarity: " << simiFactor/QdArray.size() << " simiD: "<<simiD<<" Bd "<<Bd << " AM0 " << curAm0 << " Qave " << Qave);
		if(isCom && (simiFactor/QdArray.size() > 0.01 || simiD > 0.7) && !isDC)//combining frequency and time domain : 2020.8.15 by lhy
		{
			/* while(!vals.empty())vals.pop_back();
			vals.push_back(make_pair(nowToInt,RTT));
			m_RTTmin=vals.front().second; */
			//int a = 1;
			//Bd =2*(m_RTTmax.GetMilliSeconds()-m_RTTmin.GetMilliSeconds());// adaptive Bd : 2020.11.11 by lhy
			//Bd = 1000*defaultBd;
			Bd = defaultBd*(1.0*(double)(m_RTTmax.GetSeconds()-RTTminop)/Qop);
			Bd = max(Bd,defaultBd);
			ifcompete=true;
			NS_LOG_DEBUG("debug:Compete "<< this <<" now: "<< (double)1.0*nowToInt/1000000000<<" RTTmax: "<< m_RTTmax.GetSeconds() << " RTTminop: " << RTTminop << " Qop: "<<Qop<<" Bd "<<Bd);
			//m_theta = 1;
		}
		else if(true && fm>0 && oldFm-std::min(5*fmth,2.0)<=fm&&fm<=oldFm+std::min(5*fmth,2.0) && curAm0>=0.9*preAm0 && curAm0<=1.1*preAm0){// restrain stable state by lhy 2021.9.2
			//if(!first_periodic)Bd=defaultBd;
			/* Bd /= 2;
			Bd = std::max(defaultBd,Bd); // modify 2021.08.26 by lhy */
			if(ifcompete) ifcompete=false;
			else 
			{
				cycle=5/fm;
				if(isDC)cycle=2/fm;// small cycle for Datacenter by lhy 2020.7.09
				cycle=min(20*Qave,cycle);
				double Qdamp,L_gain=1.0;
				
				//update Qdamp : 2020.8.18 by lhy
				if(!(nearEmpty || probe_ceil))pre_Qdave=Qave;// by lhy 2020.10.13
				Qdamp=std::max(m_RTTmax.GetSeconds()-pre_Qdave,pre_Qdave-tempMin.GetSeconds());
				if((m_RTTmax-tempMin).GetSeconds()/2>Qdamp)
				{
					Qdamp=(m_RTTmax-tempMin).GetSeconds()/2;
				}
				else
				{
					Qave=pre_Qdave;
				}
				
				/* if(nearEmpty)
				{
					if(m_RTTmax.GetSeconds()>pre_Qdave)Qave=pre_Qdave;
					Qdamp=m_RTTmax.GetSeconds()-Qave;
				}
				else if(pkt_loss)
				{
					if(tempMin.GetSeconds()<pre_Qdave)Qave=pre_Qdave;
					Qdamp=Qave-tempMin.GetSeconds();
				}
				else Qdamp=(m_RTTmax-tempMin).GetSeconds()/2; */
				
				//if(stable_state) L_gain=(pre_Qdave-m_RTTmin.GetSeconds())/Qdamp;
				double C_N=(m_lamuda*RTT.GetSeconds())/(Qdamp);
				if(Bd==defaultBd && first_periodic && Qdamp>=0.001*RTT.GetSeconds())//0915
				{
					//m_lamuda=min(0.1*C_N,Bdelay/Qave*C_N);// for fairness: 0917 by lhy
					//m_lamuda=0.5*Bdelay/Qave*C_N;// for fairness: 0917 by lhy
					/* m_lamuda = min(0.5 * Bdelay/Qave * Bw * std::log(Bdelay/(Qave-m_RTTmin.GetSeconds())),10*m_lamuda);// for fairness: 1106 by lhy
					if(m_lamuda<=0) m_lamuda=default_lamuda;*/
					first_periodic=false;
				}
				else if(Bd==defaultBd){ //&& Qdamp>=0.001*RTT.GetSeconds()){
					//NS_LOG_UNCOND(this<<" now "<<Simulator::Now ().GetSeconds()<<" max-Min "<<(m_RTTmax-tempMin).GetSeconds()<<" lamuda "<<m_lamuda<<" CwndC "<<m_lamuda* m_RTTmin.GetSeconds());
					//double s=0.25/fm;
					
					/* double C_N=(m_lamuda*RTT.GetSeconds())/(Qdamp); //in Mbps
					double k=std::pow(2.71828,C_N/Bw); 
					m_lamuda=min(max(L_gain*(C_N)*(Bdelay/k)/RTT.GetSeconds(),0.1*m_lamuda),10*m_lamuda);//in Mbps */
					
					//m_lamuda=min(max(L_gain*(Qave-m_RTTmin.GetSeconds())/Qdamp*m_lamuda,0.1*m_lamuda),10*m_lamuda); // 0901
					
					if(nearEmpty){
						detectedLamda = m_lamuda;
						m_lamuda/=2;
					}
					else m_lamuda+=0.2*detectedLamda;
					
					//NS_LOG_UNCOND(this<<" now "<<Simulator::Now ().GetSeconds()<<" C_N "<<C_N<<" k "<<k<<" ^x "<<0.001*1.5*C_N/Rc<<" CN_k "<<C_N/k<<" lamuda "<<m_lamuda);
				}
				else 
				{
					//m_lamuda=default_lamuda; //m_lamuda=1; //2020.7.04 modified by lhy
					m_lamuda=m_lamuda;
				}
				
				if(Bd==defaultBd)
				{
					Qop = Qave-m_RTTmin.GetSeconds();
					RTTminop = m_RTTmin.GetSeconds();
					//RTTminop = m_RTTmax.GetSeconds();
				}
				
				Bd=defaultBd;
				NS_LOG_DEBUG("debug:Interval_Arrive "<< this <<" now: "<< (double)1.0*nowToInt/1000000000 << " C/N: "<< C_N << " Qdamp: "<<Qdamp<<" pre_Qdave: "<<pre_Qdave<<" Qave: "<<Qave-m_RTTmin.GetSeconds()<<" L_gain: "<<L_gain<<" lamda: "<<m_lamuda << " Pmax: "<< m_RTTmax.GetSeconds() << " Pmin: " << tempMin.GetSeconds() << " clear? " << nearEmpty<< " Am0: "<< curAm0 << " detectedLamda: "<< detectedLamda);
			stable_state=true;
			}
		}else{
			
			//Bd=defaultBd;
			cycle = 20*Qave;
			m_lamuda = detectedLamda;
			
			//cycle=default_cycle; //cycle=1; //2020.7.04 modified by lhy
			stable_state=false;
			first_periodic=true;
			
			//m_lamuda=default_lamuda; // 2020.09.15
			//m_lamuda=1;
		}
		
		/* if(nearEmpty){
			m_lamuda=0.5*m_lamuda;
		}else{
			if(Qdamp>0){
				m_lamuda=(Bdelay*m_lamuda/Qdamp+m_lamuda)/2; 
			}else{				
				m_lamuda=m_lamuda+1;
			}

		} */

		oldFm=fm;
		
	    //m_lamuda=1;
		//cycle=10/fm;
		//cycle=1;

		oldRtt2=Simulator::Now ();
	   m_RTTmax=Time::Min ();

	   tempMin=Time::Max ();

	   m_RTTmax=max(RTT,m_RTTmax);
	   tempMin=min(RTT,tempMin);
	   nearEmpty=false;
	   probe_ceil=false;
	   pre_Qdave=Qave;
	   preAm0=curAm0;// restrain stable state by lhy 2021.9.2

		//m_delta=2;
	  sum=0.0;
	  ackCount=0;

	  QdArray.clear();
	  CwndArray.clear();
	  QdInc=0;
	  CwndInc=0;
	}

	//update m_RTTstanding
	int64_t recordTimeStandingToInt = recordTimeStanding.GetInteger();
	if(nowToInt > recordTimeStandingToInt + rtt.GetInteger()/2){

		m_RTTstanding = Time::Max ();
		m_RTTstanding = std::min (m_RTTstanding, RTT);
		oldCwndMid=tcb->m_cWnd;
		m_oldRTTstanding = m_RTTstanding;
		//m_RTTstanding = Time::Max ();
		recordTimeStanding = Simulator::Now ();
		srttPoint = Simulator::Now ().GetSeconds();
	}else{
		srttPoint=0;
		m_RTTstanding = std::min (m_RTTstanding, RTT);
	}


	//NS_LOG_UNCOND(this<<" now "<<Simulator::Now ().GetSeconds()<<" TR "<<targetRate<<" Qd "<<Qd<<" Bw "<<Bw<<" ln "<<std::log(Bdelay/Qd));
	targetRate =Bw * std::log(Bdelay/Qd);// in Mbps
	//Bdelay+=0;Bw+=0;
	//targetRate = 2/Qd;
	//double currentRate = tcb->m_cWnd/ m_RTTstanding.GetSeconds();// Byte/s
	//NS_LOG_UNCOND(this<<" now "<<Simulator::Now ().GetSeconds()<<" TR "<<targetRate<<" currentRate ");
   	intervalPoint=0;
    	// update theta
	//if (tcb->m_lastAckedSeq >= m_begSndNxt)// RTT hits
	if(Simulator::Now ().GetSeconds() > oldRtt1.GetSeconds() + RTT.GetSeconds())
	{
		// A periodicDC cycle has finished, we do Copa cwnd adjustment every RTT.
		intervalPoint=Simulator::Now ().GetSeconds();


		//NS_LOG_UNCOND(this<<" now "<<Simulator::Now ().GetSeconds()	<<" CwndList[0] "<<CwndList[0]);
		bool accelate=true;
		for(int i=1;i<=2;i++){
			if(CwndList[i-1]>CwndList[i]){
				//accelate=false;	 
			}				
		}
		
		if(CwndList[2]>cur_Rate){
			newDirect=false;
		}else{
			newDirect=true;
		}
		if(oldDirect==newDirect)UpDirectCount++;
		else UpDirectCount=0;
		if(UpDirectCount>=2){ // >2 -> >=2 for accelate at 3rd rtt, modified by lhy 2020.7.09
			accelate=true;	
		}else{
			accelate=false;
		}
		oldDirect=newDirect;
			
		

		if(accelate){
			m_theta=std::min(2*m_theta,(cur_Rate)/(m_lamuda));

			//m_theta=((double)tcb->m_cWnd/tcb->m_segmentSize)/m_lamuda;
			m_theta=std::max(1.0,m_theta);
		}
		else{
			m_theta=1;
		}

		//NS_LOG_UNCOND(this<<" now "<<Simulator::Now ().GetSeconds()<<" accelate "<<accelate<<" m_theta "<<m_theta);



		for(int i=1;i<=2;i++){
			CwndList[i-1]=CwndList[i];			
		}
		CwndList[2]=cur_Rate;
		

		m_begSndNxt = tcb->m_nextTxSequence;
		oldRtt1=Simulator::Now ();
		oldCwndpkts=newCwndpkts;
		oldDirect=newDirect;

	}
	//tcb->SetPacingRate(cur_Rate);
	
//	std::chrono::high_resolution_clock::time_point endTime = std::chrono::high_resolution_clock::now();
//	std::chrono::duration<double, std::milli> fp_ms =endTime - beginTime;
//    std::cout <<this << "Time is " << fp_ms.count()<< " ms" << endl;


	//if (!m_doingPeriodicDCNow)m_theta=1; // 0829
	
	
	/* if(false && Bd > defaultBd && Simulator::Now () > pktLostTime + RTT && (tcb->m_congState == TcpSocketState::CA_LOSS  || tcb->m_congState == TcpSocketState::CA_RECOVERY)){
                    cur_Rate = cur_Rate * 0.7;
					pktLostTime = Simulator::Now ();
	 }
	 else{
		double Qd = RTT.GetSeconds() - m_RTTmin.GetSeconds();
		targetRate =Bw* std::log(Bdelay/Qd);
		double factor = m_RTTmin.GetSeconds()/0.05; 
		factor = m_RTTmin.GetSeconds();
		
		factor=1; 


		if(cur_Rate <= targetRate){
			cur_Rate=std::min(cur_Rate+factor*m_lamuda*m_theta*ack_interval/RTT.GetSeconds(),10*cur_Rate);
		}else {
			m_theta=1;
			cur_Rate=std::max(cur_Rate-factor*m_lamuda*m_theta*ack_interval/RTT.GetSeconds(),0.5*cur_Rate);
		}
	 }
	 cur_Rate = max(cur_Rate, 0.2 * (1.0*tcb->m_segmentSize*8/1000000) / m_RTTmin.GetSeconds());//add lower boundary for rate by lhy 12.23
	 tcb->SetPacingRate(cur_Rate);
	 if(Bd==defaultBd)tcb->m_cWnd = cwnd_gain*cur_Rate*m_RTTmin.GetSeconds()*1000000/8;//in Byte
	 else tcb->m_cWnd = cwnd_gain*cur_Rate*RTT.GetSeconds()*1000000/8;
	 if(tcb->m_cWnd < tcb->m_segmentSize*2)tcb->m_cWnd=tcb->m_segmentSize*2; */

	NS_LOG_INFO(this<<" rttI "<<this->GetrttInst()<<" Stand "<<m_RTTstanding
	  <<" Old "<<m_oldRTTstanding<<" RTTmin "<<m_RTTmin
	  <<" Qd "<<Qd<<" srttPoint "<<srttPoint
	  <<" now "<<(double)1.0*nowToInt/1000000000<<" timePoint(ns) "<< nowToInt % 1000000
	  <<" cwnd "<<tcb->m_cWnd/1448.0<<" CRate "<<cur_Rate
	  <<" Trate "<<targetRate<<" pRate "<<tcb->GetPacingRate()
	  <<" lamuda "<<m_lamuda<<" theta "<<m_theta
	  <<" lsAck "<<tcb->m_lastAckedSeq.GetValue()
	  <<" newDirect "<<newDirect<<" ackCount "<<ackCount<<" rtt "<<rtt<<" sum "<<sum<<" Qave "<<sum/ackCount
	  <<" Rc "<<Rc<<" Bd "<<Bd
	  <<" cycle "<<cycle<<" targetTheta "<<Qave*ackCount*1500/cycle/1448.0<<" target "<<ackCount*1500/cycle
	  <<" fm "<<oldFm
	  <<" amplitudFm "<<amplitudFm<<" Amf0 "<<f0Am
	  <<" m_begSndNxt "<<m_begSndNxt<<" nextTxSeq "<<tcb->m_nextTxSequence
	  <<" maximal "<<m_RTTmax.GetSeconds()<<" minimal "<<tempMin.GetSeconds() << " Ack_interval "<<ack_interval<<" step "<<m_lamuda*m_theta*ack_interval/RTT.GetSeconds()<<" ifPDCC "<<m_doingPeriodicDCNow << " TcpState " << tcb->m_congState);



}
void
TcpPeriodicDC::PktsAckedAPeriodicDC (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked,
                     const Time& rtt)
{
	NS_LOG_FUNCTION (this << tcb << segmentsAcked << rtt);

}

void
TcpPeriodicDC::EnablePeriodicDC (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);

  m_doingPeriodicDCNow = true;
  m_begSndNxt = tcb->m_nextTxSequence;
  m_cntRtt = 0;
}

void
TcpPeriodicDC::DisablePeriodicDC ()
{
  NS_LOG_FUNCTION (this);

  m_doingPeriodicDCNow = false;
}

void
TcpPeriodicDC::CongestionStateSet (Ptr<TcpSocketState> tcb,
                              const TcpSocketState::TcpCongState_t newState)
{
  NS_LOG_FUNCTION (this << tcb << newState);
  if (newState == TcpSocketState::CA_OPEN)
    {
	  EnablePeriodicDC (tcb);
    }
  else
    {
		//Bd=defaultBd;//2020.11.11 by lhy
		NS_LOG_DEBUG("debug:switching "<< this <<" now: "<< (double)1.0*Simulator::Now ().GetInteger()/1000000000 << " Entering " << newState);
	  if (newState == TcpSocketState::CA_RECOVERY){
		  //NS_LOG_INFO("debug:switching "<< this <<" now: "<< (double)1.0*Simulator::Now ().GetInteger()/1000000000 << " Entering Recovery");
	      if(!defaultMode){
	    	  //m_lamuda=std::max(m_lamuda/2,2.0);
	      }else{
	    	  //m_lamuda=2.0;
	      }
		  //NS_LOG_LOGIC(this<<" delta "<<m_lamuda);
	  }
	  //NS_LOG_INFO(this<<" Debug_L: Now:"<< Simulator::Now() << " Cwnd_old:"<<tcb->m_cWnd);
	  DisablePeriodicDC ();
    }
}
void
TcpPeriodicDC::ExitRecovery (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this << tcb << segmentsAcked);
  NS_LOG_DEBUG(this<<"Debug_L: without adjust window");


}

void
TcpPeriodicDC::IncreaseWindow (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
	NS_LOG_FUNCTION (this << tcb << segmentsAcked);
	NS_LOG_DEBUG("debug:Regularte "<< this <<" now: "<< Simulator::Now () << " pktLostTime: "<< pktLostTime << " tcpState: "<< tcb->m_congState );


          if(Bd > defaultBd && Simulator::Now () > pktLostTime + this->GetrttInst() &&  tcb->m_congState == TcpSocketState::CA_LOSS){
                    cur_Rate = cur_Rate * 0.7;
					tcb->SetPacingRate(cur_Rate);
					tcb->m_cWnd = cwnd_gain*cur_Rate*m_RTTmin.GetSeconds()*1000000/8;//in Byte
					if(tcb->m_cWnd < tcb->m_segmentSize*2)tcb->m_cWnd=tcb->m_segmentSize*2;
					pktLostTime = Simulator::Now ();
		    return;
	 }
          double Bdelay=Bd*0.001;
	  double Bw=Rc*100/12;

	  if (false && !m_doingPeriodicDCNow)
	    {
	      // If Vegas is not on, we follow NewReno algorithm
	      //NS_LOG_LOGIC ("Vegas is not turned on, we follow NewReno algorithm.");
		  //NS_LOG_INFO(this<<" Debug_L: Now:"<< Simulator::Now() << " Cwnd_old:"<<tcb->m_cWnd);
	      TcpNewReno::IncreaseWindow (tcb, segmentsAcked);
		  
		  m_theta=1; // 0829
		  
		  //NS_LOG_INFO(this<<" Debug_L: Now:"<< Simulator::Now() << " Cwnd_new:"<<tcb->m_cWnd);
	      return;
	    }
	
	Time RTT=this->GetrttInst();
	//double Qd = m_RTTstanding.GetSeconds() - m_RTTmin.GetSeconds(); 
	double Qd = RTT.GetSeconds() - m_RTTmin.GetSeconds();

	//double currentRate = (tcb->m_lastAckedSeq.GetValue()-oldseq)/(Simulator::Now ().GetSeconds()-oldtime);
	//double currentRate = tcb->m_cWnd/m_RTTstanding.GetSeconds();

	//double targetRate = m_delta*tcb->m_segmentSize/Qd;
	targetRate =Bw* std::log(Bdelay/Qd);
	//Bdelay+=0;Bw+=0;
	//targetRate=2/Qd;
	double factor = m_RTTmin.GetSeconds()/0.05; 
	factor = m_RTTmin.GetSeconds();
	
	factor=1; 


	if(cur_Rate <= targetRate){
		//NS_LOG_INFO(" currentRate <= targetRate "<<tcb->m_cWnd );
		//tcb->m_cWnd = (double)tcb->m_cWnd + factor*m_lamuda*m_theta*ackTemp*tcb->m_segmentSize;
		//cur_Rate=std::min(cur_Rate+factor*m_lamuda*m_theta*ack_interval/RTT.GetSeconds(),2*cur_Rate);
		cur_Rate=std::min(cur_Rate+factor*m_lamuda*m_theta*ack_interval/RTT.GetSeconds(),10*cur_Rate);
		//NS_LOG_UNCOND(this<<" now "<<Simulator::Now ().GetSeconds()<<" currentRate <= targetRate "<<tcb->m_cWnd<<" diff "<<m_lamuda*m_theta*ackTemp*tcb->m_segmentSize  );
	}else {
		//NS_LOG_INFO(" currentRate > targetRate "<<tcb->m_cWnd );
		//tcb->m_cWnd = std::max((double)tcb->m_segmentSize, (double)tcb->m_cWnd - factor*m_lamuda*m_theta*ackTemp*tcb->m_segmentSize);
		//if(newDirect)m_theta=1;
		m_theta=1;
		cur_Rate=std::max(cur_Rate-factor*m_lamuda*m_theta*ack_interval/RTT.GetSeconds(),0.5*cur_Rate);
		//NS_LOG_UNCOND(this<<" now "<<Simulator::Now ().GetSeconds()<<" currentRate > targetRate "<<tcb->m_cWnd<<" diff "<<m_lamuda*m_theta*ackTemp*tcb->m_segmentSize );
	}
	cur_Rate = std::max(cur_Rate,0.2 * (1.0*tcb->m_segmentSize*8/1000000) / m_RTTmin.GetSeconds());
	tcb->SetPacingRate(cur_Rate);
	//tcb->SetPacingRate(0);
	tcb->m_cWnd = cwnd_gain*cur_Rate*m_RTTmin.GetSeconds()*1000000/8;//in Byte
	if(tcb->m_cWnd < tcb->m_segmentSize*2)tcb->m_cWnd=tcb->m_segmentSize*2;
}

void
TcpPeriodicDC::PeriodicDCModifyWindow (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked){





}

std::string
TcpPeriodicDC::GetName () const
{
  return "TcpPeriodicDC";
}

uint32_t
TcpPeriodicDC::GetSsThresh (Ptr<const TcpSocketState> tcb,
                       uint32_t bytesInFlight)
{
  NS_LOG_FUNCTION (this << tcb << bytesInFlight);
  //return std::max (std::min (tcb->m_ssThresh.Get (), tcb->m_cWnd.Get () - tcb->m_segmentSize), 2 * tcb->m_segmentSize);
  return std::max (tcb->m_cWnd.Get () - tcb->m_segmentSize, 2 * tcb->m_segmentSize);//^_^
}

void
TcpPeriodicDC::DecreaseWindow(Ptr<TcpSocketState> tcb)
{
	  if(Bd > defaultBd){
					Time RTT=this->GetrttInst();
                    cur_Rate = cur_Rate * 0.7;
					pktLostTime = Simulator::Now ();
					tcb->SetPacingRate(cur_Rate);
					tcb->m_cWnd = cwnd_gain*cur_Rate*RTT.GetSeconds()*1000000/8;
					if(tcb->m_cWnd < tcb->m_segmentSize*2)tcb->m_cWnd=tcb->m_segmentSize*2;
	 }
}

void
TcpPeriodicDC::find_fm_row(double *fm,double *am,double *qavr,double T,vector<double> &pr)
{
	int k=0,n=pr.size();
	double fs=n/T;
	while((1<<k)<n)k++;

	pr.insert(pr.end(),(1<<k)-n,0);
	n=(1<<k);
	vector<double> pi(n,0),fr(n,0),fi(n,0);
	//-----fft------
	int it,m,is,i,j,nv,l0;
    double p,q,s,vr,vi,poddr,poddi;
    for (it=0; it<=n-1; it++)  //œ«pr[0]ºÍpi[0]Ñ­»·ž³Öµžøfr[]ºÍfi[]
    {
		m=it;
		is=0;
		for(i=0; i<=k-1; i++)
        {
			j=m/2;
			is=2*is+(m-2*j);
			m=j;
		}
        fr[it]=pr[is];
        fi[it]=pi[is];
    }
    pr[0]=1.0;
    pi[0]=0.0;
    p=6.283185306/(1.0*n);
    pr[1]=cos(p); //œ«w=e^-j2pi/nÓÃÅ·À­¹«Êœ±íÊŸ
    pi[1]=-sin(p);
    for (i=2; i<=n-1; i++)  //ŒÆËãpr[]
    {
		p=pr[i-1]*pr[1];
		q=pi[i-1]*pi[1];
		s=(pr[i-1]+pi[i-1])*(pr[1]+pi[1]);
		pr[i]=p-q; pi[i]=s-p-q;
    }
    for (it=0; it<=n-2; it=it+2)
    {
		vr=fr[it];
		vi=fi[it];
		fr[it]=vr+fr[it+1];
		fi[it]=vi+fi[it+1];
		fr[it+1]=vr-fr[it+1];
		fi[it+1]=vi-fi[it+1];
    }
	m=n/2;
	nv=2;
    for (l0=k-2; l0>=0; l0--) //ºûµû²Ù×÷
    {
		m=m/2;
		nv=2*nv;
        for (it=0; it<=(m-1)*nv; it=it+nv)
          for (j=0; j<=(nv/2)-1; j++)
            {
				p=pr[m*j]*fr[it+j+nv/2];
				q=pi[m*j]*fi[it+j+nv/2];
				s=pr[m*j]+pi[m*j];
				s=s*(fr[it+j+nv/2]+fi[it+j+nv/2]);
				poddr=p-q;
				poddi=s-p-q;
				fr[it+j+nv/2]=fr[it+j]-poddr;
				fi[it+j+nv/2]=fi[it+j]-poddi;
				fr[it+j]=fr[it+j]+poddr;
				fi[it+j]=fi[it+j]+poddi;
            }
    }
    for (i=0; i<=n-1; i++)
    {
		  pr[i]=sqrt(fr[i]*fr[i]+fi[i]*fi[i]);
    }
    //-----------------------
	double qst=-1;
	int nn=n>>1,indd=0;
	for(i=1;i<=nn;i++)
	{
		if(i*fs/n>1.5 && pr[i]>qst)
		{
			qst=pr[i];
			indd=i;
		}
	}


	*qavr=pr[0]/n;
	*fm=indd*fs/n;
	*am=2*qst/n;
	return ;
}

void
TcpPeriodicDC::find_fm(double *fm,double *am,double *f0am,vector<double> &s,double fs,vector<double> &c,double *simi)
{
	// Imagprecise normlization by using Amp[0] -> change to use Amp[fm] (to be done 2020.11.13)
	
	int n=s.size();
	fftw_complex *outa,*ina;
	outa = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n);
	ina = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n);
	double *rtt_amp = new double [n];
	/*
	fftw_plan pp=fftw_plan_dft_1d(n,ina,outa,FFTW_FORWARD,FFTW_ESTIMATE);
	fftw_execute(pp);
	fftw_destroy_plan(pp);
	
	double P_ave=sqrt(outa[0][0]*outa[0][0]+outa[0][1]*outa[0][1]);
	cwnd_amp[0]=1;
	for(int i=1;i<n/2;i++)
	{
		cwnd_amp[i]=sqrt(outa[i][0]*outa[i][0]+outa[i][1]*outa[i][1])*2/P_ave;
	}
	*/
	
	for(int i=0;i<n;i++)
	{
		ina[i][0]=s[i];
        ina[i][1]=0;
	} 
	fftw_plan p=fftw_plan_dft_1d(n,ina,outa,FFTW_FORWARD,FFTW_ESTIMATE);
	fftw_execute(p);
	fftw_destroy_plan(p);
	double temp,qst=-1;
	//P_ave=sqrt(outa[0][0]*outa[0][0]+outa[0][1]*outa[0][1]);
	for(int i=1;i<n/2;i++)
	{
		temp=sqrt(outa[i][0]*outa[i][0]+outa[i][1]*outa[i][1]);
		if(i*fs/n>1.5 && qst<temp)
		{
			qst=temp;
			*fm=i;
		}
		/* temp=temp*2/P_ave;
		*simi+=fabs(temp-cwnd_amp[i]); */
	}
	
	//double P_ave=sqrt(outa[(int)*fm][0]*outa[(int)*fm][0]+outa[(int)*fm][1]*outa[(int)*fm][1]);
	qst=sqrt(outa[0][0]*outa[0][0]+outa[0][1]*outa[0][1]);
	//curAm0=qst;// restrain stable state by lhy 2021.9.2
	rtt_amp[0]=1;
	for(int i=1;i<n/2;i++)
	{
		rtt_amp[i]=sqrt(outa[i][0]*outa[i][0]+outa[i][1]*outa[i][1])*2/qst;
	}
	
	for(int i=0;i<n;i++)
	{
		ina[i][0]=c[i];
        ina[i][1]=0;
	}
	fftw_plan pp=fftw_plan_dft_1d(n,ina,outa,FFTW_FORWARD,FFTW_ESTIMATE);
	fftw_execute(pp);
	fftw_destroy_plan(pp);
	for(int i=1;i<n/2;i++)
	{
		temp=sqrt(outa[i][0]*outa[i][0]+outa[i][1]*outa[i][1]);
		if(i*fs/n>1.5 && qst<temp)
		{
			qst=temp;
		}
		/* temp=temp*2/P_ave;
		*simi+=fabs(temp-cwnd_amp[i]); */
	}
	qst=sqrt(outa[0][0]*outa[0][0]+outa[0][1]*outa[0][1]);
	for(int i=1;i<n/2;i++)
	{
		temp=sqrt(outa[i][0]*outa[i][0]+outa[i][1]*outa[i][1])*2/qst;
		*simi+=fabs(temp-rtt_amp[i]);
	}
	
	

	*fm=(*fm)*fs/n;
	*am=qst/n;
	*f0am=sqrt(outa[0][0]*outa[0][0]+outa[0][1]*outa[0][1])/n;
	fftw_free(outa);
	fftw_free(ina);
	delete rtt_amp;
	//cout<<"haha"<<endl;
}

} // namespace ns3




