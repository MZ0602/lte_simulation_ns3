#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/config-store-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/radio-environment-map-helper.h"
#include <stdlib.h> 
#include <fstream>
#include <string>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LENAX2HANDOVERTEST");

// setup parameters for threshold and cwnd calculations

static bool firstSshThr = true;
static bool firstCwnd = true;
static Ptr<OutputStreamWrapper> ssThreshStream;
static Ptr<OutputStreamWrapper> cWndStream;
static uint32_t ssThreshValue;
static uint32_t cWndValue;


// Flow Monitor Throughput Monitor

void ThroughputMonitor (FlowMonitorHelper *flowHelper, Ptr<FlowMonitor> flowMonitor)
{
	std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMonitor->GetFlowStats();
	Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (flowHelper->GetClassifier());

	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin (); stats != flowStats.end (); ++stats)
	{
		Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow (stats->first);
		std::cout<<"Flow ID	: " << stats->first <<";         "<< fiveTuple.sourceAddress <<" -----> "<<fiveTuple.destinationAddress<<std::endl;
		std::cout<<"Tx Packets = " << stats->second.txPackets<<std::endl;
		std::cout<<"Tx Bytes = " << stats->second.txBytes<<std::endl;
		std::cout<<"Rx Packets = " << stats->second.rxPackets<<std::endl;
		std::cout<<"Rx Bytes = " << stats->second.rxBytes<<std::endl;
		std::cout<<"Duration		: "<<stats->second.timeLastTxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds()<<std::endl;
		std::cout<<"Last Received Packet	: "<< stats->second.timeLastTxPacket.GetSeconds()<<" Seconds"<<std::endl;
		std::cout<<"Lost Packet : "<< stats->second.lostPackets<<std::endl;
		std::cout<<"Throughput: " << stats->second.rxBytes*1.0/(stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstRxPacket.GetSeconds()) << " B/s"<<std::endl;
		std::cout<<"---------------------------------------------------------------------------"<<std::endl;
	}
	Simulator::Schedule(Seconds(1.0),&ThroughputMonitor, flowHelper, flowMonitor);
}


static void
CwndTracer (uint32_t oldval, uint32_t newval)
{
  if (firstCwnd)
    {
      *cWndStream->GetStream () << "0.0 " << oldval << std::endl;
      firstCwnd = false;
    }
  *cWndStream->GetStream () << Simulator::Now ().GetSeconds () << " " << newval << std::endl;
  cWndValue = newval;

  if (!firstSshThr)
    {
      *ssThreshStream->GetStream () << Simulator::Now ().GetSeconds () << " " << ssThreshValue << std::endl;
    }
}


static void
SsThreshTracer (uint32_t oldval, uint32_t newval)
{
  if (firstSshThr)
    {
      *ssThreshStream->GetStream () << "0.0 " << oldval << std::endl;
      firstSshThr = false;
    }
  *ssThreshStream->GetStream () << Simulator::Now ().GetSeconds () << " " << newval << std::endl;
  ssThreshValue = newval;

  if (!firstCwnd)
    {
      *cWndStream->GetStream () << Simulator::Now ().GetSeconds () << " " << cWndValue << std::endl;
    }
}




static void
TraceCwnd (std::string cwnd_tr_file_name)
{
  AsciiTraceHelper ascii;
  cWndStream = ascii.CreateFileStream (cwnd_tr_file_name.c_str ());
  Config::ConnectWithoutContext ("/NodeList/1/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow", MakeCallback (&CwndTracer));
}

static void
TraceSsThresh (std::string ssthresh_tr_file_name)
{
  AsciiTraceHelper ascii;
  ssThreshStream = ascii.CreateFileStream (ssthresh_tr_file_name.c_str ());
  Config::ConnectWithoutContext ("/NodeList/1/$ns3::TcpL4Protocol/SocketList/0/SlowStartThreshold", MakeCallback (&SsThreshTracer));
}




int main(int argc, char *argv[])
{
    LogLevel logLevel = (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_LEVEL_ALL);
    LogComponentEnable ("EpcX2", logLevel);
    LogComponentEnable ("LteUeNetDevice", logLevel);
    // LogComponentEnable ("EpcEnbApplication", logLevel);


    // Simulation Setup
    //uint16_t numberOfNodes = 3;
    std::string prefix_file_name = "CalculateCwnd";
    uint16_t numberOfUes = 1;
    uint16_t numberOfEnbs = 2;
    uint16_t numberOfBearersPerUe = 2;
    double distance = 400.0;
    double speed = 40.0;
    double yForUe = 200.0;
    double enbTxPowerDbm = 46.0;
    //double simTime = 2.000;
    double simTime = (double) (numberOfEnbs + 1) * distance / speed;
    
    // Change default attributes so that they are reasonable for our scenario.
    Config::SetDefault("ns3::UdpClient::Interval" , TimeValue(MilliSeconds(10)));
    Config::SetDefault("ns3::UdpClient::MaxPackets" , UintegerValue(1000000));
    Config::SetDefault("ns3::LteHelper::UseIdealRrc" , BooleanValue(false));


    // Command line arguments
    CommandLine cmd;
    cmd.AddValue ("simTime", "Total duration of the simulation (in seconds)", simTime);
    cmd.AddValue ("speed", "Speed of the UE (default = 20 m/s)", speed);
    cmd.AddValue ("enbTxPowerDbm", "TX power [dBm] used by HeNBs (default = 46.0)", enbTxPowerDbm);

    cmd.Parse (argc, argv);
    // ConfigStore config;
	// config.ConfigureDefaults ();

    // Initialize LTE Helper for our Simulation
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
    lteHelper->SetEpcHelper (epcHelper);
    lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
    //lteHelper->SetHandoverAlgorithmType ("ns3::NoOpHandoverAlgorithm");

    // Set A3RSRP Handover Algorithm
    lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
    lteHelper->SetHandoverAlgorithmAttribute ("Hysteresis",
                                             DoubleValue (0.0));
    lteHelper->SetHandoverAlgorithmAttribute ("TimeToTrigger",
                                             TimeValue (MilliSeconds (256)));

    // Propagation Loss Model
    lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisPropagationLossModel"));

    // Fading Traces
    // lteHelper->SetFadingModel("ns3::TraceFadingLossModel");
    // lteHelper->SetFadingModelAttribute ("TraceFilename", StringValue ("src/lte/model/fading-traces/fading_trace_EPA_3kmph.fad"));
    // lteHelper->SetFadingModelAttribute ("TraceLength", TimeValue (Seconds (simTime)));
    // lteHelper->SetFadingModelAttribute ("SamplesNum", UintegerValue (10000));
    // lteHelper->SetFadingModelAttribute ("WindowSize", TimeValue (Seconds (0.5)));
    // lteHelper->SetFadingModelAttribute ("RbNum", UintegerValue (100));


    // Get the PGW node
    Ptr<Node> pgw = epcHelper->GetPgwNode ();

    // Create a single remote host
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create (1);
    Ptr<Node> remoteHost = remoteHostContainer.Get (0);
    InternetStackHelper internet;
    internet.Install (remoteHostContainer);

    // Create the Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute ("DataRate" , DataRateValue (DataRate ("100Gb/s")));
    p2ph.SetDeviceAttribute ("Mtu" , UintegerValue(1500));
    p2ph.SetChannelAttribute ("Delay" , TimeValue (Seconds (0.010) ) );
    // InternetStackHelper stack;
    // stack.InstallAll ();
    NetDeviceContainer internetDevices = p2ph.Install (pgw , remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase ("1.0.0.0" , "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

    

    // Routing of the Internet Host (towards the LTE network)
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
    // interface 0 is localhost , 1 is the p2p device
    remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0") , Ipv4Mask ("255.0.0.0") , 1);

    // Initialize UE and ENB nodes
    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create (numberOfEnbs);
    ueNodes.Create (numberOfUes);

    // Install mobility model for stationary eNB nodes
    Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
    for(uint16_t i = 0 ; i < numberOfEnbs ; i++)
    {
        enbPositionAlloc->Add (Vector (distance * (i+1), distance , 0 ));
    }
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    enbMobility.SetPositionAllocator (enbPositionAlloc);
    enbMobility.Install (enbNodes);

    // Install mobility model for mobile UE nodes
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
    ueMobility.Install (ueNodes);
    ueNodes.Get (0)->GetObject<MobilityModel> ()-> SetPosition (Vector (0,yForUe,0));
    ueNodes.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (speed , 0 , 0));

    // Install LTE Devices in UEs and eNBs
    Config::SetDefault ("ns3::LteEnbPhy::TxPower" , DoubleValue (enbTxPowerDbm));
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
    NetDeviceContainer ueLteDevs  = lteHelper->InstallUeDevice (ueNodes);

    // Install the IP stack on UEs
    internet.Install (ueNodes);
    Ipv4InterfaceContainer ueIpIfaces;
    ueIpIfaces = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));

    // Attach all UEs to the first eNodeB
    for(uint16_t i = 0; i < numberOfUes ; i++)
    {
        lteHelper->Attach (ueLteDevs.Get (i) , enbLteDevs.Get (0));
    }
    
    NS_LOG_LOGIC ("setting up applications");

    // Install and Start applications on UE and remote host
    uint16_t dlPort = 10000;
    uint16_t ulPort = 20000;

    // randomize a bit start times to avoid simulations artifacts
    Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
    startTimeSeconds->SetAttribute ("Min", DoubleValue (0));
    startTimeSeconds->SetAttribute ("Max", DoubleValue (0.010));


    for(uint16_t i = 0 ; i < numberOfUes ; i++)
    {
        // Extract the UE node 
        Ptr<Node> ue = ueNodes.Get (i);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
        ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress () , 1);

        for(uint32_t j = 0 ; j < numberOfBearersPerUe ; j++)
        {
            ++dlPort;
            ++ulPort;

            ApplicationContainer clientApps;
            ApplicationContainer serverApps;

            // NS_LOG_LOGIC ("installing UDP DL app for UE" << i);
            // UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (i) , dlPort);
            // clientApps.Add (dlClientHelper.Install (remoteHost));
            // PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory" , InetSocketAddress (Ipv4Address::GetAny () , dlPort));
            // serverApps.Add (dlPacketSinkHelper.Install (ue));

            // NS_LOG_LOGIC ("installing UDP UL app for UE" << i);
            // UdpClientHelper ulClientHelper (remoteHostAddr , ulPort);
            // clientApps.Add (ulClientHelper.Install (ue));
            // PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory" , InetSocketAddress (Ipv4Address::GetAny () , ulPort));
            // serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
            
            
            BulkSendHelper dlClientHelper ("ns3::TcpSocketFactory", InetSocketAddress (ueIpIfaces.GetAddress (i), dlPort));
            dlClientHelper.SetAttribute ("MaxBytes", UintegerValue (0));
            clientApps.Add (dlClientHelper.Install (remoteHost));
            PacketSinkHelper dlPacketSinkHelper ("ns3::TcpSocketFactory", 
                                                InetSocketAddress (Ipv4Address::GetAny (), dlPort));
            serverApps.Add (dlPacketSinkHelper.Install (ue));
                 
            BulkSendHelper ulClientHelper ("ns3::TcpSocketFactory",
                                            InetSocketAddress (remoteHostAddr, ulPort));
            ulClientHelper.SetAttribute ("MaxBytes", UintegerValue (0));                  
            clientApps.Add (ulClientHelper.Install (ue));
            PacketSinkHelper ulPacketSinkHelper ("ns3::TcpSocketFactory", 
                                                InetSocketAddress (Ipv4Address::GetAny (), ulPort));
            serverApps.Add (ulPacketSinkHelper.Install (ue));  

            Ptr<EpcTft> tft = Create<EpcTft> ();
            EpcTft::PacketFilter dlpf;
            dlpf.localPortStart = dlPort;
            dlpf.localPortEnd = dlPort;
            tft->Add (dlpf);
            EpcTft::PacketFilter ulpf;
            ulpf.remotePortStart = ulPort;
            ulpf.remotePortEnd = ulPort;
            tft->Add (ulpf);
            
            EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
            lteHelper->ActivateDedicatedEpsBearer (ueLteDevs.Get (i), bearer, tft);

            Time startTime = Seconds (startTimeSeconds->GetValue ());
            serverApps.Start (startTime);
            clientApps.Start (startTime);
            
        }
    }
    
    // Add X2 Interface
    lteHelper->AddX2Interface (enbNodes);

    // X2-based Handover
    //lteHelper->HandoverRequest (Seconds (1.000) , ueLteDevs.Get (0) , enbLteDevs.Get (0) , enbLteDevs.Get (1));
    //lteHelper->HandoverRequest (Seconds (3.000) , ueLteDevs.Get (0) , enbLteDevs.Get (1) , enbLteDevs.Get (0));


    // Trace Configurations
    lteHelper->EnableTraces ();
    // lteHelper->EnablePhyTraces ();
    // lteHelper->EnableMacTraces ();
    // lteHelper->EnableRlcTraces ();
    // lteHelper->EnablePdcpTraces ();
    // Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats ();
    // rlcStats->SetAttribute ("EpochDuration", TimeValue (Seconds (1.0)));
    // Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
    // pdcpStats->SetAttribute ("EpochDuration", TimeValue (Seconds (1.0)));
    std::ofstream ascii;
    Ptr<OutputStreamWrapper> ascii_wrap;
    ascii.open ((prefix_file_name + "-ascii").c_str ());
    ascii_wrap = new OutputStreamWrapper ((prefix_file_name + "-ascii").c_str (),
                                        std::ios::out);
    internet.EnableAsciiIpv4All (ascii_wrap);
    Simulator::Schedule (Seconds (0.00001), &TraceCwnd, prefix_file_name + "-cwnd.data");
    Simulator::Schedule (Seconds (0.00001), &TraceSsThresh, prefix_file_name + "-ssth.data");

        
    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;
    flowMonitor = flowHelper.Install (ueNodes);
    flowMonitor = flowHelper.Install (remoteHostContainer);
    flowMonitor->CheckForLostPackets ();
    flowMonitor->SerializeToXmlFile("Handover_Stats.xml", true, true);
    // Simulator::Schedule(Seconds(1) , &ThroughputMonitor, &flowHelper , flowMonitor);


    Simulator::Stop (Seconds (simTime));

    Simulator::Run();


    





    // flowMonitor->CheckForLostPackets ();
    // Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowHelper.GetClassifier ());
    // std::map<FlowId , FlowMonitor::FlowStats> stats = flowMonitor->GetFlowStats ();
    // for (std::map<FlowId , FlowMonitor::FlowStats>::const_iterator iter = stats.begin (); iter != stats.end ();iter++)
    // {
    //     Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first);
    //     NS_LOG_UNCOND("Flow ID: " << iter->first << " Src Addr " << t.sourceAddress << " Dst Addr " << t.destinationAddress);
    //     NS_LOG_UNCOND("Tx Packets = " << iter->second.txPackets);
    //     NS_LOG_UNCOND("Rx Packets = " << iter->second.rxPackets);
    //     NS_LOG_UNCOND("Throughput: " << iter->second.rxBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds()) / 1024  << " Kbps");
    // }
    flowMonitor->SerializeToXmlFile("Handover_Stats.xml", true, true);

    Simulator::Destroy();

    return 0;
}