/*
 * This example program allows one to run ns-3 DSDV, AODV, or OLSR under
 * a typical random waypoint mobility model.
 *
 * By default, the simulation runs for 200 simulated seconds, of which
 * the first 50 are used for start-up time.  The number of nodes is 50.
 * Nodes move according to RandomWaypointMobilityModel with a speed of
 * 20 m/s and no pause time within a 300x1500 m region.  The WiFi is
 * in ad hoc mode with a 2 Mb/s rate (802.11b) and a Friis loss model.
 * The transmit power is set to 7.5 dBm.
 *
 * It is possible to change the mobility and density of the network by
 * directly modifying the speed and the number of nodes.  It is also
 * possible to change the characteristics of the network by changing
 * the transmit power (as power increases, the impact of mobility
 * decreases and the effective density increases).
 *
 * By default, OLSR is used, but specifying a value of 2 for the protocol
 * will cause AODV to be used, and specifying a value of 3 will cause
 * DSDV to be used.
 *
 * By default, there are 10 source/sink data pairs sending UDP data
 * at an application rate of 2.048 Kb/s each.    This is typically done
 * at a rate of 4 64-byte packets per second.  Application data is
 * started at a random time between 50 and 51 seconds and continues
 * to the end of the simulation.
 *
 * The program outputs a few items:
 * - packet receptions are notified to stdout such as:
 *   <timestamp> <node-id> received one packet from <src-address>
 * - each second, the data reception statistics are tabulated and output
 *   to a comma-separated value (csv) file
 * - some tracing and flow monitor configuration that used to work is
 *   left commented inline in the program
 */

#include <fstream>
#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;
using namespace dsr;

NS_LOG_COMPONENT_DEFINE ("manet-routing-compare");//������Ϊmanet-routing-compare����־���

class RoutingExperiment
{//������ʵ������ķ����ͱ���
public:
  RoutingExperiment ();
  //��������ʵ�飬��������������nWifis-WiFi�ڵ�������nSinks-sink�ڵ�������txp-���书��
  void Run (int nWifis, int nSinks, double txp);

  void CommandSetup (int argc, char **argv);//���������в���

private:
  //�������ýڵ�������ݵ�socket������������addr�ǽڵ��IPv4��ַ��node�ǽڵ��ָ��
  //�����д�����һ��UDP��socket���������˻ص����������ڽ������ݡ�
  Ptr<Socket> SetupPacketReceive (Ipv4Address addr, Ptr<Node> node);
  //��Ϊ�ص����������ڴ����յ������ݰ�������һ�� Socket ���͵�ָ����Ϊ������
  //��socket���յ�����ʱ����������������������յ������ݰ���
  void ReceivePacket (Ptr<Socket> socket);

  uint32_t port;//socket�Ķ˿ں�
  uint32_t bytesTotal;//�ܹ�������ֽ�����
  uint32_t packetsReceived;//�յ������ݰ�����

  std::string m_protocolName;//��ǰʹ�õ�·��Э�������
  bool m_traceMobility;//�Ƿ������˽ڵ��ƶ��Ե�׷��
  uint32_t m_protocol;// ʹ������·��Э����з���ʵ��
};

RoutingExperiment::RoutingExperiment ()
  : port (9),
    bytesTotal (0),
    packetsReceived (0),
    m_traceMobility (false),
    m_protocol (2) // AODV
{
}

//���ɽ��յ����ݰ�����־��Ϣ
static inline std::string
PrintReceivedPacket (Ptr<Socket> socket, Ptr<Packet> packet, Address senderAddress)
{
  std::ostringstream oss;

  oss << Simulator::Now ().GetSeconds () << " " << socket->GetNode ()->GetId ();

  if (InetSocketAddress::IsMatchingType (senderAddress))
    {
      InetSocketAddress addr = InetSocketAddress::ConvertFrom (senderAddress);
      oss << " received one packet from " << addr.GetIpv4 ();
    }
  else
    {
      oss << " received one packet!";
    }
  return oss.str ();
}

void
RoutingExperiment::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address senderAddress;
  while ((packet = socket->RecvFrom (senderAddress)))
    {
      bytesTotal += packet->GetSize ();
      packetsReceived += 1;
      //NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
    }
}

//����һ���������ݰ���Socket�����󶨵��ض��ĵ�ַ�Ͷ˿���
Ptr<Socket>
RoutingExperiment::SetupPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
//�����ַ��� "ns3::UdpSocketFactory" ��ȡ��Ӧ�� TypeId�����ڴ���һ��UDP���͵�Socket
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);//������һ��Socket���� sink��ʹ����֮ǰ��ȡ�� UDP Socket �� TypeId������󶨵�ָ���Ľڵ� node ��
  InetSocketAddress local = InetSocketAddress (addr, port);//������һ�� InetSocketAddress ���� local����ʾҪ�󶨵ĵ�ַ�Ͷ˿�
  sink->Bind (local);// ��������Socket sink �󶨵�ָ���ĵ�ַ�Ͷ˿��ϣ��Ա������Խ��շ��͵������ַ�Ͷ˿ڵ����ݰ�
  sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceivePacket, this));//�����˵�Socket���յ����ݰ�ʱ�Ļص�����

  return sink;//���ش�����Socket���� sink���Ա��������ط�ʹ�����������ݰ�
}

void
RoutingExperiment::CommandSetup (int argc, char **argv)
{
  CommandLine cmd (__FILE__);
  cmd.AddValue ("traceMobility", "Enable mobility tracing", m_traceMobility);
  cmd.AddValue ("protocol", "1=OLSR;2=AODV;3=DSDV;4=DSR", m_protocol);
  cmd.Parse (argc, argv);
}

int
main (int argc, char *argv[])
{
  RoutingExperiment experiment;
  
  double txp = 7.5;
  int nWifis = 65;
  int nSinks = 30;

  CommandLine cmd (__FILE__);
  cmd.AddValue ("nWifis", "nWifis", nWifis);
  cmd.AddValue ("nSinks", "nSinks", nSinks);
  cmd.Parse (argc, argv);
  experiment.Run (nWifis, nSinks, txp);
}

void
RoutingExperiment::Run (int nWifis, int nSinks, double txp)
{
  Packet::EnablePrinting ();//�������ݰ��Ĵ�ӡ����
  double TotalTime = 80.0;//������ʱ��
  std::string rate ("2048bps");//���ݴ�������,����Ϊ2048������Ϊ��ģ��һ���������绷��
  std::string phyMode ("DsssRate11Mbps");//����ģʽΪ DSSS������Ϊ 11 Mbps
  std::string tr_name ("AODV_Topology_Trace");//׷���ļ���
  int nodeSpeed = 20; //�ڵ��ƶ��ٶ�m/s
  int nodePause = 0; //��ͣʱ��s
  m_protocolName = "protocol";

  Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("512"));//����OnOffApplication Ӧ�ó�������ݰ���СΪ 512 �ֽ�
  Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (rate));//���� OnOff Ӧ�õ����ݴ�������Ϊ֮ǰ����� rate

  //Set Non-unicastMode rate to unicast mode
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode));//������ Wi-Fi �������ķǵ���ģʽΪ֮ǰ����� phyMode

  NodeContainer adhocNodes;
  adhocNodes.Create (nWifis);//������ nWifis ���ڵ�Ľڵ�����

  // setting up wifi phy and channel using helpers
  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);//������һ����Ϊ wifi �� WifiHelper ���󣬲������� Wi-Fi ��׼Ϊ 802.11b

  YansWifiPhyHelper wifiPhy;//YANS �� NS-3 ������ģ����������������ģ��֮һ������ģ�� WiFi �豸���������Ϊ��
  YansWifiChannelHelper wifiChannel;//������ Wi-Fi ���������ŵ����Եİ���������
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");//�����ŵ������ӳ�ģ��Ϊ�㶨�ٶȴ����ӳ�ģ��
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");//����� Friis �������ģ�ͣ�����һ�ֻ��ھ���ĳ��ô������ģ��
  wifiPhy.SetChannel (wifiChannel.Create ());//��֮ǰ���úõ��ŵ�Ӧ�õ� Wi-Fi ���������

  // Add a mac and disable rate control
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));

  wifiPhy.Set ("TxPowerStart",DoubleValue (txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txp));//���� Wi-Fi �ķ��͹�����ʼֵ�ͽ���ֵΪ txp

  wifiMac.SetType ("ns3::AdhocWifiMac");//�� Wi-Fi MAC ����������Ϊ Adhoc ģʽ�������ڵ㽫��������ģʽ�½���ͨ��
  NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, adhocNodes);//��װ Wi-Fi �������� MAC �㵽֮ǰ�����Ľڵ����� adhocNodes �У������������豸���� adhocDevices ��

  MobilityHelper mobilityAdhoc;//�ƶ�ģ�Ͱ���������
//��ȷ���ڲ�ͬ�����л��һ�µ��ƶ��ԡ�streamIndex��α����������������ӡ�
  //����ͬ�ĳ����������£�����������ģ����������ͬ��α�����������������ͬ��˳��������ͬ����������У�����������ͬ���ƶ���ģʽ��
  //����ȷ���ڶ������ģ�������£�����ͬ�������µõ���ͬ���ƶ��ԣ��Ա����һ�µ�ģ��ͱȽϡ�
  int64_t streamIndex = 0; 

  //ʹ�ýڵ���һ�� 500x500 �ľ�������������ֲ�
  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
  pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));

  Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();//λ�÷�����
  streamIndex += taPositionAlloc->AssignStreams (streamIndex);//Ϊλ�÷����������������������Щ��������������ɽڵ��λ�ú��ƶ������е����������

  //�����ַ����� ssSpeed �� ssPause���ֱ��������ɽڵ���ƶ��ٶȺ���ͣʱ����������
  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";//��Сֵ0�����ֵnodeSpeed=20m/s
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";//��ΪnodePause=0
  //���ýڵ���ƶ�ģ��Ϊ��������ƶ�ģ�ͣ����������ٶȡ���ͣʱ���Լ�֮ǰ������λ�÷�������
  mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc));
  //Ϊ�ƶ�ģ������λ�÷�����
  mobilityAdhoc.SetPositionAllocator (taPositionAlloc);
  //���ƶ�ģ�Ͱ�װ��֮ǰ���������߽ڵ����� adhocNodes ��
  mobilityAdhoc.Install (adhocNodes);
  streamIndex += mobilityAdhoc.AssignStreams (adhocNodes, streamIndex);//Ϊ�ƶ�ģ�ͷ������������
  NS_UNUSED (streamIndex); //����һ�㿪ʼ��streamIndex ���ٱ�ʹ��

  //һϵ��·��Э��İ����������Լ� Internet Э��ջ�İ���������
  AodvHelper aodv;
  OlsrHelper olsr;
  DsdvHelper dsdv;
  DsrHelper dsr;
  DsrMainHelper dsrMain;
  Ipv4ListRoutingHelper list;
  InternetStackHelper internet;

  switch (m_protocol)
    {
    case 1:
      list.Add (olsr, 100);
      m_protocolName = "OLSR";
      break;
    case 2:
      list.Add (aodv, 100);//���ȼ���Ϊ100
      m_protocolName = "AODV";
      break;
    case 3:
      list.Add (dsdv, 100);
      m_protocolName = "DSDV";
      break;
    case 4:
      m_protocolName = "DSR";
      break;
    default:
      NS_FATAL_ERROR ("No such protocol:" << m_protocol);
    }

  if (m_protocol < 4)
    {
      internet.SetRoutingHelper (list);//����·�ɸ�������
      internet.Install (adhocNodes);//��װ��Э��ջ�� adhocNodes ��
    }
  else if (m_protocol == 4)
    {
      internet.Install (adhocNodes);
      dsrMain.Install (dsr, adhocNodes);
    }

  NS_LOG_INFO ("assigning ip address");

  Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");//һ����������ʼ��ַ������
  Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces = addressAdhoc.Assign (adhocDevices);//����Щ��ַ����� adhocDevices �е��豸����������ĵ�ַ�洢��adhocInterfaces ��

  //�������� UDP ���ӵ� on/off Ӧ�ó��򣬸�Ӧ�ó����Ժ㶨��� 1.0 ��Ƶ�ʷ������ݣ����ڷ��������ݺ������ٴ��������͡�
  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));//Ӧ�ó���û�йر�ʱ�䣬��������

  //���ý��սڵ㣬���ͽڵ㣬��i+nSinks�ڵ㷢�͸���i���ڵ㡣i<nSinks
  for (int i = 0; i < nSinks; i++)
  {
	  //�����ض��ڵ���Ϊ���ݰ��Ľ����ߣ�sink��������һ��ָ�� Socket �����ָ�룬�ö������ڽ��մ�ָ����ַ���͵����ݰ�
      Ptr<Socket> sink = SetupPacketReceive (adhocInterfaces.GetAddress (i), adhocNodes.Get (i));

	  //����һ��Զ�̵�ַ������ΪĿ���ַ
      AddressValue remoteAddress (InetSocketAddress (adhocInterfaces.GetAddress (i), port));//port��ʼ��Ϊ9
      onoff1.SetAttribute ("Remote", remoteAddress);

	  //���÷��ͽڵ�
      Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
      ApplicationContainer temp = onoff1.Install (adhocNodes.Get (i + nSinks));//��װ�� On/Off Ӧ�ó���ָ���Ľڵ㣬����ģ�����ݵķ���
      //����Ӧ�ó����������ֹͣʱ��
	  temp.Start (Seconds (var->GetValue (50.0,51.0)));//ʹ�����������ȷ��Ӧ�ó��������ʱ�䣬����ʱ���� 50 �� 51 ��֮�䡣
      temp.Stop (Seconds (TotalTime));//������ʱ��80s
  }

  std::stringstream ss;
  ss << nWifis;
  std::string nodes = ss.str ();

  std::stringstream ss2;
  ss2 << nodeSpeed;
  std::string sNodeSpeed = ss2.str ();

  std::stringstream ss3;
  ss3 << nodePause;
  std::string sNodePause = ss3.str ();

  std::stringstream ss4;
  ss4 << rate;
  std::string sRate = ss4.str ();

  //NS_LOG_INFO ("Configure Tracing.");
  //tr_name = tr_name + "_" + m_protocolName +"_" + nodes + "nodes_" + sNodeSpeed + "speed_" + sNodePause + "pause_" + sRate + "rate";

  // AsciiTraceHelper ascii;
  // Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream ( (tr_name + ".tr").c_str());
  // wifiPhy.EnableAsciiAll (osw);

  // AsciiTraceHelper ascii;
  // MobilityHelper::EnableAsciiAll (ascii.CreateFileStream (tr_name + ".mob"));

  uint32_t rxPacketsum = 0;
  double Delaysum = 0; 
  uint32_t txPacketsum = 0;
  uint32_t txBytessum = 0;
  uint32_t rxBytessum = 0;
  double txTimeFirst = 0;
  double rxTimeLast = 0;
  uint32_t lostPacketssum = 0;

  //ʹ�� FlowMonitor �����ӷ�������е��������
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();


  NS_LOG_INFO ("Run Simulation.");

  Simulator::Stop (Seconds (TotalTime));//���÷�����ʱ��
  Simulator::Run ();//��ʼ��������

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();// ��ȡ������������ͳ����Ϣ
  
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
  {//���������������FlowMonitor���е�ÿ������i->first ��ʾ���ı�ʶ��FlowId����i->second ��������������ص�ͳ����Ϣ
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);//����flowid�ҵ���������������Ԫ����Ϣ��Դ IP ��ַ��Ŀ�� IP ��ַ��Դ�˿ڡ�Ŀ��˿ڡ�Э�����ͣ�
    if(t.sourcePort==654){//Դ�˿��Ƿ�Ϊ 654������ǣ�������
      continue;
    }
    //�ۼӽ��պͷ��͵����ݰ������������ֽ������ӳ��ܺ��Լ���ʧ�����ݰ�����
    rxPacketsum += i->second.rxPackets;
    txPacketsum += i->second.txPackets;
    txBytessum += i->second.txBytes;
    rxBytessum += i->second.rxBytes;
    Delaysum += i->second.delaySum.GetSeconds();
    lostPacketssum += i->second.lostPackets;
    //��¼�׸��������ݰ���ʱ������һ���������ݰ���ʱ��
    if(txTimeFirst == 0)
    {
      txTimeFirst = i->second.timeFirstTxPacket.GetSeconds();
    }
    
    rxTimeLast = i->second.timeLastRxPacket.GetSeconds();
  }
  //�� FlowMonitor ���������л�Ϊ XML �ļ��������䱣�浽��Ϊ tr_name + ".flowmon" ���ļ���
  monitor->SerializeToXmlFile ((tr_name + ".flowmon").c_str(), false, false);

  double timeDiff = (rxTimeLast - txTimeFirst);

  std::cout << "\n\n";
  std::cout << "Total Tx Packets: " << txPacketsum << "\n";
  std::cout << "Total Rx Packets: " << rxPacketsum << "\n";
  std::cout << "Total Packets Lost: " << lostPacketssum << "\n";
  std::cout << "Throughput: " << ((rxBytessum * 8.0) / timeDiff)/1024<<" Kbps"<<"\n";
  std::cout << "Packets Delivery Ratio: " << (double)((rxPacketsum * 100.0) /txPacketsum) << "%" << "\n";
  std::cout << "Packets Loss Ratio: " << (double)((lostPacketssum * 100.0) /txPacketsum) << "%" << "\n";
  std::cout << "Avg End to End Delay: " << Delaysum/rxPacketsum << "\n";

  std::ofstream myfile;
  myfile.open ("DATA_AODV_TOPOLOGY.txt", std::ios::app);
  myfile
        <<nWifis<<" "
        <<nSinks<<" "
        <<lostPacketssum<<" "
        <<((rxBytessum * 8.0) / timeDiff)/1024<<" "
        <<(double)((rxPacketsum * 100.0) /txPacketsum)<<" "
        <<(double)((lostPacketssum * 100.0) /txPacketsum)<<" "
        <<Delaysum/rxPacketsum<<std::endl;
  myfile.close();

  Simulator::Destroy ();
}

