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

NS_LOG_COMPONENT_DEFINE ("manet-routing-compare");//定义名为manet-routing-compare的日志组件

class RoutingExperiment
{//包含了实验所需的方法和变量
public:
  RoutingExperiment ();
  //负责运行实验，接受三个参数：nWifis-WiFi节点数量，nSinks-sink节点数量，txp-传输功率
  void Run (int nWifis, int nSinks, double txp);

  void CommandSetup (int argc, char **argv);//处理命令行参数

private:
  //用于设置节点接收数据的socket。两个参数：addr是节点的IPv4地址，node是节点的指针
  //函数中创建了一个UDP的socket，并设置了回调函数，用于接收数据。
  Ptr<Socket> SetupPacketReceive (Ipv4Address addr, Ptr<Node> node);
  //作为回调函数被用于处理收到的数据包。接受一个 Socket 类型的指针作为参数。
  //当socket接收到数据时，会调用这个函数来处理接收到的数据包。
  void ReceivePacket (Ptr<Socket> socket);

  uint32_t port;//socket的端口号
  uint32_t bytesTotal;//总共传输的字节数量
  uint32_t packetsReceived;//收到的数据包数量

  std::string m_protocolName;//当前使用的路由协议的名称
  bool m_traceMobility;//是否启用了节点移动性的追踪
  uint32_t m_protocol;// 使用哪种路由协议进行仿真实验
};

RoutingExperiment::RoutingExperiment ()
  : port (9),
    bytesTotal (0),
    packetsReceived (0),
    m_traceMobility (false),
    m_protocol (2) // AODV
{
}

//生成接收到数据包的日志信息
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

//设置一个接收数据包的Socket，并绑定到特定的地址和端口上
Ptr<Socket>
RoutingExperiment::SetupPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
//根据字符串 "ns3::UdpSocketFactory" 获取相应的 TypeId，用于创建一个UDP类型的Socket
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);//创建了一个Socket对象 sink，使用了之前获取的 UDP Socket 的 TypeId，将其绑定到指定的节点 node 上
  InetSocketAddress local = InetSocketAddress (addr, port);//创建了一个 InetSocketAddress 对象 local，表示要绑定的地址和端口
  sink->Bind (local);// 将创建的Socket sink 绑定到指定的地址和端口上，以便它可以接收发送到这个地址和端口的数据包
  sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceivePacket, this));//设置了当Socket接收到数据包时的回调函数

  return sink;//返回创建的Socket对象 sink，以便在其他地方使用它接收数据包
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
  Packet::EnablePrinting ();//启用数据包的打印功能
  double TotalTime = 80.0;//仿真总时间
  std::string rate ("2048bps");//数据传输速率,设置为2048可能是为了模拟一个低速网络环境
  std::string phyMode ("DsssRate11Mbps");//物理模式为 DSSS，速率为 11 Mbps
  std::string tr_name ("AODV_Topology_Trace");//追踪文件名
  int nodeSpeed = 20; //节点移动速度m/s
  int nodePause = 0; //暂停时间s
  m_protocolName = "protocol";

  Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("512"));//设置OnOffApplication 应用程序的数据包大小为 512 字节
  Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (rate));//设置 OnOff 应用的数据传输速率为之前定义的 rate

  //Set Non-unicastMode rate to unicast mode
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode));//设置了 Wi-Fi 管理器的非单播模式为之前定义的 phyMode

  NodeContainer adhocNodes;
  adhocNodes.Create (nWifis);//创建了 nWifis 个节点的节点容器

  // setting up wifi phy and channel using helpers
  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);//创建了一个名为 wifi 的 WifiHelper 对象，并设置了 Wi-Fi 标准为 802.11b

  YansWifiPhyHelper wifiPhy;//YANS 是 NS-3 中用于模拟无线网络的物理层模型之一，用于模拟 WiFi 设备的物理层行为。
  YansWifiChannelHelper wifiChannel;//创建了 Wi-Fi 的物理层和信道属性的帮助器对象
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");//设置信道传播延迟模型为恒定速度传播延迟模型
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");//添加了 Friis 传播损耗模型，这是一种基于距离的常用传播损耗模型
  wifiPhy.SetChannel (wifiChannel.Create ());//将之前设置好的信道应用到 Wi-Fi 的物理层中

  // Add a mac and disable rate control
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));

  wifiPhy.Set ("TxPowerStart",DoubleValue (txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txp));//设置 Wi-Fi 的发送功率起始值和结束值为 txp

  wifiMac.SetType ("ns3::AdhocWifiMac");//将 Wi-Fi MAC 层类型设置为 Adhoc 模式，表明节点将在自组网模式下进行通信
  NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, adhocNodes);//安装 Wi-Fi 的物理层和 MAC 层到之前创建的节点容器 adhocNodes 中，并将创建的设备放入 adhocDevices 中

  MobilityHelper mobilityAdhoc;//移动模型帮助器对象
//以确保在不同场景中获得一致的移动性。streamIndex作伪随机数生成器的种子。
  //在相同的场景或条件下，如果多次运行模拟且种子相同，伪随机数生成器将以相同的顺序生成相同的随机数序列，进而导致相同的移动性模式。
  //可以确保在多次运行模拟的情况下，在相同的条件下得到相同的移动性，以便进行一致的模拟和比较。
  int64_t streamIndex = 0; 

  //使得节点在一个 500x500 的矩形区域内随机分布
  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
  pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));

  Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();//位置分配器
  streamIndex += taPositionAlloc->AssignStreams (streamIndex);//为位置分配器分配了随机数流，这些随机数流用于生成节点的位置和移动参数中的随机变量。

  //两个字符串流 ssSpeed 和 ssPause，分别用于生成节点的移动速度和暂停时间的随机变量
  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";//最小值0，最大值nodeSpeed=20m/s
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";//恒为nodePause=0
  //设置节点的移动模型为随机航点移动模型，并传入了速度、暂停时间以及之前创建的位置分配器。
  mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc));
  //为移动模型设置位置分配器
  mobilityAdhoc.SetPositionAllocator (taPositionAlloc);
  //将移动模型安装到之前创建的无线节点容器 adhocNodes 中
  mobilityAdhoc.Install (adhocNodes);
  streamIndex += mobilityAdhoc.AssignStreams (adhocNodes, streamIndex);//为移动模型分配了随机数流
  NS_UNUSED (streamIndex); //从这一点开始，streamIndex 不再被使用

  //一系列路由协议的帮助器对象以及 Internet 协议栈的帮助器对象
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
      list.Add (aodv, 100);//优先级设为100
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
      internet.SetRoutingHelper (list);//设置路由辅助程序
      internet.Install (adhocNodes);//安装该协议栈到 adhocNodes 上
    }
  else if (m_protocol == 4)
    {
      internet.Install (adhocNodes);
      dsrMain.Install (dsr, adhocNodes);
    }

  NS_LOG_INFO ("assigning ip address");

  Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");//一个子网的起始地址，掩码
  Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces = addressAdhoc.Assign (adhocDevices);//将这些地址分配给 adhocDevices 中的设备，并将分配的地址存储在adhocInterfaces 中

  //用于配置 UDP 连接的 on/off 应用程序，该应用程序以恒定间隔 1.0 的频率发送数据，并在发送完数据后立即再次启动发送。
  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));//应用程序没有关闭时间，持续发送

  //设置接收节点，发送节点，第i+nSinks节点发送给第i个节点。i<nSinks
  for (int i = 0; i < nSinks; i++)
  {
	  //设置特定节点作为数据包的接收者（sink），返回一个指向 Socket 对象的指针，该对象用于接收从指定地址发送的数据包
      Ptr<Socket> sink = SetupPacketReceive (adhocInterfaces.GetAddress (i), adhocNodes.Get (i));

	  //构造一个远程地址，设置为目标地址
      AddressValue remoteAddress (InetSocketAddress (adhocInterfaces.GetAddress (i), port));//port初始化为9
      onoff1.SetAttribute ("Remote", remoteAddress);

	  //设置发送节点
      Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
      ApplicationContainer temp = onoff1.Install (adhocNodes.Get (i + nSinks));//安装了 On/Off 应用程序到指定的节点，用于模拟数据的发送
      //设置应用程序的启动和停止时间
	  temp.Start (Seconds (var->GetValue (50.0,51.0)));//使用随机变量来确定应用程序的启动时间，启动时间在 50 到 51 秒之间。
      temp.Stop (Seconds (TotalTime));//仿真总时间80s
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

  //使用 FlowMonitor 来监视仿真过程中的流量情况
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();


  NS_LOG_INFO ("Run Simulation.");

  Simulator::Stop (Seconds (TotalTime));//设置仿真总时间
  Simulator::Run ();//开始仿真运行

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();// 获取流量监视器的统计信息
  
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
  {//遍历流量监控器（FlowMonitor）中的每个流，i->first 表示流的标识（FlowId），i->second 则包含了与该流相关的统计信息
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);//根据flowid找到与该流相关联的五元组信息（源 IP 地址、目标 IP 地址、源端口、目标端口、协议类型）
    if(t.sourcePort==654){//源端口是否为 654，如果是，则跳过
      continue;
    }
    //累加接收和发送的数据包数量、数据字节数、延迟总和以及丢失的数据包数量
    rxPacketsum += i->second.rxPackets;
    txPacketsum += i->second.txPackets;
    txBytessum += i->second.txBytes;
    rxBytessum += i->second.rxBytes;
    Delaysum += i->second.delaySum.GetSeconds();
    lostPacketssum += i->second.lostPackets;
    //记录首个发送数据包的时间和最后一个接收数据包的时间
    if(txTimeFirst == 0)
    {
      txTimeFirst = i->second.timeFirstTxPacket.GetSeconds();
    }
    
    rxTimeLast = i->second.timeLastRxPacket.GetSeconds();
  }
  //将 FlowMonitor 的数据序列化为 XML 文件，并将其保存到名为 tr_name + ".flowmon" 的文件中
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

