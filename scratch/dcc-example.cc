/*
 * Copyright (c) 2020, Universidad Miguel Hernandez de Elche (UMH)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Jorge Mira <jorge.mira01@alu.umh.es> (main contributor)
 * 	    Gokulnath Thandavarayan <gthandavarayan@umh.es>
 * 	    Miguel Sepulcre <msepulcre@umh.es>
 *          Javier Gozalvez <j.gozalvez@umh.es>
 *
 */

//
// This script, adapted from examples/wireless/wifi-simple-adhoc illustrates
// the use of DCC.
//
// ./waf --run dcc-example
//
// ./waf --run "dcc-example --Algorithm=Reactive"

#include "common-functions.h"

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/dcc-module.h"
#include "ns3/applications-module.h"
#include "ns3/dcc-module.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/arp-cache.h"

#include "ns3/integer.h"
#include "ns3/string.h"
#include "ns3/buffer.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("DCC Prueba");


#define N_STEPS_METRIC 200
#define N_LAST_STEP 99
#define N_LAST_DISTANCE_STEP 990

bool cluster = false;


int LineSize = 5000;

uint32_t NumCars;
double density;
double spd;
int X_NumLines;
int X_lineSize;
int Y_NumCells; //initially Y_NumCells = 100
int Y_CellsSize; //initially Y_CellsSize = 4


std::string DCC_Algorithm;
int alg;


int speed = 0;
int RngSeed;              //Random number generator's seed
int RngRun;
int iteration;


int CAM_Priority = 3;
int CPM_Priority = 1;


void ReceivePacket (Ptr<Socket> socket)
{
	//SocketPriorityTag qos;
	Ptr<Packet> packet;
	Ptr<Packet> CAM;
	Ptr<Packet> CPM;
	packet = socket->Recv();

}

void GeneratePacketsCPM(Ptr<Socket> socket, uint32_t pktSize,
						Time inicial, Time pktInterval){

	SocketPriorityTag qos;
	qos.SetPriority(CPM_Priority);

	Ptr<Packet> packet = Create<Packet>(pktSize);
	packet->AddPacketTag(qos);
	socket->Send(packet);

	Simulator::Schedule(pktInterval, &GeneratePacketsCPM,
						socket, pktSize, inicial, pktInterval);

}


static void GeneratePacketsCAM (Ptr<Socket> socket, uint32_t pktSize,
                             Time inicial, Time pktInterval )
{

	Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
	SocketPriorityTag qos;

	Ptr<Packet> packet = Create<Packet>(pktSize);

	qos.SetPriority(CAM_Priority);
	packet->AddPacketTag(qos);
	socket->Send (packet);

	Simulator::Schedule (pktInterval, &GeneratePacketsCAM,
							   socket,pktSize, inicial, pktInterval);

}



int main (int argc, char *argv[])
{
  std::cout << "dcc-example"  << std::endl;
	
  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  uint32_t CAMSize = 350; // Bytes
  uint32_t CPMSize = 261; //Bytes
  double CAMinterval = 0.333333; // Interval between CAMs (Seconds)
  double CPMinterval = 0.117647; //Interval between CPMs (Seconds)

  double inicio = 0;
  Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> rand2 = CreateObject<UniformRandomVariable> ();

  // Convert to time object
   Time inicialTime = Seconds(inicio);
   Time interCAMInterval = Seconds(CAMinterval);
   Time interCPMInterval = Seconds(CPMinterval);
   int simulationtime = 6;

   //DCC_Algorithm = ("Reactive");
   NumCars = 2;
   spd = 19.44; //speed, 19.44 or 38.88 (m/s)


   RngSeed = 6;


  CommandLine cmd;
  // Hay que poner densidad / algoritmo / velocidad /

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("NumCars", "Number of cars", NumCars);
  cmd.AddValue ("Speed", "Speed of cars", speed);
  cmd.AddValue("Algorithm", "algoritmo", DCC_Algorithm); // GOKUL: NoDCC, Reactive, Adaptive
  cmd.AddValue("Alg", "algoritmo", alg); // GOKUL: 0 NoDCC, 1 Reactive, 2 Adaptive.
  cmd.AddValue("simulationTime", "Simulation time in seconds", simulationtime);

  cmd.Parse (argc, argv);


  // disable fragmentation for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  // turn off RTS/CTS for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue (phyMode));


  RngSeedManager::SetSeed (RngSeed);  // Changes seed from default of 1 to X
  RngSeedManager::SetRun (RngRun);


  NodeContainer DSRCnodes;
  DSRCnodes.Create (NumCars);


  //Configuración capa PHY
  YansWifiPhyHelper DSRCPhy =  YansWifiPhyHelper::Default ();


  DSRCPhy.Set ("TxPowerStart",DoubleValue (23.0));
  DSRCPhy.Set ("TxPowerEnd", DoubleValue (23.0));
  DSRCPhy.Set ("RxNoiseFigure", DoubleValue (9));


  DSRCPhy.Set ("CcaMode1Threshold", DoubleValue (-85));
  DSRCPhy.Set ("EnergyDetectionThreshold", DoubleValue (-85));

  YansWifiChannelHelper DSRCChannel;

  DSRCChannel.AddPropagationLoss ("ns3::WINNERB1plusFreeWayPropagationLossModel", //
  								  "Frequency", DoubleValue (5.9e9),
  								  "StdShadowFading", DoubleValue(3.0));




  DSRCChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  Ptr<YansWifiChannel> channel = DSRCChannel.Create ();
  DSRCPhy.SetChannel (channel);

  //Configuración Capa MAC
  NqosWaveMacHelper wifiMac = NqosWaveMacHelper::Default ();
  wifiMac.SetType ("ns3::OcbWifiMac", "QosSupported", BooleanValue (false));
  Wifi80211pHelper wifi = Wifi80211pHelper::Default ();
  wifi.SetStandard(WIFI_PHY_STANDARD_80211_10MHZ);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
								"DataMode",StringValue (phyMode),
								"ControlMode",StringValue (phyMode),
								"NonUnicastMode", StringValue (phyMode));

  NetDeviceContainer devices;
  devices = wifi.Install (DSRCPhy, wifiMac, DSRCnodes);


  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  int X = 0;
  int Y = 0;
  /*Setting initial position of nodes: 'Randomly'*/
  for (uint n = 0 ; n < NumCars ; n++ )
  {
	  positionAlloc->Add (Vector (X, Y, 0.0));
	  Y += LineSize / NumCars;
  }

  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (DSRCnodes);

  density = NumCars / (LineSize/1000);


  InternetStackHelper internet;

  ///////////////////////////////////////////////////////////////////////
  ////////////////// CONFIGURE AND INSTALL DCC //////////////////////////
  DccMainHelper dccMain;
  DccHelper dcc;

  dcc.Set("MaxMaintTime", TimeValue(Seconds(1)));
  dcc.Set("MaxMaintTimeCAM", TimeValue(Seconds(1)));
  dcc.Set("MaxMaintTimeCPM", TimeValue (Seconds(1)));

  /*if(alg == 0){
	  DCC_Algorithm = ("NoDCC");
	  dcc.Set("Algorithm", StringValue("NoDCC"));
	  dcc.Set("m_Alg", IntegerValue(alg));
  }else if(alg == 1){
	  DCC_Algorithm = ("Reactive");
	  dcc.Set("Algorithm", StringValue("Reactive"));
	  dcc.Set("m_Alg", IntegerValue(alg));
  }else if(alg == 2){
	  DCC_Algorithm = ("Adaptive");
	  dcc.Set("Algorithm", StringValue("Adaptive"));
	  dcc.Set("m_Alg", IntegerValue(alg));
  }*/


  if(DCC_Algorithm == "NoDCC"){
	  dcc.Set("Algorithm", StringValue("NoDCC"));
	  dcc.Set("m_Alg", IntegerValue(0));
  }else if(DCC_Algorithm == "Reactive"){
	  dcc.Set("Algorithm", StringValue("Reactive"));
	  dcc.Set("m_Alg", IntegerValue(1));
  }else if(DCC_Algorithm == "Adaptive"){
	  dcc.Set("Algorithm", StringValue("Adaptive"));
	  dcc.Set("m_Alg", IntegerValue(2));
  }

  // To Log.
  int cars = (int)NumCars;
  dcc.Set("NumCars", IntegerValue(cars));
  dcc.Set("density", DoubleValue(density));
  dcc.Set("spd", DoubleValue(spd));
  //dcc.Set("m_Alg", IntegerValue(alg));


  internet.Install(DSRCnodes);

  dccMain.Install(dcc, DSRCnodes);
  /////////////////////////////////////////////////////////////////////////////////////////////

  Ipv4AddressHelper address;
  NS_LOG_INFO("assign IP Addresses");
  address.SetBase("10.0.0.0", "255.255.0.0");
  Ipv4InterfaceContainer allInterfaces;
  allInterfaces = address.Assign(devices);

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  //PopulateArpCache();

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

	uint32_t i = 0;
	double start;
	for(i = 0; i < NumCars; i++){
		Ptr<NetDevice> device = devices.Get(i)->GetObject<WifiNetDevice>();
		Ptr<Node> node = device->GetNode();
		Ptr<WifiPhy> wifiPhy = devices.Get(i)->GetObject<WifiNetDevice>()->GetPhy();
		//wifiPhy->SetNodeStatus(true);//Para Jorge: Descomentar esta línea para ns-3 de TransAid (la versión que usa GOKUL)
		Ptr<Socket> DSRCsocket = Socket::CreateSocket (node, tid);
		InetSocketAddress local = InetSocketAddress (allInterfaces.GetAddress(i,0), 80);
		DSRCsocket->Bind (local);
		DSRCsocket->BindToNetDevice(device);
		DSRCsocket->SetRecvCallback (MakeCallback (&ReceivePacket));
		DSRCsocket->SetAllowBroadcast (true); //Broadcast Mode

		InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);//Broadcast Mode
		DSRCsocket->Connect(remote);

		var->SetAttribute("Min", DoubleValue(0));
		var->SetAttribute("Max", DoubleValue(1));

		start = var->GetValue();
		Simulator::Schedule(Seconds(start), &GeneratePacketsCAM, DSRCsocket, CAMSize, inicialTime,
										  interCAMInterval);

		Simulator::Schedule(Seconds(start + var->GetValue()), &GeneratePacketsCPM, DSRCsocket, CPMSize, inicialTime,
							interCPMInterval);
	}


  Simulator::Stop (Seconds(simulationtime));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
