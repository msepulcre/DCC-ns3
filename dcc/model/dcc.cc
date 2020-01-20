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
 *          UWICORE laboratory, http://www.uwicore.umh.es
 *          Universidad Miguel Hernandez de Elche (UMH), Spain
*/
 

#include "dcc.h"
#include "dcc-buffer.h"

#include <list>
#include <ctime>
#include <map>
#include <limits>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include "ns3/random-variable-stream.h"
#include "ns3/config.h"
#include "ns3/enum.h"
#include "ns3/string.h"
#include "ns3/ptr.h"
#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/uinteger.h"
#include "ns3/net-device.h"
#include "ns3/packet.h"
#include "ns3/boolean.h"
#include "ns3/node-list.h"
#include "ns3/double.h"
#include "ns3/pointer.h"
#include "ns3/timer.h"
#include "ns3/object-vector.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv4-header.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-route.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/icmpv4-l4-protocol.h"
#include "ns3/adhoc-wifi-mac.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy-state-helper.h"
#include "ns3/wifi-phy.h"
#include "ns3/inet-socket-address.h"
#include "ns3/udp-l4-protocol.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/tcp-socket-factory.h"
#include "ns3/llc-snap-header.h"
#include "ns3/arp-header.h"
#include "ns3/ipv6-interface.h"

#include "ns3/qos-utils.h"

namespace ns3 {



NS_LOG_COMPONENT_DEFINE ("Dcc");

namespace dcc {

NS_OBJECT_ENSURE_REGISTERED (Dcc);
const uint8_t Dcc::PROT_NUMBER = 50;


TypeId Dcc::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::dcc::Dcc")
    .SetParent<Object>()
    .SetGroupName ("Dcc")
    .AddConstructor<Dcc> ()
    .AddAttribute ("MaxMaintLen",
                   "Maximum number of packets that can be stored "
                   "in maintenance buffer.",
                   UintegerValue (10),
                   MakeUintegerAccessor (&Dcc::m_maxMaintainLen),
                   MakeUintegerChecker<uint32_t> ())

    .AddAttribute ("MaxMaintTime",
                   "Maximum time packets can be queued in maintenance buffer.",
                   TimeValue (Seconds (10)),
                   MakeTimeAccessor (&Dcc::m_maxMaintainTime),
                   MakeTimeChecker ())
     .AddAttribute ("MaxMaintTimeCAM",
    		 	 	"Maximum time CAM packets can be queued in maintenance buffer.",
					TimeValue (Seconds (0.1)),
					MakeTimeAccessor (&Dcc::m_maxMaintainTimeCAM),
					MakeTimeChecker ())
	 .AddAttribute ("MaxMaintTimeCPM",
			        "Maximum time CPM packets can be queued in maintenance buffer.",
					TimeValue (Seconds (0.1)),
					MakeTimeAccessor (&Dcc::m_maxMaintainTimeCPM),
					MakeTimeChecker ())
    .AddAttribute ("Algorithm",
                   "Type of algorithm. Reactive or Adaptive",
                   StringValue ("Reactive"),
                   MakeStringAccessor (&Dcc::Algorithm),
                   MakeStringChecker ())
   .AddAttribute ("m_Alg",
				  "Type of algorithm. Reactive or Adaptive",
				  IntegerValue (1),
				  MakeIntegerAccessor (&Dcc::m_algorithm),
				  MakeIntegerChecker<int> ())
    .AddAttribute ("CBRtarget",
                   "CBR target for adaptive algorithm",
                   DoubleValue (0.68),
                   MakeDoubleAccessor (&Dcc::CBR_target),
                   MakeDoubleChecker<double>())

    .AddAttribute ("NumCars",
				  "Number of cars in the simulation",
				  IntegerValue (4),
				  MakeIntegerAccessor (&Dcc::NumCars),
				  MakeIntegerChecker<int> ())
	.AddAttribute ("MargenAleatorio",
				  "Number of cars in the simulation",
				  IntegerValue (0),
				  MakeIntegerAccessor (&Dcc::margen_aleatorio),
				  MakeIntegerChecker<int> ())
    .AddAttribute ("spd",
				  "Speed",
				  DoubleValue (4),
				  MakeDoubleAccessor (&Dcc::spd),
				  MakeDoubleChecker<double> ())

    .AddAttribute ("density",
					"vehicle per km",
					DoubleValue (4),
				  MakeDoubleAccessor (&Dcc::density),
				  MakeDoubleChecker<double> ())
//GOKUL
.AddTraceSource ("DCCTOFF"," Computed Toff ",MakeTraceSourceAccessor (&Dcc::m_Toff))
.AddTraceSource ("DCCTxDCC", "Transmitted packets from DCC ",            MakeTraceSourceAccessor (&Dcc::m_TxDCC))
.AddTraceSource ("DCCTxALL", "Generated packets from application layer",MakeTraceSourceAccessor (&Dcc::m_TxALL))
//GOKUL

;

  return tid;
}


// Constructor
Dcc::Dcc()
{
  NS_LOG_FUNCTION_NOARGS ();

  CBR_Timer.SetFunction (&Dcc::CBR_TimerExpire, this);
  Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();

  var->SetAttribute("Min", DoubleValue(0));
  var->SetAttribute("Max", DoubleValue(1000000));
  uint8_t start = var->GetValue();

  CBR_Timer.Schedule (MicroSeconds(start));
 }

Dcc::~Dcc ()
{
  NS_LOG_FUNCTION_NOARGS ();
}


void
Dcc::NotifyNewAggregate ()
{
  NS_LOG_FUNCTION (this << "NotifyNewAggregate");
  if (m_node == 0)
    {
      Ptr<Node> node = this->GetObject<Node> ();

      if (node != 0)
        {
          m_ipv4 = this->GetObject<Ipv4L3Protocol> ();
          if (m_ipv4 != 0)
            {
              this->SetNode (node);
              m_ipv4->Insert (this);
              this->SetDownTarget (MakeCallback (&Ipv4L3Protocol::Send, m_ipv4));
              std::basic_ostringstream<char> oss;
              oss << "/NodeList/" << m_node->GetId() << "/DeviceList/*/$ns3::WifiNetDevice/Phy/State/State";
              std::string var = oss.str();
              Config::ConnectWithoutContext(var, MakeCallback(&Dcc::ChannelBusyRatioProbing, this));
            }

          m_ip = node->GetObject<Ipv4> ();

          if (m_ip != 0)
            {
              NS_LOG_DEBUG ("Ipv4 started");
            }
        }
    }
  IpL4Protocol::NotifyNewAggregate ();

  Simulator::ScheduleNow (&Dcc::Start, this);
}

//GOKUL
void
Dcc::NotifyDCCTOFF (Ptr<const Packet> packet, double Time, uint32_t Sender_ID, uint32_t T_off)
{
	m_Toff (packet, Time, Sender_ID, T_off);
}

void
Dcc::NotifyDCCTXDCC (Ptr<const Packet> packet, double Time, uint32_t Sender_ID, uint32_t T_off)
{
	m_TxDCC (packet, Time, Sender_ID, T_off);
}

void
Dcc::NotifyDCCTXALL (Ptr<const Packet> packet, double Time, uint32_t Sender_ID, uint32_t T_off)
{
	m_TxALL (packet, Time, Sender_ID, T_off);
}
//GOKUL

void Dcc::Start ()
{
  NS_LOG_FUNCTION (this << "Start Descentralized Congestion Control DCC");

  m_maintainBuffer.SetMaxQueueLen (m_maxMaintainLen);
  m_maintainBuffer.SetMaintainBufferTimeout (m_maxMaintainTime);
  m_maintainBuffer.SetMaintainBufferTimeoutCAM (m_maxMaintainTimeCAM);
  m_maintainBuffer.SetMaintainBufferTimeoutCPM (m_maxMaintainTimeCPM);
  m_maintainBuffer.SetNode(m_node);
  m_maintainBuffer.SetNumCAM(0);
  m_maintainBuffer.SetNumCPM(0);
  m_maintainBuffer.SetNumCars(NumCars);
  m_maintainBuffer.SetMargenAleatorio(margen_aleatorio);



  if (m_mainAddress == Ipv4Address ())
    {
      Ipv4Address loopback ("127.0.0.1");
      for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
        {

          Ipv4Address addr = m_ipv4->GetAddress (i, 0).GetLocal ();
          m_broadcast = m_ipv4->GetAddress (i, 0).GetBroadcast ();
          AlgorithmCheck();
          if (addr != loopback)
            {
              tiempoespera = 50;
              Toff = Time(MilliSeconds(0));
              UltimoEnvio = 0;
              primero = true;
              LastTxTime = Seconds(0);
              LastTx = 0;

              //Reactive parameters
              DccState = 0;
              CBR = 0;
              node_Not_Idle = Seconds(0);
              Last_DeltaUpdate = Seconds(0);
              CBR_PacketId = 0;

              //Adaptivfe parameters
              alpha = 0.016;
              beta = 0.0012;
              CBR_target = 0.68;
              delta_max = 0.03;
              delta_min = 0.0006;
              Gmax = 0.0005;
              Gmin = -0.00025;
              delta = 0;
              Tonpp = Seconds(0); //Duration of last transmission

              cluster = false;
			}
        }
    }
}



Ptr<NetDevice>
Dcc::GetNetDeviceFromContext (std::string context)
{
  std::vector <std::string> elements = GetElementsFromContext (context);
  Ptr<Node> n = NodeList::GetNode (atoi (elements[1].c_str ()));
  NS_ASSERT (n);
  return n->GetDevice (atoi (elements[3].c_str ()));
}

std::vector<std::string>
Dcc::GetElementsFromContext (std::string context)
{
  std::vector <std::string> elements;
  size_t pos1=0, pos2;
  while (pos1 != context.npos)
  {
    pos1 = context.find ("/",pos1);
    pos2 = context.find ("/",pos1+1);
    elements.push_back (context.substr (pos1+1,pos2-(pos1+1)));
    pos1 = pos2;
    pos2 = context.npos;
  }
  return elements;
}



void
Dcc::DoDispose (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  m_node = 0;
  IpL4Protocol::DoDispose ();
}


void
Dcc::SetNode (Ptr<Node> node)
{
  m_node = node;
}

Ptr<Node>
Dcc::GetNode () const
{
  NS_LOG_FUNCTION_NOARGS ();
  return m_node;
}



Ptr<Node>
Dcc::GetNodeWithAddress (Ipv4Address ipv4Address)
{
  NS_LOG_FUNCTION (this << ipv4Address);
  int32_t nNodes = NodeList::GetNNodes ();
  for (int32_t i = 0; i < nNodes; ++i)
    {
      Ptr<Node> node = NodeList::GetNode (i);
      Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
      int32_t ifIndex = ipv4->GetInterfaceForAddress (ipv4Address);
      if (ifIndex != -1)
        {
          return node;
        }
    }
  return 0;
}


Ipv4Address
Dcc::GetIPfromMAC (Mac48Address address)
{
  NS_LOG_FUNCTION (this << address);
  int32_t nNodes = NodeList::GetNNodes ();
  for (int32_t i = 0; i < nNodes; ++i)
    {
      Ptr<Node> node = NodeList::GetNode (i);
      Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
      Ptr<NetDevice> netDevice = ipv4->GetNetDevice (1);

      if (netDevice->GetAddress () == address)
        {
          return ipv4->GetAddress (1, 0).GetLocal ();
        }
    }
  return 0;
}


uint16_t
Dcc::GetIDfromIP (Ipv4Address address)
{
  int32_t nNodes = NodeList::GetNNodes ();
  for (int32_t i = 0; i < nNodes; ++i)
    {
      Ptr<Node> node = NodeList::GetNode (i);
      Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
      if (ipv4->GetAddress (1, 0).GetLocal () == address)
        {
          return uint16_t (i);
        }
    }
  return 256;
}

Ipv4Address
Dcc::GetIPfromID (uint16_t id)
{
  if (id >= 256)
    {
      NS_LOG_DEBUG ("Exceed the node range");
      return "0.0.0.0";
    }
  else
    {
      Ptr<Node> node = NodeList::GetNode (uint32_t (id));
      Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
      return ipv4->GetAddress (1, 0).GetLocal ();
    }
}



int
Dcc::GetProtocolNumber (void) const
{
  // / This is the protocol number for DCC which is 48
  return PROT_NUMBER;
}


//------------------------------------------------------ FUNCIONES PARA ALMACENAR INFO----------------------------------------------------------------


/*Cada vez que se envia un mensaje, se almacena info sobre: Tiempo de tx, última estimación del CBR, DCC state, Toffactual,
  ID del vehículo y prioridad y tamaño del mensaje
  */
void Dcc::SaveTxRateFile(Ptr<Packet> packet){

		  //GOKUL
			double Time = (Simulator::Now ()).GetSeconds ();
			uint32_t Sender_ID 	= m_node->GetId();
			uint32_t T_off 	= Toff.GetMilliSeconds();
		    NotifyDCCTXDCC(packet, Time, Sender_ID,T_off);
}

/*
 * Cada vez que un vehículo recibe un mensaje se almacena info sobre:
 * Tiempo de rx, última estimación del CBR, DCC state, Toffactual,
 * ID del vehículo y prioridad y tamaño del mensaje
*/

void
Dcc::SavePktRx(Ptr<Packet> packet){

}

/* Almacena nueva estimación del CBR, nuevo Toff, ID del vehículo, Tiempo actual y DCC state*/
void
Dcc::SaveCBR(){

}

/* Función para almacenar Toff cuando se emplea el algoritmo Adaptive.*/
void
Dcc::SaveToff(){ //Save Toff when Adaptive

}

/*
 *  Función para habilitar almecanamiento de información
 */
bool
Dcc::Save(){

	return true;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------


/*
 * Función para enviar. Cada vez que un mensaje de las capas
 * superiores genera un mensaje, el mensaje entra a la clase DCC
 * a través de esta función.
 * Se comprueba si se cumple el Toff. En caso de no cumplirse,
 * el mensaje se almacena en las DCC Queue dependiendo de su
 * prioridad. Si la condición de Toff se satisface, el mensaje es enviado
 */
void
Dcc::Send (Ptr<Packet> packet,
                  Ipv4Address source,
                  Ipv4Address destination,
                  uint8_t protocol,
                  Ptr<Ipv4Route> route)
{
  NS_LOG_FUNCTION (this << packet << source << destination << (uint32_t)protocol << route);

  double now = Simulator::Now().GetDouble();
  Time waiting;
  SocketPriorityTag qos;
  DccHeader dccHeader;
  uint8_t priority;

  packet->PeekPacketTag(qos);
  priority = qos.GetPriority();

  dccHeader.SetNextHeader(protocol);

  packet->AddHeader(dccHeader);


  if(primero == true){
  	  primero = false;
  }

  //GOKUL
	double Time = (Simulator::Now ()).GetSeconds ();
	uint32_t Sender_ID 	= m_node->GetId();
	uint32_t T_off 	= Toff.GetMilliSeconds();
    NotifyDCCTXALL(packet, Time, Sender_ID,T_off);
//GOKUL

  // Toff checking
  if( (now - LastTx) < Toff.GetDouble() && priority > 1){
	  // Toff condition don't satisfy
	  waiting = NanoSeconds(now - LastTx);
	  //Packet is enqueued
	  DccBuffEntry newEntry(packet, Simulator::Now());
	  m_maintainBuffer.Enqueue(newEntry);
	  Simulator::ScheduleWithContext(m_node->GetId(), Toff - waiting, &Dcc::SendPacketQueue,this, source, destination, protocol);
  }else{
	  // Toff condition satisfy
	  DccBuffEntry newEntry;
	  if(m_maintainBuffer.Dequeue(newEntry) && priority > 1){
		  //Packet sent from queue
		  Ptr<Packet> packetQueue;
		  packetQueue = newEntry.GetPacket()->Copy();
		  //NS_LOG_UNCOND("Paquete enviado desde el buffer");

		  DccBuffEntry entry(packet, Simulator::Now());
		  m_maintainBuffer.Enqueue(entry);

		  if(Save() == true)
			  SaveTxRateFile(packetQueue);

		  if(Algorithm == "Adaptive"){
			  PacketDuration(packet);
			  ToffUpdateAfterTransmission();
		  }
		  LastTx = Simulator::Now().GetDouble();
		  Simulator::ScheduleWithContext(m_node->GetId(), Toff - waiting, &Dcc::SendPacketQueue,this, source, destination, protocol);
		  m_downTarget (packetQueue, source, destination, GetProtocolNumber(), 0);

	  }else{
		  //Queue empty. Packet is tx
		  LastTx = Simulator::Now().GetDouble();
		  if(Save() == true)
			  SaveTxRateFile(packet);

		  if(Algorithm == "Adaptive"){
			  PacketDuration(packet);
			  ToffUpdateAfterTransmission();
		  }
		  m_downTarget (packet, source, destination, GetProtocolNumber(), 0);

	  }
  }


}

/*
 * Send a packet from DCC Queues
 */
void Dcc::SendPacketQueue (Ipv4Address source, Ipv4Address destination, uint8_t protocol){

	Time waiting;
	double now;

	now = Simulator::Now().GetDouble();


	//Toff checking
	if(now - LastTx < Toff.GetDouble()){
		//Toff does not fullfil. Wait for next tx window
		waiting = NanoSeconds(now - LastTx);
		Simulator::ScheduleWithContext(m_node->GetId(), Toff - waiting, &Dcc::SendPacketQueue,this, source, destination, protocol);
	}else{
		//Toff condition satisfy
		DccBuffEntry newEntry;

		if(m_maintainBuffer.Dequeue(newEntry)){ //
			//Packet sent from queue
			Ptr<Packet> packetQueue;
			packetQueue = newEntry.GetPacket()->Copy();
			LastTx = Simulator::Now().GetDouble();

			if(Save() == true)
				SaveTxRateFile(packetQueue);


			if(Algorithm == "Adaptive"){
				PacketDuration(packetQueue);
				ToffUpdateAfterTransmission();
			}

			m_downTarget (packetQueue, source, destination, GetProtocolNumber(), 0);
		}
	}

}


/*
 * Envío de un mensaje cuando el nuevo Toff es menor que el Toff anterior
 */
void
Dcc::SendPacket (Ipv4Address source)
{

	DccBuffEntry newEntry;
	if(m_maintainBuffer.Dequeue(newEntry) == true){
		Ptr<Packet> packet;
		packet = newEntry.GetPacket()->Copy();
		LastTx = Simulator::Now().GetDouble();
		if(Save() == true)
			SaveTxRateFile(packet);

		if(Algorithm == "Adaptive"){
			PacketDuration(packet);
			ToffUpdateAfterTransmission();
		}
		m_downTarget(packet, source, m_broadcast, GetProtocolNumber(), 0);
	}
}

/*
 * Toff setting (Reactive DCC)
 */
void Dcc::CurrentState(){
	Time now = Simulator::Now();
	Time Toff_Before = Toff;

	StateUpdate();
	if(DccState == 0){
		Toff = MilliSeconds(50);
	}else if(DccState == 1){
		Toff = MilliSeconds(100);
	}else if(DccState == 2){
		Toff = MilliSeconds(200);
	}else if(DccState == 3){
		Toff = MilliSeconds(250);
	}else{
		Toff = MilliSeconds(1000);
	}

	// If new Toff is smaller than old Toff and bigger than LastTx. Send a packet.
	if(Toff.GetDouble() < Toff_Before.GetDouble()){
		if( (LastTx > 0) && (now.GetDouble() - LastTx > Toff.GetDouble() ) ){
			Ipv4Address ip = GetIPfromID(m_node->GetId());
			Ptr<Packet> pa;
			SendPacket(ip);
		}
	}
}


/*
 * Updates DCC state depending on new CBR estimation
 * Only active when Reactive algorithm is used
 */
void Dcc::StateUpdate(){
	NS_LOG_FUNCTION (this);

	//Updates state depending on CBR

	if(CBR < 0.30){
		switch(DccState){
		case 0:
			DccState = 0;
			break;
		default:
			DccState--;
			break;
		}
	}else if(CBR >= 0.30 && CBR < 0.40){

		switch(DccState){
		case 0:
			DccState = 1;
			break;
		case 1:
			DccState = 1;
			break;
		case 2:
			DccState =1;
			break;
		case 3:
			DccState = 2;
			break;
		case 4:
			DccState = 3;
			break;
		}
	}else if(CBR >= 0.4 && CBR < 0.50){

		switch(DccState){
		case 0:
			DccState = 1;
			break;
		case 1:
			DccState = 2;
			break;
		case 2:
			DccState = 2;
			break;
		case 3:
			DccState = 2;
			break;
		case 4:
			DccState = 3;
			break;
		}

	}else if(CBR >= 0.50 && CBR <= 0.65){

		switch(DccState){
			case 0:
				DccState = 1;
				break;
			case 1:
				DccState = 2;
				break;
			case 2:
				DccState = 3;
				break;
			case 3:
				DccState = 3;
				break;
			case 4:
				DccState = 3;
				break;
		}
	}else if(CBR > 0.65){

		switch(DccState){
			case 0:
				DccState = 1;
				break;
			case 1:
				DccState = 2;
				break;
			case 2:
				DccState = 3;
				break;
			case 3:
				DccState = 4;
				break;
			case 4:
				DccState = 4;
				break;

		}
	}

}

/*
 * Estimación del CBR
 */
void Dcc::ChannelBusyRatioProbing(Time start, Time duration, enum WifiPhy::State state){

	if(state != WifiPhy::IDLE && state != WifiPhy::SLEEP){
		node_Not_Idle = node_Not_Idle + duration;
	}
}

/*
 *
 */
void
Dcc::CBRupadte(){
	CBR_Before = CBR;
	CBR = (node_Not_Idle.GetDouble() / 100000000 + CBR_Before)*0.5;
	Time now = Simulator::Now();

	node_Not_Idle = NanoSeconds(0);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// I use this configuration for the cluster to be able to launch several simulations

	if(m_algorithm == 1){
		CurrentState();
	}else if(m_algorithm == 2){
		if(now.GetMicroSeconds() - Last_DeltaUpdate.GetMilliSeconds() > 199 ||
			now.GetMicroSeconds() - Last_DeltaUpdate.GetMilliSeconds() < 1 ){
			Last_DeltaUpdate = Simulator::Now();
			DeltaUpdate();
		}

	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
	if(Algorithm == "Reactive"){
		CurrentState();

		//Delta is updated each 200 ms (ETSI TS 102 687 V1.2.1 (2018-04) 5.4 Adaptive Approach)
	}else if( (Algorithm == "Adaptive") && (now.GetMicroSeconds() - Last_DeltaUpdate.GetMicroSeconds() > 199) &&
			(now.GetMicroSeconds() - Last_DeltaUpdate.GetMicroSeconds() < 1)){
		Last_DeltaUpdate = Simulator::Now();
		DeltaUpdate();
	}
*/
	if(Save() == true)
		SaveCBR();
}

// Timer for CBR updating (100 ms) (ETSI TS 102 687 V1.2.1 (2018-04))
void Dcc::CBR_TimerExpire(){
	if (CBR_Timer.IsRunning ())
	    {
	      CBR_Timer.Cancel ();
	    }
	  CBR_Timer.Schedule (Seconds(0.1));
	  CBRupadte();
}

//Adaptive approach
void Dcc::DeltaUpdate(){
	double delta_offset = 0;
	double aux = 0;
	//double delta;

	CBR_its = 0.5*CBR_its + 0.25* (CBR_Before + CBR); //Step 1

	aux = beta*(CBR_target - CBR_its);

	if((CBR_target - CBR_its) > 0){ // step 2

		if(aux > Gmax ){
			delta_offset = Gmax;
		}else {
			delta_offset = aux;
		}

	}else{

		if(aux > Gmin){
			delta_offset = aux;
		}else{
			delta_offset = Gmin;
		}
	}

	delta = (1-alpha) * delta + delta_offset; //step 3

	if(delta > delta_max) //step 4
		delta = delta_max;

	if(delta < delta_min) // step 5
		delta = delta_min;

	ToffUpdateAfterCBRupdate();
}


/*Función que estima la duración de la
 *última transmisión
 */
void Dcc::PacketDuration(Ptr<Packet> packet){
	uint32_t PacketSize = packet->GetSize();
	Tonpp = NanoSeconds((PacketSize*8) / 0.006);
	Tonpp = Tonpp + MicroSeconds(68);
}

/*Función que actualiza el Toff
 * después de una transmisión
 * (DCC Adaptive)
 */
void Dcc::ToffUpdateAfterTransmission(){

	if(Tonpp.GetDouble() / delta > 25*1000000){
		Toff = NanoSeconds(Tonpp.GetDouble() / delta);
	}else{
		Toff = NanoSeconds(25*1000000);
	}

	if(Toff.GetSeconds() > 1){
		Toff = Seconds(1);
	}
	if(Save() == true)
		SaveToff();
}
/*Función que actualiza Toff cada
 * dos actualizaciones de la
 * estimación de CBR
 */
void Dcc::ToffUpdateAfterCBRupdate(){

	// Necesito el Toff y el waiting
	double aux;
	double waiting;


	waiting = Simulator::Now().GetDouble() - (LastTx);

	aux = Tonpp.GetDouble()/delta * (Toff.GetDouble() - waiting) / Toff.GetDouble() + waiting;

	if(aux < 25*1000000){
		aux = 25*1000000;
	}

	if(aux > 1000000000){
		Toff = Seconds(1);
	}else{
		Toff = NanoSeconds(aux);
	}
	if(Save() == true)
		SaveToff();
}

/*
 * Función para escoger el algoritmo
 * DCC deseado
 */
void Dcc::AlgorithmCheck(){
	if( (Algorithm != "Adaptive") && (Algorithm != "Reactive") && (Algorithm != "NoDCC") ){
		NS_FATAL_ERROR ("DCC algorithm incorrect. Try:\nReactive \nAdaptive \nNoDCC \n");
	}
}


/*
 * Función por la que pasa un mensaje recibido
 * En versiones siguientes se empleará para
 * obtener el CBR global estimado por todos
 * los vehículos
 */
enum IpL4Protocol::RxStatus
Dcc::Receive (Ptr<Packet> p,
                     Ipv4Header const &ip,
                     Ptr<Ipv4Interface> incomingInterface)
{
	Ptr<Packet> packet = p->Copy();
	DccHeader dccHeader;
	SocketPriorityTag qos;
	packet->PeekPacketTag(qos);

	packet->RemoveHeader(dccHeader);


	uint8_t nextHeader = dccHeader.GetNextHeader();
	Ptr<Ipv4L3Protocol> l3proto = m_node->GetObject<Ipv4L3Protocol> ();
	Ptr<IpL4Protocol> nextProto = l3proto->GetProtocol (nextHeader);

	if (nextProto != 0)
	{
		// we need to make a copy in the unlikely event we hit the
		// RX_ENDPOINT_UNREACH code path
		// Here we can use the packet that has been get off whole DSR header
		enum IpL4Protocol::RxStatus status =
				nextProto->Receive (packet, ip, incomingInterface);
				NS_LOG_DEBUG ("The receive status " << status);
				switch (status)
				{
				case IpL4Protocol::RX_OK:
					if(Save() == true)
						SavePktRx(packet);
					break;
					// fall through
				case IpL4Protocol::RX_ENDPOINT_CLOSED:
				// fall through
				case IpL4Protocol::RX_CSUM_FAILED:
					break;
				case IpL4Protocol::RX_ENDPOINT_UNREACH:
					if (ip.GetDestination ().IsBroadcast () == true
						|| ip.GetDestination ().IsMulticast () == true)
					{
						break;
					}

				}
				return status;
	}
	else
	{
		NS_FATAL_ERROR ("Should not have 0 next protocol value");
	}


  return IpL4Protocol::RX_OK;
}

/*Misma función que la otra pero para
 * IPv6. Se implementará en versiones futuras.
 */
enum IpL4Protocol::RxStatus
Dcc::Receive (Ptr<Packet> p,
                     Ipv6Header const &ip,
                     Ptr<Ipv6Interface> incomingInterface)
{
  NS_LOG_FUNCTION (this << p << ip.GetSourceAddress () << ip.GetDestinationAddress () << incomingInterface);
  return IpL4Protocol::RX_ENDPOINT_UNREACH;
}

/*
 * Callback para enviar paquete a capas inferiores
 */
void
Dcc::SetDownTarget (DownTargetCallback callback)
{
  m_downTarget = callback;
}
/*
 * Callback para enviar paquetes a capas inferiores
 * con IPv6
 */
void
Dcc::SetDownTarget6 (DownTargetCallback6 callback)
{
  NS_FATAL_ERROR ("Unimplemented");
}

/*
 *
 */
IpL4Protocol::DownTargetCallback
Dcc::GetDownTarget (void) const
{
  return m_downTarget;
}

IpL4Protocol::DownTargetCallback6
Dcc::GetDownTarget6 (void) const
{
  NS_FATAL_ERROR ("Unimplemented");
  return MakeNullCallback<void,Ptr<Packet>, Ipv6Address, Ipv6Address, uint8_t, Ptr<Ipv6Route> > ();
}


}
}
