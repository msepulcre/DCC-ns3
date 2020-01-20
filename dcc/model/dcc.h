/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil;
 * 
 *  Author: Jorge Mira <jorge.mira01@alu.umh.es>
 * 
 *  -*- */
#ifndef DCC_H
#define DCC_H



#include <map>
#include <list>
#include <vector>
#include <stdint.h>
#include <cassert>
#include <sys/types.h>

#include "ns3/callback.h"
#include "ns3/object.h"
#include "ns3/node.h"
#include "ns3/ptr.h"
#include "ns3/buffer.h"
#include "ns3/packet.h"
#include "ns3/ipv4.h"
#include "ns3/ip-l4-protocol.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/icmpv4-l4-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-header.h"
#include "ns3/ipv4-address.h"
#include "ns3/traced-callback.h"
#include "ns3/random-variable-stream.h"
#include "ns3/ipv4-route.h"
#include "ns3/timer.h"
#include "ns3/net-device.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-phy-state-helper.h"
#include "ns3/socket.h"
#include "ns3/event-garbage-collector.h"
#include "ns3/test.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/yans-wifi-phy.h"

#include "ns3/dcc-buffer.h"



namespace ns3
{

class Packet;
class Node;
class Ipv4;
class Ipv4Address;
class Ipv4Header;
class Ipv4Interface;
class Ipv4L3Protocol;
class Time;




namespace dcc {

class Dcc : public IpL4Protocol
{
public:

  /**
   * \brief Get the type identificator.
   * \return type identificator
   */
  static TypeId GetTypeId ();
  /**
    * \brief Define the dsr protocol number.
    */
  static const uint8_t PROT_NUMBER;
  /**
   * \brief Constructor.
   */
  Dcc ();
  /**
   * \brief Destructor.
   */
  virtual ~Dcc ();
  /**
   * \brief Get the node.
   * \return the node
   */


  Ptr<Node> GetNode () const;
  /**
   * \brief Set the node.
   * \param node the node to set
   */



  void ChannelBusyRatioProbing( Time start, Time duration, enum WifiPhy::State);


  void SetNode (Ptr<Node> node);
  /**
    * \brief Get the netdevice from the context.
    * \param context context
    * \return the netdevice we are looking for
    */
  Ptr<NetDevice> GetNetDeviceFromContext (std::string context);
  /**
    * \brief Get the elements from the tracing context.
    * \param context context
    * \return the elements we are looking for
    */
  std::vector<std::string> GetElementsFromContext (std::string context);
  /**
    * \brief Get the node id from ip address.
    * \param address IPv4 address
    * \return the node id
    */
  uint16_t GetIDfromIP (Ipv4Address address);
  /**
    * \brief Get the ip address from id.
    * \param id unique ID
    * \return the ip address for the id
    */
  Ipv4Address GetIPfromID (uint16_t id);
  /**
    * \brief Get the Ip address from mac address.
    * \param address Mac48Address
    * \return the ip address
    */
  Ipv4Address GetIPfromMAC (Mac48Address address);
  /**
    * \brief Get the node with give ip address.
    * \param ipv4Address IPv4 address
    * \return the node associated with the ip address
    */
  Ptr<Node> GetNodeWithAddress (Ipv4Address ipv4Address);
  /**
      * \brief Get the protocol number of DCC
      * \param void
      * \return the protocol number of DCC
  */
  int GetProtocolNumber (void) const;

  int GetTxProtocolNumber(void) const;


  void Send (Ptr<Packet> packet, Ipv4Address source,
             Ipv4Address destination, uint8_t protocol, Ptr<Ipv4Route> route);

  void SendPacketQueue (Ipv4Address source, Ipv4Address destination, uint8_t protocol);

  void SendPacketFromQueue(Ipv4Address source, Ipv4Address destination, uint8_t protocol);

  void SendPacket (Ipv4Address source);


  //void CBRupdateTimerExpire ();

  void Scheduler (uint32_t priority);
  /**
   * \brief This function is called to schedule sending packets from the network queue by priority
   */
  void PriorityScheduler (uint32_t priority, bool continueWithFirst);

  //-------------------------------------------------- PARA DCC -------------------------------------------------------
  bool Toff_Check();

  double TimeToNextTx();

  void StateUpdate(); //Reactive approach

  void ObtencionCBR();
  void CBRupadte();

  void CBR_TimerExpire();

  void CbrProbeTimerExpire();

  void CurrentState(); // Reactive approach

  void DeltaUpdate(); // Adaptive approach

  void ToffUpdateAfterTransmission(); // Adaptive aapproach

  void ToffUpdateAfterCBRupdate(); //Adaptive approach

  void PacketDuration(Ptr<Packet> packet); // (Tonpp) Adaptive approach

  void AlgorithmCheck();

  //////////// Log functions /////////////////////////////////
  void SaveTxRateFile(Ptr<Packet> packet);

  void SaveCBR();

  void SaveToff();

  void SavePktRx(Ptr<Packet> packet);

  bool Save();

   /////////////////////////////////////////////////////////////////////

  /**
   * \param p packet to forward up
   * \param header IPv4 Header information
   * \param incomingInterface the Ipv4Interface on which the packet arrived
   * \return receive status
   *
   * Called from lower-level layers to send the packet up
   * in the stack.
   */
  virtual enum IpL4Protocol::RxStatus Receive (Ptr<Packet> p,
                                               Ipv4Header const &header,
                                               Ptr<Ipv4Interface> incomingInterface);

  /**
   * \param p packet to forward up
   * \param header IPv6 Header information
   * \param incomingInterface the Ipv6Interface on which the packet arrived
   * \return receive status
   *
   * Called from lower-level layers to send the packet up
   * in the stack.  Not implemented (IPv6).
   */


  virtual enum IpL4Protocol::RxStatus Receive (Ptr<Packet> p,
                                               Ipv6Header const &header,
                                               Ptr<Ipv6Interface> incomingInterface);

  void SetDownTarget (IpL4Protocol::DownTargetCallback callback);
  void SetDownTarget6 (IpL4Protocol::DownTargetCallback6 callback);
  IpL4Protocol::DownTargetCallback GetDownTarget (void) const;
  IpL4Protocol::DownTargetCallback6 GetDownTarget6 (void) const;

protected:
  /*
 *    * This function will notify other components connected to the node that a new stack member is now connected
 *       * This will be used to notify Layer 3 protocol of layer 4 protocol stack to connect them together.
 *          */
  virtual void NotifyNewAggregate ();
  /**
   * \brief Drop trace callback.
   */
  virtual void DoDispose (void);
  /**
   * The trace for drop, receive and send data packets
   */
  TracedCallback<Ptr<const Packet> > m_dropTrace;
  //GOKUL
  TracedCallback<Ptr<const Packet>, double, uint32_t, uint32_t > m_Toff;
  TracedCallback<Ptr<const Packet>, double, uint32_t, uint32_t > m_TxALL;
  TracedCallback<Ptr<const Packet>, double, uint32_t, uint32_t > m_TxDCC;

  void NotifyDCCTOFF  (Ptr<const Packet> packet, double Time, uint32_t Sender_ID, uint32_t T_off);
  void NotifyDCCTXALL (Ptr<const Packet> packet, double Time, uint32_t Sender_ID, uint32_t T_off);
  void NotifyDCCTXDCC (Ptr<const Packet> packet, double Time, uint32_t Sender_ID, uint32_t T_off);
//GOKUL

private:

  void Start ();


  DccBuffEntry entry;                                   ///< DccBuffEntry (Para las colas)

  Ptr<Ipv4L3Protocol> m_ipv4;                           ///< Ipv4l3Protocol

  Ptr<Ipv4> m_ip;                                       ///< The ip ptr

  Ptr<Node> m_node;                                     ///< The node ptr

  Ipv4Address m_mainAddress;                            ///< Our own Ip address

  IpL4Protocol::DownTargetCallback m_downTarget;        ///< The callback for down layer

  uint32_t  m_maxMaintainLen;                           ///< Max # of entries for maintainance buffer

  Time     m_maxMaintainTime;                           ///< Time out for maintainance buffer

  Time m_maxMaintainTimeCAM;							/// < Time out for maintanance buffer for CAM pakcets

  Time m_maxMaintainTimeCPM;							/// < Time out for maintanance buffer for CPM pakcets

  DccBuffer m_maintainBuffer;                           ///< The declaration of maintain buffer

  Ipv4Address m_broadcast;                              ///< The broadcast IP address

  Ptr<UniformRandomVariable> m_uniformRandomVariable;    ///< Provides uniform random variables.

  //--------------------------------- PARA DCC -----------------------------------

  std::string Algorithm; //

  int m_algorithm;
  //Current CBR
  double CBR;

  //CBR before
  double CBR_Before;

  //Toff
  Time Toff;

  //Last sending
  double UltimoEnvio;

  double LastTx;

  Time LastTxTime;

  Time WaitingToTx;
  //First time simulation
  bool primero;

  //Time waiting to send
  double tiempoespera;

  //CBR timer
  Timer CBR_Timer;
  //CBR probes timer
  Timer CBR_Probes_Timer;

  Time CBR_Interval; ///< how often to update CBR

  /**
   * State of DCC. Relaxed -> 0.
   * Active1 -> 1.
   * Active2 -> 2.
   * Active3 -> 3.
   * Restringed -> 4.
   */

  enum DccStates
  	{
  	  Relaxed = 0,

	  Active1 = 1,

  	  Active2 = 2,

  	  Active3 = 3,

  	  Restringed = 4
  	};

  int DccState;


  Time node_Not_Idle;
  Time Last_DeltaUpdate;
  double States_Dur;




  uint8_t CBR_PacketId; // Id de un paquete para diferenciarlo a la hora de medir el CBR.

  std::multimap <uint8_t, Time> CBR_Packet_Start;
  std::multimap <uint8_t, Time> CBR_Packet_Duration;



  Ptr<WifiPhy> m_wifiphy;
  std::multiset <WifiPhy::State> states;


  double CBR_its; //Adaptive algorithm parameter

  double alpha; //Adaptive algorithm parameter

  double beta; //Adaptive algorithm parameter

  double CBR_target; //Adaptive algorithm parameter

  double delta_max; //Adaptive algorithm parameter

  double delta_min; //Adaptive algorithm parameter

  double Gmax; //Adaptive algorithm parameter

  double Gmin; //Adaptive algorithm parameter

  double delta; //Adaptive algorithm parameter

  Time Tonpp; //Duration of last transmission

  ///////////////////////////////////////////////////////////// PARA LOGGEAR

  int NumCars;

  int margen_aleatorio;

  double density;

  double spd;

  bool cluster;


  //DCC GOKUL
  int QUEUE;

  //------------------------------------------------------------------------------
};//class

}//dcc
}//namespace

#endif /* DCC_H */

