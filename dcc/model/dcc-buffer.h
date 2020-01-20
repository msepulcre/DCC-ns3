/* */
 /* dcc-buffer.h
 *
 *  Created on: 23 mar. 2019
 *      Author: Jorge Mira <jorge.mira01@alu.umh.es>
 */


#ifndef SRC_DCC_MODEL_DCC_BUFFER_H_
#define SRC_DCC_MODEL_DCC_BUFFER_H_



#include <vector>
#include "ns3/node.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/simulator.h"
#include "ns3/ipv4-header.h"
#include "ns3/header.h"


namespace ns3 {
namespace dcc {



/**
 * \brief DCC Queues
 */

class DccHeader : public Header
{
public:
	DccHeader ();
	virtual ~DccHeader();

	static TypeId GetTypeid (void);
	virtual TypeId GetInstanceTypeId (void) const;
	virtual void Print (std::ostream &os) const;
	virtual void Serialize (Buffer::Iterator start) const;
	virtual uint32_t Deserialize (Buffer::Iterator start);
	virtual uint32_t GetSerializedSize (void) const;
	uint8_t GetNextHeader(void) const;
	void SetNextHeader(uint8_t prot);

private:

	uint8_t m_protocol;
};


class DccBuffEntry
{
public:
  /**
   * Construct a DsrMaintainBuffEntry with the given parameters
   *
   * \param pa packet
   * \param us our IPv4 address
   * \param n next hop IPv4 address
   * \param s IPv4 address of the source
   * \param dst IPv4 address of the destination
   * \param ackId ACK ID
   * \param segs number of segments left
   * \param exp expiration time
   */

	DccBuffEntry (Ptr<const Packet> pa = 0, Time exp = Simulator::Now ())
	    : m_packet (pa),
	      m_expire (exp + Simulator::Now ())
		{
		}


  // Fields
  Ptr<const Packet> GetPacket () const
  {
    return m_packet;
  }
  void SetPacket (Ptr<const Packet> p)
  {
    m_packet = p;
  }
  void SetExpireTime (Time exp)
  {
    m_expire = exp + Simulator::Now ();
  }
  Time GetExpireTime () const
  {
    return m_expire - Simulator::Now ();
  }

private:
  /// Data packet
  Ptr<const Packet> m_packet;
  /// Our own ip address
  Ipv4Address m_ourAdd;
  /// Next hop Ip address
  Ipv4Address m_nextHop;
  /// The source address
  Ipv4Address m_src;
  /// The destination address
  Ipv4Address m_dst;
  /// The data ack id
  uint16_t m_ackId;
  /// The segments left field
  uint8_t m_segsLeft;
  /// Expire time for queue entry
  Time m_expire;

};

enum DccPriorities
{

	DP0 = 0,

	DP1 = 1,

	DP2 = 2,

	DP3 = 3,

	UNDEF

};


enum AcIndexe
	{
	  /** Best Effort */
	  AC_BE = 0,
	  /** Background */
	  AC_BK = 1,
	  /** Video */
	  AC_VI = 2,
	  /** Voice */
	  AC_VO = 3,
	  /** Total number of ACs */
	  AC_BE_NQOS = 4,

	  AC_UNDEF

	};

/**
 * \ingroup dcc
 * \brief DCC maintain buffer
 */
/************************************************************************************************************************/
class DccBuffer
{
public:
  /**
   * Default constructor
   */
  DccBuffer ()
  {
  }




  /// Push entry in queue, if there is no entry with the same packet and destination address in queue.
  bool Enqueue (DccBuffEntry & entry);
  /// Return first found (the earliest) entry for given destination
  bool Dequeue (DccBuffEntry & entry);
  /// Remove all packets with destination IP address dst
  void DropPacketWithNextHop (Ipv4Address nextHop);
  /// Finds whether a packet with destination dst exists in the queue
  bool Find (Ipv4Address nextHop);
  /// Number of entries
  uint32_t GetSize ();
  //Save drop messages
  void SaveDrop(Ptr<Packet> DropPacket);

  void SaveEnqueue(Ptr<Packet> pa);

  void SaveDequeue(Ptr<Packet> pa);

  void SaveMensajes();

  // Fields
  uint32_t GetMaxQueueLen () const
  {
    return m_maxLen;
  }
  void SetMaxQueueLen (uint32_t len)
  {
    m_maxLen = len;
  }
  Time GetMaintainBufferTimeout () const
  {
    return m_maintainBufferTimeout;
  }
  void SetMaintainBufferTimeout (Time t)
  {
    m_maintainBufferTimeout = t;
  }

  // Set differents TTL to CAM and CPM
  void SetMaintainBufferTimeoutCAM (Time t){

	  m_maintainBufferTimeoutCAM = t;
  }

  Time GetMaintainBufferTimeoutCAM () const{

	  return m_maintainBufferTimeoutCAM;
  }

  void SetMaintainBufferTimeoutCPM(Time t){

	  m_maintainBufferTimeoutCPM = t;
  }

  Time GetMaintainBufferTimeoutCPM () const{
	  return m_maintainBufferTimeoutCPM;
  }

  void SetNode(Ptr<Node> node){
	  m_node = node;
  }

  Ptr<Node> GetNode(void){
	  return m_node;
  }

  void SetNumCPM(int num){
	  numCPM = num;
  }

  void SetNumCAM(int num){
	  numCAM = num;
  }

  void SetMargenAleatorio(int num){
	  margen_aleatorio = num;
  }

  void SetNumCars(int num){
  	  NumCars = num;
    }

  DccPriorities QoSToDccPriority (uint8_t tid);


private:
  /// The vector of maintain buffer entries
  std::vector<DccBuffEntry> m_maintainBuffer;



  /// Remove all expired entries
  void Purge ();
  /// The maximum number of packets that we allow a routing protocol to buffer.
  uint32_t m_maxLen;
  /// The maximum period of time that a routing protocol is allowed to buffer a packet for, seconds.
  Time m_maintainBufferTimeout;

  Time m_maintainBufferTimeoutCAM;
  Time m_maintainBufferTimeoutCPM;

  // Para loggear-------------------------
  Ptr<Node> m_node;

  int NumCars;
  int margen_aleatorio;
  int numCPM;
  int numCAM;

  //DCC-----------------------------------

  DccHeader dccHeader;

  // DCC QUEUES
  std::vector<DccBuffEntry> m_QueueDP0;

  std::vector<DccBuffEntry> m_QueueDP1;

  std::vector<DccBuffEntry> m_QueueDP2;

  std::vector<DccBuffEntry> m_QueueDP3;


  DccPriorities prior;


};
/*******************************************************************************************************************************/
} // namespace dsr
} // namespace ns3


#endif /* SRC_DCC_MODEL_DCC_BUFFER_H_ */
