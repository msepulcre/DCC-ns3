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
#include "dcc-buffer.h"
#include <algorithm>
#include <functional>
#include "ns3/node.h"
#include "ns3/ipv4-route.h"
#include "ns3/socket.h"
#include "ns3/log.h"
#include <stdio.h>

//GOKUL
#include "ns3/socket-type-tag.h"
//GOKUL

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("DccBuffer");

namespace dcc {


// ---------------- DCC Header -------------------------------

NS_OBJECT_ENSURE_REGISTERED (DccHeader);

DccHeader::DccHeader(){

}

DccHeader::~DccHeader(){

}

TypeId
DccHeader::GetTypeid(){
	static TypeId tid = TypeId ("ns3::DccHeader")
	.SetParent<Header> ()
	.AddConstructor<DccHeader> ()
	;
	return tid;
}

TypeId
DccHeader::GetInstanceTypeId(void) const
{
	return GetTypeId ();
}

void
DccHeader::Print(std::ostream &os) const
{
	//Method invoked by the packet printing routines
	//to print tjen content of the header
	//os << "Priority = " << m_priority;
}

uint32_t
DccHeader::GetSerializedSize(void) const
{
	//¿Cuántos bytes reservo para la cabecera??
	//Reservo 2 bytes
	return 2;
}

void
DccHeader::Serialize(Buffer::Iterator start) const
{
	//Serialize 2 bytes from the start of the buffer
	Buffer::Iterator i = start;

	i.WriteU8 (GetNextHeader ());
}

uint32_t
DccHeader::Deserialize(Buffer::Iterator start)
{
	Buffer::Iterator i = start;
	//Deserialize 2 bytes froom the start of the buffer
	SetNextHeader (i.ReadU8 ());
	return 2;
}

uint8_t DccHeader::GetNextHeader() const
{
	return m_protocol;
}

void DccHeader::SetNextHeader(uint8_t prot){
	m_protocol = prot;
}
// --------------------------------------------------------------------

/*
 * Conversion of SocketPriorityTag into
 * vehicular communications priorities
 */


//GOKUL The case numbers are defined based on the EDCA Parameters (wifi/model/qos-utils)
DccPriorities DccBuffer::QoSToDccPriority (uint8_t tid){

	switch (tid)
	{
	case 7:
		return DP0;
		break;
	case 6:
		return DP0;
		break;
	case 5:
		return DP1;
		break;
	case 4:
		return DP1;
		break;
	case 3:
		return DP2;
		break;
	case 2:
		return DP2;
		break;
	case 1:
		return DP3;
		break;
	case 0:
		return DP3;
		break;
	}
	return UNDEF;
}



/*
 * Enqueue a message. Depending on its priority,
 * the message is stored in a specific queue.
 */
bool
DccBuffer::Enqueue (DccBuffEntry & entry)
{
  Purge ();
  Ptr<Packet> packet;
  SocketPriorityTag qos;
  packet = entry.GetPacket()->Copy();


  packet->PeekPacketTag(qos);

  prior = QoSToDccPriority(qos.GetPriority());


  switch (prior){

  case DP0:
	  entry.SetExpireTime (m_maintainBufferTimeout);
	  m_QueueDP0.push_back(entry);
	  if(m_QueueDP0.size() >= m_maxLen){
		  NS_LOG_DEBUG("Drop the most aged packet");
		  Ptr<Packet> pa = entry.GetPacket()->Copy();
		  m_QueueDP1.pop_back();
		  SaveDrop(pa);
	  }

	  break;

  case DP1:
	  	  entry.SetExpireTime (m_maintainBufferTimeout);

	  	  m_QueueDP1.push_back(entry);
	  	  SaveEnqueue(packet);
	  	if(m_QueueDP1.size() > m_maxLen){
			  NS_LOG_DEBUG("Drop the most aged packet");
			  Ptr<Packet> pa = entry.GetPacket()->Copy();
			  m_QueueDP1.pop_back();
			  SaveDrop(pa);
	  	}
	  	  break;

  case DP2:
	  	  entry.SetExpireTime (m_maintainBufferTimeout);
	  	  m_QueueDP2.push_back(entry);
	  	  if(m_QueueDP2.size() >= m_maxLen){
	  		  NS_LOG_DEBUG("Drop the most aged packet");
	  				  Ptr<Packet> pa = entry.GetPacket()->Copy();
	  				  m_QueueDP2.pop_back();
	  				  SaveDrop(pa);
		  }

	  	  break;

  case DP3:
	  	  entry.SetExpireTime (m_maintainBufferTimeout);
	  	  m_QueueDP3.push_back(entry);
	  	  if(m_QueueDP3.size() >= m_maxLen){
	  		  NS_LOG_DEBUG("Drop the most aged packet");
	  				  Ptr<Packet> pa = entry.GetPacket()->Copy();
	  				  m_QueueDP3.pop_back();
	  				  SaveDrop(pa);
	  	  }

	  	  break;
  case UNDEF:
	  break;
  }

  return true;
}
/*
 * Deuqueue a message for TX.
 * Message tx is the most priority.
 */
bool
DccBuffer::Dequeue (DccBuffEntry & entry)
{
	NS_LOG_DEBUG("Dequeueing packet");
  	Purge();
  	if(m_QueueDP0.size() > 0){
  		entry = *m_QueueDP0.begin();
  		m_QueueDP0.erase(m_QueueDP0.begin());
  		return true;
  	}else if(m_QueueDP1.size() > 0){
  		Ptr<Packet> pa = m_QueueDP1.begin()->GetPacket()->Copy();
  		entry = *m_QueueDP1.begin();
  		m_QueueDP1.erase(m_QueueDP1.begin());
  		SaveDequeue(pa);
  		return true;
  	}else if(m_QueueDP2.size() > 0){
  		entry = *m_QueueDP2.begin();
  		m_QueueDP2.erase(m_QueueDP2.begin());
  		return true;
  	}else if(m_QueueDP3.size() > 0){
  		entry = *m_QueueDP3.begin();
  		m_QueueDP3.erase(m_QueueDP3.begin());
  		return true;
  	}

  return false;
}

/*
 * Struct to check if a message
 * has reached its TTL
 */
struct IsExpired
{
  bool
  operator() (DccBuffEntry const & e) const
  {
    return (e.GetExpireTime () < Seconds (0));
  }
};

/*
 * Función que elimina mensajes que
 * han alcanzado su tiempo de vida
 */
void
DccBuffer::Purge ()
{
	NS_LOG_DEBUG ("Purging Queues");
	IsExpired pred;

	std::vector<DccBuffEntry>::iterator DropPackets0;
	std::vector<DccBuffEntry>::iterator DropPackets1;
	std::vector<DccBuffEntry>::iterator DropPackets2;
	std::vector<DccBuffEntry>::iterator DropPackets3;


	/// APUNTAN AL ÚLTIMO MENSAJE ELIMINADO
	DropPackets0 = std::remove_if (m_QueueDP0.begin (), m_QueueDP0.end (),pred);
	DropPackets1 = std::remove_if (m_QueueDP1.begin (), m_QueueDP1.end (),pred);
	DropPackets2 = std::remove_if (m_QueueDP2.begin (), m_QueueDP2.end (),pred);
	DropPackets3 = std::remove_if (m_QueueDP3.begin (), m_QueueDP3.end (),pred);


	int counter = 0;
	//Cuantos son los NO eliminados
	for ( std :: vector<DccBuffEntry> :: iterator q =m_QueueDP1.begin(); q != DropPackets1; ++q){
		 counter++;

	 }

	/*if(m_QueueDP1.size() > 0){
		DccBuffEntry &en;
		en = *DropPackets1;

		NS_LOG_UNCOND(en.GetPacket()->GetSize());
	}*/
		//NS_LOG_UNCOND("Counter " << counter);

	//NS_LOG_UNCOND(DropPackets1->GetPacket()->GetSize());
	m_QueueDP0.erase (std::remove_if (m_QueueDP0.begin (), m_QueueDP0.end (), pred),
 					  m_QueueDP0.end ());
	m_QueueDP1.erase (DropPackets1, m_QueueDP1.end());

	/*m_QueueDP1.erase (std::remove_if (m_QueueDP1.begin (), m_QueueDP1.end (), pred),
   					  m_QueueDP1.end ());*/

	m_QueueDP2.erase (std::remove_if (m_QueueDP2.begin (), m_QueueDP2.end (), pred),
   					  m_QueueDP2.end ());

	m_QueueDP3.erase (std::remove_if (m_QueueDP3.begin (), m_QueueDP3.end (), pred),
   					  m_QueueDP3.end ());



}

/*
 * Función para registrar un descarte por cola llena.
 *
 *  */
void
DccBuffer::SaveDrop(Ptr<Packet> DropPacket){

}

/*
 * Guarda métricas cuando un mensaje se introduce en la cola
 */
void
DccBuffer::SaveEnqueue(Ptr<Packet> pa){

}


/*
 * Guarda cuando un mensaje es extraido de la cola
 */

void
DccBuffer::SaveDequeue(Ptr<Packet> pa){

}

/*
 * Función para almacenar evolución de la cola m_QueueDP1.
 */
void
DccBuffer::SaveMensajes(){

}

}  // namespace DCC
}  // namespace ns3



