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

#include "dcc-helper.h"
#include "ns3/dcc.h"
#include "ns3/node-container.h"
#include "ns3/node.h"
#include "ns3/callback.h"
#include "ns3/ptr.h"
#include "ns3/log.h"
#include "ns3/tcp-l4-protocol.h"
#include "ns3/udp-l4-protocol.h"
#include "ns3/ipv4-route.h"
#include "ns3/node-list.h"
#include "ns3/names.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("DccHelper");

DccHelper::DccHelper ()
{
  NS_LOG_FUNCTION (this);
  m_agentFactory.SetTypeId ("ns3::dcc::Dcc");
}

DccHelper::DccHelper (const DccHelper &o)
  : m_agentFactory (o.m_agentFactory)
{
  NS_LOG_FUNCTION (this);
}

DccHelper::~DccHelper ()
{
  NS_LOG_FUNCTION (this);
}

DccHelper*
DccHelper::Copy (void) const
{
  NS_LOG_FUNCTION (this);
  return new DccHelper (*this);
}

Ptr<ns3::dcc::Dcc>
DccHelper::Create (Ptr<Node> node) const
{
  NS_LOG_FUNCTION (this);
  Ptr<ns3::dcc::Dcc> agent = m_agentFactory.Create<ns3::dcc::Dcc> ();
  // deal with the downtargets, install UdpL4Protocol, TcpL4Protocol, Icmpv4L4Protocol
  Ptr<UdpL4Protocol> udp = node->GetObject<UdpL4Protocol> ();
  agent->SetDownTarget (udp->GetDownTarget ());
  udp->SetDownTarget (MakeCallback (&dcc::Dcc::Send, agent));
  Ptr<TcpL4Protocol> tcp = node->GetObject<TcpL4Protocol> ();
  tcp->SetDownTarget (MakeCallback (&dcc::Dcc::Send, agent));
  Ptr<Icmpv4L4Protocol> icmp = node->GetObject<Icmpv4L4Protocol> ();
  icmp->SetDownTarget (MakeCallback (&dcc::Dcc::Send, agent));
  node->AggregateObject (agent);
  return agent;
}

void
DccHelper::Set (std::string name, const AttributeValue &value)
{
  m_agentFactory.Set (name, value);
}

} // namespace ns3


