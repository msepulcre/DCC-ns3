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
#include "dcc-main-helper.h"
#include "ns3/dcc.h"
#include "ns3/node-list.h"
#include "ns3/names.h"
#include "ns3/log.h"
#include "ns3/ptr.h"
#include "ns3/node.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("DccMainHelper");

DccMainHelper::DccMainHelper ()
  : m_dccHelper (0)
{
  NS_LOG_FUNCTION (this);
}

DccMainHelper::DccMainHelper (const DccMainHelper &o)
{
  NS_LOG_FUNCTION (this);
  m_dccHelper = o.m_dccHelper->Copy ();
}

DccMainHelper::~DccMainHelper ()
{
  NS_LOG_FUNCTION (this);
  delete m_dccHelper;
}

DccMainHelper &
DccMainHelper::operator = (const DccMainHelper &o)
{
  if (this == &o)
    {
      return *this;
    }
  m_dccHelper = o.m_dccHelper->Copy ();
  return *this;
}

void
DccMainHelper::Install (DccHelper &dccHelper, NodeContainer nodes)
{
  NS_LOG_DEBUG ("Passed node container");
  delete m_dccHelper;
  m_dccHelper = dccHelper.Copy ();
  for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i)
    {
      Install (*i);
    }
}

void
DccMainHelper::Install (Ptr<Node> node)
{
  NS_LOG_FUNCTION (node);

  Ptr<ns3::dcc::Dcc> dcc = m_dccHelper->Create (node); 
  dcc->SetNode (node);
}

void
DccMainHelper::SetDccHelper (DccHelper &dccHelper)
{
  NS_LOG_FUNCTION (this);
  delete m_dccHelper;
  m_dccHelper = dccHelper.Copy ();
}

} // namespace ns3
