/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef DCC_HELPER_H
#define DCC_HELPER_H

#include "ns3/dcc.h"

#include "ns3/node-container.h"
#include "ns3/object-factory.h"
#include "ns3/dsr-routing.h"
#include "ns3/node.h"
#include "ns3/udp-l4-protocol.h"
#include "ns3/tcp-l4-protocol.h"
#include "ns3/icmpv4-l4-protocol.h"

namespace ns3 {
class DccHelper
{
public:
  /**
   * Create an DsrHelper that makes life easier for people who want to install
   * Dsr routing to nodes.
   */
  DccHelper ();
  ~DccHelper ();
  /**
   * \brief Construct an DsrHelper from another previously initialized instance
   * (Copy Constructor).
   */
  DccHelper (const DccHelper &);
  /**
   * \returns pointer to clone of this DsrHelper
   *
   * This method is mainly for internal use by the other helpers;
   * clients are expected to free the dynamic memory allocated by this method
   */
  DccHelper* Copy (void) const;
  /**
   * \param node the node on which the routing protocol will run
   * \returns a newly-created L4 protocol
   */
  Ptr<ns3::dcc::Dcc> Create (Ptr<Node> node) const;
  void Set (std::string name, const AttributeValue &value);
private:
  /**
   * \brief Assignment operator declared private and not implemented to disallow
   * assignment and prevent the compiler from happily inserting its own.
   */
  DccHelper & operator = (const DccHelper &o);
  ObjectFactory m_agentFactory;
  NodeContainer m_nodes;
};

} // namespace ns3


#endif /* DCC_HELPER_H */

