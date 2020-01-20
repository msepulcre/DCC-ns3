/*
 * dcc-main-helper.h
 *
 *  Created on: 23 mar. 2019
 *      Author: jorge
 */

#ifndef SRC_DCC_HELPER_DCC_MAIN_HELPER_H_
#define SRC_DCC_HELPER_DCC_MAIN_HELPER_H_



#include "ns3/object-factory.h"
#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/dsr-routing.h"
//#include "ns3/dsr-helper.h"
#include "ns3/dcc-helper.h"
#include "ns3/dcc.h"

namespace ns3 {
/**
 * \brief Helper class that adds DSR routing to nodes.
 */
class DccMainHelper
{
public:
  /**
   * Create an DsrMainHelper that makes life easier for people who want to install
   * DSR routing to nodes.
   */
  DccMainHelper ();
  ~DccMainHelper ();
  /**
   * \brief Construct an DsrMainHelper from another previously initialized instance
   * (Copy Constructor).
   */
  DccMainHelper (const DccMainHelper &);
  void Install (DccHelper &dccHelper, NodeContainer nodes);
  void SetDccHelper (DccHelper &dccHelper);

private:
  void Install (Ptr<Node> node);
  /**
   * \brief Assignment operator declared private and not implemented to disallow
   * assignment and prevent the compiler from happily inserting its own.
   */
  DccMainHelper &operator = (const DccMainHelper &o);
  const DccHelper *m_dccHelper;
};

} // namespace ns3


#endif /* SRC_DCC_HELPER_DCC_MAIN_HELPER_H_ */
