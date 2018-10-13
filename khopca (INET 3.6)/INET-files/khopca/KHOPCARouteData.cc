/*
 * KHOPCARouteData.cc
 *
 *  Created on: Jun 28, 2017
 *      Author: chrst
 */

#include "inet/routing/khopca/KHOPCARouteData.h"

namespace inet {

std::ostream& operator<<(std::ostream& out, const KHOPCARouteData *data)
{
    out << " isActive = " << data->isActive()
        << ", hasValidDestNum = " << data->hasValidDestNum()
        << ", destNum = " << data->getDestSeqNum()
        << ", lifetime = " << data->getLifeTime();

    const std::set<L3Address>& preList = data->getPrecursorList();

    if (!preList.empty()) {
        out << ", precursor list: ";
        std::set<L3Address>::const_iterator iter = preList.begin();
        out << *iter;
        for (++iter; iter != preList.end(); ++iter)
            out << "; " << *iter;
    }
    return out;
};



}
