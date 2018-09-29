/*
 * KHOPCARouteData.h
 *
 *  Created on: Jun 28, 2017
 *      Author: Christopher Medrano-Berumen
 */

#ifndef INET_ROUTING_KHOPCA_KHOPCAROUTEDATA_H_
#define INET_ROUTING_KHOPCA_KHOPCAROUTEDATA_H_

#include <set>
#include "inet/networklayer/common/L3Address.h"
#include "inet/common/INETDefs.h"

namespace inet {

    class INET_API KHOPCARouteData : public cObject {

    protected:
        std::set<L3Address> precursorList;
        bool active;
        bool repariable;
        bool beingRepaired;
        bool validDestNum;
        unsigned int destSeqNum;
        simtime_t lifeTime;    // expiration or deletion time of the route

      public:

        KHOPCARouteData()
        {
            active = true;
            repariable = false;
            beingRepaired = false;
            validDestNum = true;
            lifeTime = SIMTIME_ZERO;
            destSeqNum = 0;
        }

        virtual ~KHOPCARouteData() {}

        unsigned int getDestSeqNum() const { return destSeqNum; }
        void setDestSeqNum(unsigned int destSeqNum) { this->destSeqNum = destSeqNum; }
        bool hasValidDestNum() const { return validDestNum; }
        void setHasValidDestNum(bool hasValidDestNum) { this->validDestNum = hasValidDestNum; }
        bool isBeingRepaired() const { return beingRepaired; }
        void setIsBeingRepaired(bool isBeingRepaired) { this->beingRepaired = isBeingRepaired; }
        bool isRepariable() const { return repariable; }
        void setIsRepariable(bool isRepariable) { this->repariable = isRepariable; }
        const simtime_t& getLifeTime() const { return lifeTime; }
        void setLifeTime(const simtime_t& lifeTime) { this->lifeTime = lifeTime; }
        bool isActive() const { return active; }
        void setIsActive(bool active) { this->active = active; }
        void addPrecursor(const L3Address& precursorAddr) { precursorList.insert(precursorAddr); }
        const std::set<L3Address>& getPrecursorList() const { return precursorList; }

    };

    std::ostream& operator<<(std::ostream& out, const KHOPCARouteData *data);

}



#endif /* INET_ROUTING_KHOPCA_KHOPCAROUTEDATA_H_ */
