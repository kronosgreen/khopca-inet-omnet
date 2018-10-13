/*
 * StatsTracker.h
 *
 *  Created on: Sep 30, 2018
 *      Author: Christopher Medrano-Berumen
 */

#ifndef __INET_STATSTRACKER_H_
#define __INET_STATSTRACKER_H_


#include "inet/common/INETDefs.h"


namespace inet {

class KHOPCARouting;

class StatsTracker : public cSimpleModule {

    protected:

        std::vector<KHOPCARouting *> khopcaNodes;
        int numHosts;
        cMessage *selfMsg = nullptr;
        double updateInterval;

        cOutVector roleChange;
        cOutVector clusterDuration;
        cOutVector clusterSize;
        cOutVector numClusters;
        cOutVector ratioClusterheads;
        cOutVector numMsgExchanged;
        cOutVector RoCinClusterheads;
        cOutVector nodeDelay;
        cOutVector ratioUnconnectedNodes;

        cStdDev roleChangeStats;
        cStdDev clusterDurationStats;
        cStdDev clusterSizeStats;
        cStdDev numClustersStats;
        cStdDev ratioClusterheadsStats;
        cStdDev numMsgExchangedStats;
        cStdDev RoCinClusterheadsStats;
        cStdDev nodeDelayStats;
        cStdDev ratioUnconnectedStats;

        void initialize(int stage) override;
        void handleMessage(cMessage *msg) override;

        void getSimulationStats();
        void getRateOfChangeHeads();

    public:

        void recordDurationHead(double dur);
        void recordRoleChange(int rol);
        void recordExcMesg(int exMsg);
        void recordDelay(double delay);

        StatsTracker();
        virtual ~StatsTracker();


};

}

#endif /* END DEF KHOPCA-TEST_STATSTRACKER_STATSTRACKER_H_ */
