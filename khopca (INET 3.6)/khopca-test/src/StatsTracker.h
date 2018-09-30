// Stats Tracker
// Will store vectors for data collection


#include "inet/common/INETDefs.h"

class StatsTracker {

    protected:

        cOutVector roleChange;
        cOutVector clusterDuration;
        cOutVector clusterSize;
        cOutVector ratioClusterheads;
        cOutVector numMsgExchanged;
        cOutVector RoCinClusterheads;
        cOutVector nodeDelay;
        cOutVector ratioUnconnectedNodes;

        cStdDev roleChangeStats;
        cStdDev clusterDurationStats;
        cStdDev clusterSizeStats;
        cStdDev ratioClusterheadsStats;
        cStdDev numMsgStats;
        cStdDev RoCinClusterheadsStats;
        cStdDev delayStats;
        cStdDev ratioUnconnectedStats;

        void getClusterSize();
        void getNumClusters();
        void getRatioHeads();
        void getRatioUnconnected();
        void getRateOfChangeHeads();

        void recordDurationHead(double dur);


};
