/*
 * KHOPCARouting.h
 *
 *  Created on: Jun 28, 2017
 *      Author: Christopher Medrano
 */

#ifndef __INET_KHOPCAROUTING_H
#define __INET_KHOPCAROUTING_H


#include "inet/common/INETDefs.h"
#include "inet/networklayer/contract/IInterfaceTable.h"
#include "inet/networklayer/contract/IL3AddressType.h"
#include "inet/networklayer/contract/IRoutingTable.h"
#include "inet/networklayer/contract/INetfilter.h"
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/transportlayer/udp/UdpHeader_m.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include <map>

namespace inet {
namespace khopca {

class INET_API KHOPCARouting : public cSimpleModule, public ILifecycle, public cListener { //, public NetfilterBase::HookBase

    protected:

        // context
        IL3AddressType *addressType = nullptr;    // to support both IPv4 and v6 addresses.
        L3Address address;

        // Environment & Status
        cModule *host = nullptr;
        IRoutingTable *routingTable = nullptr;
        IInterfaceTable *interfaceTable = nullptr;
        INetfilter *networkProtocol = nullptr;

        // Lifecycle
        simtime_t rebootTime;    // the last time when the node rebooted
        bool isOperational = false;

        int khopcaUDPPort = 0;

        // Stat Collecting
        simtime_t headDur;
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

        // Will contain information about itself for hello packet
        struct nodeProperties {
            L3Address addr;
            unsigned int wt;
            double power;
            double entryTime;
            std::string tree;
        };

        std::vector<nodeProperties> neighbors;
        L3Address parent;
        //std::vector<std::string> neighborTrees;
        std::vector<std::vector<std::string>> connected;

        cMessage *selfMsg = nullptr;

        int MAX;
        int MIN;
        int w_n;
        double updateInterval;
        double power;
        //const char* destAddress;

        //Creates and sends weight/address info
        const Ptr<KHOPCAWeight>& CreateKHOPCAWeight(const L3Address& destAddr);
        void SendKHOPCAWeight(const Ptr<KHOPCAWeight>& packet, const L3Address& destAddr);
        IRoute *createRoute(const L3Address& destAddr, const L3Address& nextHop, unsigned int hopCount, bool hasValidDestNum, unsigned int destSeqNum, bool isActive, simtime_t lifeTime);

        //Initialization, message/error handlers, Lifecycle
        void initialize(int stage) override;
        virtual int numInitStages() const override { return NUM_INIT_STAGES; }
        L3Address getSelfIPAddress() const;
        void handleMessage(cMessage *msg) override;
        virtual bool handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback) override;

        //KHOPCA W(N(n))
        int MaxNeighborWeight();

        //Receives info of neighbors and implements KHOPCA
        void HandleKHOPCAWeight(const Ptr<KHOPCAWeight>& kwPacket);

        //KHOPCA Rules
        void ClusteringHierarchy();     //rule 1
        void AllMin();                  //rule 2
        void AvoidFragmentation();      //rule 3
        void MultipleMax();             //rule 4
        void ImplementRules();          //Implements all 4
        void SetMinIfAlone();
        void SetConnected();
        std::vector<std::vector<std::string>> ParseTree(std::string info);
        std::string CreateTreeString();

        // Print info
        void printKHOPCAInfo(char* filename, float calcTime);

        virtual void finish() override;
        void clearState();

    public:
        int getWeight();
        KHOPCARouting();
        virtual ~KHOPCARouting();

};

}
} // End Namespaces


#endif /* __INET_KHOPCAROUTING_H */
