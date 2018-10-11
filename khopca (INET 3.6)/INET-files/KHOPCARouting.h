/*
 * KHOPCARouting.h
 *
 *  Created on: Jun 28, 2017
 *      Author: chrst
 */

#ifndef INET_ROUTING_KHOPCA_KHOPCAROUTING_H_
#define INET_ROUTING_KHOPCA_KHOPCAROUTING_H_


#include "inet/common/INETDefs.h"
#include "inet/networklayer/contract/IInterfaceTable.h"
#include "inet/networklayer/contract/IL3AddressType.h"
#include "inet/networklayer/contract/IRoutingTable.h"
#include "inet/networklayer/contract/INetfilter.h"
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/transportlayer/contract/udp/UDPSocket.h"
#include "inet/routing/khopca/KHOPCARouteData.h"
#include "inet/transportlayer/udp/UDPPacket.h"
#include "inet/routing/khopca/KHOPCAControlPackets_m.h"
#include <map>


namespace inet {

    class StatsTracker;

    class KHOPCARouting : public cSimpleModule, public cListener {// , public INetfilter::IHook

        protected:

            // context
            IL3AddressType *addressType = nullptr;    // to support both IPv4 and v6 addresses.
            L3Address address;

            // environment
            cModule *host = nullptr;
            IRoutingTable *routingTable = nullptr;
            IInterfaceTable *interfaceTable = nullptr;
            INetfilter *networkProtocol = nullptr;


            int khopcaUDPPort = 0;

            double headDur;
            StatsTracker *stats = nullptr;
            int clusterSize;

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

            //KHOPCA Parameters
            int MAX;
            int MIN;
            int w_n;
            double updateInterval;
            double power;
            //const char* destAddress;

            //Creates and sends weight/address info
            KHOPCAWeight *CreateKHOPCAWeight(const L3Address& destAddr);
            void SendKHOPCAWeight(KHOPCAWeight* packet, const L3Address& destAddr);
            IRoute *createRoute(const L3Address& destAddr, const L3Address& nextHop, unsigned int hopCount, bool hasValidDestNum, unsigned int destSeqNum, bool isActive, simtime_t lifeTime);

            //Initialization, message/error handlers
            void initialize(int stage) override;
            virtual int numInitStages() const override { return NUM_INIT_STAGES; }
            L3Address getSelfIPAddress() const;
            void handleMessage(cMessage *msg) override;

            //KHOPCA W(N(n))
            int MaxNeighborWeight();

            //Receives info of neighbors and implements KHOPCA
            void HandleKHOPCAWeight(KHOPCAWeight* kwPacket);

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


            // End functions
            void clearState();
            virtual void finish() override;

        public:
            KHOPCARouting();
            virtual ~KHOPCARouting();
            int getWeight();

            // For Statistics Collection
            bool isCluster();
            bool isClusterHead();
            bool isUnconnectedNode();
            int getClusterSize();

    };

}



#endif /* INET_ROUTING_KHOPCA_KHOPCAROUTING_H_ */
