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

    class KHOPCARouting : public cSimpleModule, public cListener, public INetfilter::IHook {

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
            KHOPCAWeight *CreateKHOPCAWeight(const L3Address& destAddr);
            void SendKHOPCAWeight(KHOPCAWeight* packet, const L3Address& destAddr);
            //void BroadcastKHOPCAWeight(KHOPCAWeight* packet);
            IRoute *createRoute(const L3Address& destAddr, const L3Address& nextHop, unsigned int hopCount, bool hasValidDestNum, unsigned int destSeqNum, bool isActive, simtime_t lifeTime);

            //Initialization, message/error handlers
            void initialize(int stage) override;
            virtual int numInitStages() const override { return NUM_INIT_STAGES; }
            L3Address getSelfIPAddress() const;
            void handleMessage(cMessage *msg) override;
            virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details) override;

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

            //Print info
            void printKHOPCAInfo(char* filename, float calcTime);
            void printClusterDuration(char* filename, simtime_t dur);
            void printRoleChanges(char* filename, int roleChanges);

            // Netfilter hooks
            Result ensureRouteForDatagram(INetworkDatagram *datagram){ return ACCEPT; };
            virtual Result datagramPreRoutingHook(INetworkDatagram *datagram, const InterfaceEntry *inputInterfaceEntry, const InterfaceEntry *& outputInterfaceEntry, L3Address& nextHopAddress) override { Enter_Method("datagramPreRoutingHook"); return ensureRouteForDatagram(datagram); }
            virtual Result datagramForwardHook(INetworkDatagram *datagram, const InterfaceEntry *inputInterfaceEntry, const InterfaceEntry *& outputInterfaceEntry, L3Address& nextHopAddress) override;
            virtual Result datagramPostRoutingHook(INetworkDatagram *datagram, const InterfaceEntry *inputInterfaceEntry, const InterfaceEntry *& outputInterfaceEntry, L3Address& nextHopAddress) override { return ACCEPT; }
            virtual Result datagramLocalInHook(INetworkDatagram *datagram, const InterfaceEntry *inputInterfaceEntry) override { return ACCEPT; }
            virtual Result datagramLocalOutHook(INetworkDatagram *datagram, const InterfaceEntry *& outputInterfaceEntry, L3Address& nextHopAddress) override { Enter_Method("datagramLocalOutHook"); return ensureRouteForDatagram(datagram); }
            //void delayDatagram(INetworkDatagram *datagram);

            // End functions
            void clearState();
            virtual void finish() override;

        public:
            KHOPCARouting();
            virtual ~KHOPCARouting();
            int getWeight();

    };

}



#endif /* INET_ROUTING_KHOPCA_KHOPCAROUTING_H_ */
