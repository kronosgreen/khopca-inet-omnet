/*
 * KHOPCARouting.cc
 *
 *  Created on: Jun 28, 2017
 *      Author: chrst
 */

#include "inet/routing/khopca/KHOPCARouting.h"
#include "inet/networklayer/ipv4/ICMPMessage.h"
#include "inet/networklayer/ipv4/IPv4Route.h"

#ifdef WITH_IDEALWIRELESS
#include "inet/linklayer/ideal/IdealMacFrame_m.h"
#endif // ifdef WITH_IDEALWIRELESS

#ifdef WITH_IEEE80211
#include "inet/linklayer/ieee80211/mac/Ieee80211Frame_m.h"
#endif // ifdef WITH_IEEE80211

#ifdef WITH_CSMA
#include "inet/linklayer/csma/CSMAFrame_m.h"
#endif // ifdef WITH_CSMA

#ifdef WITH_CSMACA
#include "inet/linklayer/csmaca/CsmaCaMacFrame_m.h"
#endif // ifdef WITH_CSMA

#ifdef WITH_LMAC
#include "inet/linklayer/lmac/LMacFrame_m.h"
#endif // ifdef WITH_LMAC

#ifdef WITH_BMAC
#include "inet/linklayer/bmac/BMacFrame_m.h"
#endif // ifdef WITH_BMAC

#include "inet/networklayer/common/IPSocket.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/transportlayer/contract/udp/UDPControlInfo.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/NodeOperations.h"
#include "inet/power/storage/SimpleEpEnergyStorage.h"

#include <string>
#include <sstream>
#include <vector>
#include <iterator>

#include <algorithm>

#include <iostream>
#include <fstream>

#include <time.h>

//
// Split function from Community Answer on Stack Overflow
// Link: https://stackoverflow.com/questions/236129/the-most-elegant-way-to-iterate-the-words-of-a-string
//
template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

namespace inet {

Define_Module(KHOPCARouting);

void KHOPCARouting::initialize(int stage){

    if (stage == INITSTAGE_LOCAL) {
        EV << "KHOPCA: Initializing in stage " << stage << endl;
        host = getContainingNode(this);
        routingTable = getModuleFromPar<IRoutingTable>(par("routingTableModule"), this);
        interfaceTable = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);
        networkProtocol = getModuleFromPar<INetfilter>(par("networkProtocolModule"), this);

        MAX = par("MAX");
        MIN = par("MIN");
        w_n = par("w_n");
        updateInterval = par("updateInterval");
        updateInterval += (uniform(1,100)/50);
        //destAddress = par("destAddress");
        power = uniform(1,100)/100;

        // Naming Output Vectors for Statistics Collectio*n
        clusterSize.setName("Cluster Size");
        roleChange.setName("Role Changes");
        clusterDuration.setName("Cluster Duration");
        ratioClusterheads.setName("Ratio of Clusterheads to Non-Clusterheads");
        numMsgExchanged.setName("Number of Messages Exchanged");
        RoCinClusterheads.setName("Rate of Change in # of Clusterheads");
        nodeDelay.setName("Delay");
        ratioUnconnectedNodes.setName("Ratio of Unconnected Nodes");

        khopcaUDPPort = par("udpPort");
        selfMsg = new cMessage("sendTimer");

    }
    else if (stage == INITSTAGE_ROUTING_PROTOCOLS) {
        EV << "KHOPCA: Initializing Routing Protocol: " << stage << endl;
        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(host->getSubmodule("status"));

        addressType = getSelfIPAddress().getAddressType();
        address = this->getSelfIPAddress();
        IPSocket socket(gate("ipOut"));
        socket.registerProtocol(IP_PROT_MANET);
        networkProtocol->registerHook(0, this);
        host->subscribe(NF_LINK_BREAK, this);

        scheduleAt(simTime() + updateInterval, selfMsg);
    }
}

//
// Printing to a text file from : https://groups.google.com/forum/#!topic/omnetpp/ZgcG96lYcdo
//
void KHOPCARouting::printKHOPCAInfo(char* filename, float calcTime)
{
    std::ofstream  out;
    out.open(filename, std::ios_base::app);

    if(connected[0].size() > 0){
        out << endl << "CLUSTER HEAD : " << address << "    SIM TIME : " << simTime() << endl << "----------------------------------" << endl;
        std::vector<std::vector<std::string>>::iterator connIt;
        std::vector<std::string>::iterator nodeIt;
        int weightInd = MAX - 1;
        for(connIt = connected.begin(); connIt != connected.end(); connIt++){
            out << endl << "LEVEL : " << weightInd << endl << "---------------------------------" << endl;
            for(nodeIt = (*connIt).begin(); nodeIt != (*connIt).end(); nodeIt++){
                out << (*nodeIt).c_str() << endl;
            }
            weightInd--;
        }
        out << endl << "TIME TO COMPUTE : " << calcTime << endl;
    }



    out.close();
}

//
// When a cluster head drops in weight, prints the duration of its cluster
//
void KHOPCARouting::printClusterDuration(char* filename, simtime_t dur){

    std::ofstream  out;
    out.open(filename, std::ios_base::app);

    out << dur << endl;

    out.close();
}

//
// Print the number of times the role or weight changed during the simulation
//
void KHOPCARouting::printRoleChanges(char* filename, int roleChanges){

    std::ofstream  out;
    out.open(filename, std::ios_base::app);

    out << roleChanges << endl;

    out.close();
}

//
// Calculates the max weight in the neighborhood of nodes
//
int KHOPCARouting:: MaxNeighborWeight(){
    unsigned int maxWeight = 0;
    for(std::vector<nodeProperties>::iterator it = neighbors.begin(); it != neighbors.end(); ++it)
        if(it -> wt >  maxWeight)
            maxWeight = it -> wt;
    return maxWeight;
}

//
// Implements the first rule of K-HopCA
//
void KHOPCARouting:: ClusteringHierarchy(){
    int maxWeight = MaxNeighborWeight();
    if(maxWeight > w_n){
        //EV << getContainingNode(this) << " : Rule 1" << endl;
        w_n = maxWeight - 1;
    }
}

//
// Implements the second rule of K-HopCA
//
void KHOPCARouting:: AllMin(){
    if(MaxNeighborWeight() == MIN && w_n == MIN){
        //EV << getContainingNode(this) << " : Rule 2" << endl;
        w_n = MAX;
    }
}

//
// Implements the third rule of K-HopCA
//
void KHOPCARouting:: AvoidFragmentation(){
    if(MaxNeighborWeight() <= w_n && w_n != MAX && MaxNeighborWeight() != 0){
        //EV << getContainingNode(this) << " : Rule 3" << endl;
        w_n -= 1;
    }
}

//
// Implements the fourth rule of K-HopCA
//
void KHOPCARouting:: MultipleMax(){
    if(MaxNeighborWeight() == MAX && w_n == MAX)
    //selection criterion for two nodes with MAX weight goes here
    {
        //EV << getContainingNode(this) << address << " : Rule 4, vec length " << neighbors.size() << endl;

        bool mostBattery = true;

        for(std::vector<nodeProperties>::iterator it = neighbors.begin(); it != neighbors.end(); ++it)
        {
            EV << (*it).power << endl;
            if((*it).power > this->power)
                mostBattery = false;
        }

        if(!mostBattery)
            w_n -= 1;
    }
}

//
// Resets weight to minimum when no other node is w/in range
//
void KHOPCARouting:: SetMinIfAlone(){
    if(neighbors.size() == 0){
        w_n = MIN;
        EV << "Host " << getContainingNode(this) << " is alone." << endl;
    }
}

//
// Creates packet with node information for K-HopCA
//
KHOPCAWeight *KHOPCARouting:: CreateKHOPCAWeight(const L3Address& destAddr){

    KHOPCAWeight *kwPacket = new KHOPCAWeight("KHOPCAWeight");

    IRoute *lastKnownRoute = routingTable->findBestMatchingRoute(destAddr);
    //PacketUnreachableNode node;

    power::SimpleEpEnergyStorage *energyStor = getModuleFromPar<power::SimpleEpEnergyStorage>(par("energyStorage"), this);
    power::J en = energyStor -> getResidualEnergyCapacity();
    power = en.get();
    EV << getContainingNode(this) << " - Energy : " << power << endl;
    EV << "CURRENT TIME: " << simTime() << endl;
    kwPacket -> setW_n(w_n);
    kwPacket -> setOriginatorAddr(address);
    kwPacket -> setDestAddr(destAddr);
    kwPacket -> setPow(power);
    kwPacket -> setEntryTime(SIMTIME_DBL(simTime()));
    kwPacket -> setTree(CreateTreeString().c_str());

    return kwPacket;
}

void KHOPCARouting::SendKHOPCAWeight(KHOPCAWeight* packet, const L3Address& destAddr){

    /*cStringTokenizer tokenizer(destAddress);
    const char *token = nullptr;
    token = tokenizer.nextToken();
    L3Address da = L3AddressResolver().resolve(token);*/

    INetworkProtocolControlInfo *networkProtocolControlInfo = addressType->createNetworkProtocolControlInfo();
    networkProtocolControlInfo->setTransportProtocol(IP_PROT_MANET);
    networkProtocolControlInfo->setSourceAddress(address);
    networkProtocolControlInfo->setHopLimit(1);
    networkProtocolControlInfo->setDestinationAddress(destAddr);

    InterfaceEntry *ifEntry = interfaceTable->getInterfaceByName("wlan0");
    networkProtocolControlInfo->setInterfaceId(ifEntry->getInterfaceId());


    UDPPacket *udpPacket = new UDPPacket(packet->getName());
    udpPacket->encapsulate(packet);
    udpPacket->setSourcePort(khopcaUDPPort);
    udpPacket->setDestinationPort(khopcaUDPPort);
    udpPacket->setControlInfo(dynamic_cast<cObject *>(networkProtocolControlInfo));

    send(udpPacket, "ipOut");
}

void KHOPCARouting:: HandleKHOPCAWeight(KHOPCAWeight* kwPacket){

    nodeProperties node = {
        kwPacket-> getOriginatorAddr(),
        kwPacket-> getW_n(),
        kwPacket-> getPow(),
        SIMTIME_DBL(simTime()),
        kwPacket-> getTree()
    };
    bool alreadyFound = false;
    EV << "Checking neighbors" << endl;
    for(std::vector<nodeProperties>::iterator it = neighbors.begin(); it != neighbors.end(); ++it){
        if((*it).addr == node.addr){
            EV << "Updated node: " << getContainingNode(this) << endl;
            *it = node;
            alreadyFound = true;
        }
    }

    if(!alreadyFound){
        EV << "Not Found" << endl;
        neighbors.push_back(node);
    }

}

void KHOPCARouting:: ImplementRules(){

    std::vector<L3Address> outOfRange;
    //Gets rid of hosts no longer in range
    for(std::vector<nodeProperties>::iterator it = neighbors.end(); it != neighbors.begin(); --it){
        if((simTime().dbl() - (*it).entryTime) > (updateInterval + 1)){
            EV << "Host " << getContainingNode(this) << " no longer reaches host " << (*it).addr << endl;
            neighbors.erase(it);
        }
    }

    clock_t startTime = clock();
    int old_w_n = w_n;

    //Implements 4 KHOPCA Rules
    if(simTime() > updateInterval + 1){
        EV << "Implementing KHOPCA Rules" << endl;

        ClusteringHierarchy();
        AllMin();
        AvoidFragmentation();
        MultipleMax();

        SetMinIfAlone();

        SetConnected();
    }
    // Record role changes
    if (old_w_n != w_n)
    {
        roleChange.record(w_n);
        roleChangeStats.collect(w_n);
    }

    // Check if node has become clusterhead or has lost clusterhead position
    if (old_w_n != MAX && w_n == MAX){
        headDur = simTime();
    } else if (old_w_n == MAX && w_n != MAX) {
        clusterDuration.record(simTime() - headDur);
        clusterDurationStats.collect(simTime() - headDur);
        headDur = 0;
    }

    //Displays weight at router/host level
    std::string s = "Weight: " + std::to_string(w_n);
    std::vector<char> v(s.begin(), s.end());
    v.push_back('\0');
    const char* w = &v[0];
    getDisplayString().setTagArg("t", 0, w);

    cModule *cmod = getContainingNode(this);
    cmod ->getDisplayString().setTagArg("t", 0, w);

    EV << this << " : Setting " << s << endl;

    if(w_n == MAX){
        clock_t timeToCalc = clock() - startTime;
        printKHOPCAInfo("output.txt", ((float)timeToCalc)/CLOCKS_PER_SEC);
    }

}

//Set up 2d vector of all nodes beneath this one
//0 index of vector of layers is the one directly beneath the node calling
//the function, then continues down through each vector

void KHOPCARouting::SetConnected(){
    connected.clear();
    if(w_n == 1)
        return;
    connected = std::vector<std::vector<std::string>>(w_n - 1);
    std::vector<nodeProperties>::iterator neighborsIt;
    std::vector<std::vector<std::string>>::iterator layerIt;
    std::vector<std::string>::iterator nodesIt;
    int layerIndex;
    for(neighborsIt = neighbors.begin(); neighborsIt != neighbors.end(); neighborsIt++){
        std::vector<std::vector<std::string>> tree = ParseTree((*neighborsIt).tree);
        if(tree.empty())
            continue;
        layerIndex = 0;
        for(layerIt = tree.begin(); layerIt != tree.end(); layerIt++){
            for(nodesIt = (*layerIt).begin(); nodesIt != (*layerIt).end(); nodesIt++){
                if(std::find(connected[layerIndex].begin(), connected[layerIndex].end(), *nodesIt) == connected[layerIndex].end())
                    connected[layerIndex].push_back(*nodesIt);
            }
            layerIndex++;
        }

    }

}

//take strings that were sent over the messages and loop over them to construct a 2d vector, putting nodes
std::vector<std::vector<std::string>> KHOPCARouting::ParseTree(std::string info){

    std::vector<std::vector<std::string>> tree = std::vector<std::vector<std::string>>(w_n - 1);

    std::vector<std::string>::iterator layersIt;
    std::vector<std::string>::iterator nodesIt;
    int wtIndex;
    int nodeInfo = 0;
    std::vector<std::string> layers = split(info, ':');
    //EV << "LAYERS SZ: " << layers.size() << " , [0] : " << layers[0] << endl;
    if(layers.size() == 1){
        std::vector<std::string> nodes = split(layers[0], ' ');

        wtIndex = stoi(nodes[1]);
        //EV << "NODES[1] : " << wtIndex << ", W_N : " << w_n << endl;
        if(wtIndex >= w_n || w_n - wtIndex > 1)
            return tree;
        nodes.pop_back();

        if(std::find(tree[w_n - wtIndex - 1].begin(), tree[w_n - wtIndex - 1].end(), nodes[0]) == tree[w_n - wtIndex].end()){
            //std::cout << "@ ParseTree : Pushing back to tree to " << w_n - wtIndex - 1 << "\n" << endl;
            tree[0].push_back(nodes[0]);
        }
    } else {
        for(layersIt = layers.begin(); layersIt != layers.end(); layersIt++){
                std::vector<std::string> nodes = split(*layersIt, ' ');
                //EV << "NODES SZ : " << nodes.size() << " , [0] : " << nodes[0] << endl;
                if(nodeInfo == 0){
                    wtIndex = stoi(nodes[1]);
                    //EV << "NODES[1] : " << wtIndex << ", W_N : " << w_n << endl;
                    if(wtIndex >= w_n || w_n - wtIndex > 1)
                        return tree;
                    nodes.pop_back();
                    nodeInfo = 1;
                }

                for(nodesIt = nodes.begin(); nodesIt != nodes.end(); nodesIt++){
                    if(std::find(tree[w_n - wtIndex - 1].begin(), tree[w_n - wtIndex - 1].end(), *nodesIt) == tree[w_n - wtIndex].end()){
                        //std::cout << "@ ParseTree : Pushing back to tree" << endl;
                        tree[w_n - wtIndex - 1].push_back(*nodesIt);
                    }
                }
                wtIndex--;
            }

    }

    return tree;

}

//Create string with node addr, and weight, then addr of all following nodes storing them by weight
//in the appropriate vector
std::string KHOPCARouting::CreateTreeString(){

    std::string routingInfo = getSelfIPAddress().str() + " " + std::to_string(w_n) + ":";

    std::vector<std::string>::iterator nodeIt;
    //EV << "CONNECTED SIZE : " << connected.size() << ";  W_N : " << w_n << " ; NGHBRTREES SIZE : " << neighborTrees.size() << endl;
    for(int i = 0; i < connected.size(); i++){
        for(nodeIt = connected[i].begin(); nodeIt != connected[i].end(); nodeIt++){
            routingInfo += *nodeIt + " ";
        }
        routingInfo[routingInfo.length() - 1] = ':';
    }
    routingInfo.erase(routingInfo.length() - 1);
    EV << routingInfo << endl;

    return routingInfo;
}

void KHOPCARouting::handleMessage(cMessage *msg) {
    EV << "KHOPCA: Handling Message" << endl;
    if(msg->isSelfMessage()){
        if(msg == selfMsg){
            EV << "Handling Send Timer" << endl;
            KHOPCAWeight* kwPacket = CreateKHOPCAWeight(addressType->getBroadcastAddress());
            SendKHOPCAWeight(kwPacket, addressType->getBroadcastAddress());
            scheduleAt(simTime() + updateInterval, selfMsg);

            ImplementRules();
        }
        else{
            throw cRuntimeError("Unknown Self Message");
        }
    }
    else if (ICMPMessage *icmpPacket = dynamic_cast<ICMPMessage *>(msg)) {
            // ICMP packet arrived, dropped
            delete icmpPacket;
    }
    else{
        EV << "Handling KHOPCA Packet" << endl;
        UDPPacket *udpPacket = check_and_cast<UDPPacket *>(msg);
        KHOPCAWeight *kwPacket = check_and_cast<KHOPCAWeight *>(udpPacket -> decapsulate());

        HandleKHOPCAWeight(kwPacket);

        delete kwPacket;
        delete udpPacket;
    }
}


L3Address KHOPCARouting:: getSelfIPAddress() const{
    return routingTable->getRouterIdAsGeneric();
}

/*IRoute *KHOPCARouting::createRoute(const L3Address& destAddr, const L3Address& nextHop, unsigned int hopCount, bool hasValidDestNum, unsigned int destSeqNum, bool isActive, simtime_t lifeTime)
{
    IRoute *newRoute = routingTable->createRoute();
    KHOPCARouteData *newProtocolData = new KHOPCARouteData();

    newProtocolData->setHasValidDestNum(hasValidDestNum);

    // active route
    newProtocolData->setIsActive(isActive);

    // A route towards a destination that has a routing table entry
    // that is marked as valid.  Only active routes can be used to
    // forward data packets.

    newProtocolData->setLifeTime(lifeTime);
    newProtocolData->setDestSeqNum(destSeqNum);

    InterfaceEntry *ifEntry = interfaceTable->getInterfaceByName("wlan0");    // TODO: IMPLEMENT: multiple interfaces
    if (ifEntry)
        newRoute->setInterface(ifEntry);

    newRoute->setDestination(destAddr);
    //newRoute->setSourceType(IRoute::KHOPCA);
    newRoute->setSource(this);
    newRoute->setProtocolData(newProtocolData);
    newRoute->setMetric(hopCount);
    newRoute->setNextHop(nextHop);
    newRoute->setPrefixLength(addressType->getMaxPrefixLength());    // TODO:

    EV_DETAIL << "Adding new route " << newRoute << endl;
    routingTable->addRoute(newRoute);
    //scheduleExpungeRoutes();
    return newRoute;
}*/

INetfilter::IHook::Result KHOPCARouting::datagramForwardHook(INetworkDatagram *datagram, const InterfaceEntry *inputInterfaceEntry, const InterfaceEntry *& outputInterfaceEntry, L3Address& nextHopAddress)
{
    // TODO: Implement: Actions After Reboot
    // If the node receives a data packet for some other destination, it SHOULD
    // broadcast a RERR as described in subsection 6.11 and MUST reset the waiting
    // timer to expire after current time plus DELETE_PERIOD.

    Enter_Method("datagramForwardHook");
    const L3Address& destAddr = datagram->getDestinationAddress();
    const L3Address& sourceAddr = datagram->getSourceAddress();
    IRoute *ipSource = routingTable->findBestMatchingRoute(sourceAddr);

    if (destAddr.isBroadcast() || routingTable->isLocalAddress(destAddr) || destAddr.isMulticast()) {
        //if (routingTable->isLocalAddress(destAddr) && ipSource && ipSource->getSource() == this)
            //updateValidRouteLifeTime(ipSource->getNextHopAsGeneric(), simTime() + activeRouteTimeout);

        return ACCEPT;
    }

    // TODO: IMPLEMENT: check if the datagram is a data packet or we take control packets as data packets

    IRoute *routeDest = routingTable->findBestMatchingRoute(destAddr);
    KHOPCARouteData *routeDestData = routeDest ? dynamic_cast<KHOPCARouteData *>(routeDest->getProtocolData()) : nullptr;

    // Each time a route is used to forward a data packet, its Active Route
    // Lifetime field of the source, destination and the next hop on the
    // path to the destination is updated to be no less than the current
    // time plus ACTIVE_ROUTE_TIMEOUT

    //updateValidRouteLifeTime(sourceAddr, simTime() + activeRouteTimeout);
    //updateValidRouteLifeTime(destAddr, simTime() + activeRouteTimeout);

    //if (routeDest && routeDest->getSource() == this)
    //    updateValidRouteLifeTime(routeDest->getNextHopAsGeneric(), simTime() + activeRouteTimeout);

    // Since the route between each originator and destination pair is expected
    // to be symmetric, the Active Route Lifetime for the previous hop, along the
    // reverse path back to the IP source, is also updated to be no less than the
    // current time plus ACTIVE_ROUTE_TIMEOUT.

    //if (ipSource && ipSource->getSource() == this)
    //    updateValidRouteLifeTime(ipSource->getNextHopAsGeneric(), simTime() + activeRouteTimeout);

    EV_INFO << "We can't forward datagram because we have no active route for " << destAddr << endl;
    //if (routeDest && routeDestData && !routeDestData->isActive()) {    // exists but is not active
        // A node initiates processing for a RERR message in three situations:
        // (ii)      if it gets a data packet destined to a node for which it
        //           does not have an active route and is not repairing (if
        //           using local repair)

        // TODO: check if it is not repairing (if using local repair)

        // 1. The destination sequence number of this routing entry, if it
        // exists and is valid, is incremented for cases (i) and (ii) above,
        // and copied from the incoming RERR in case (iii) above.

    //    if (routeDestData->hasValidDestNum())
    //        routeDestData->setDestSeqNum(routeDestData->getDestSeqNum() + 1);

        // 2. The entry is invalidated by marking the route entry as invalid <- it is invalid

        // 3. The Lifetime field is updated to current time plus DELETE_PERIOD.
        //    Before this time, the entry SHOULD NOT be deleted.
    //    routeDestData->setLifeTime(simTime() + deletePeriod);

    //    sendRERRWhenNoRouteToForward(destAddr);
    //}
    //else if (!routeDest || routeDest->getSource() != this) // doesn't exist at all
    //    sendRERRWhenNoRouteToForward(destAddr);

    return ACCEPT;
}


void KHOPCARouting::receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details)
{
    Enter_Method("receiveChangeNotification");
    if (signalID == NF_LINK_BREAK) {
        EV_DETAIL << "Received link break signal" << endl;
        // XXX: This is a hack for supporting both IdealMac and Ieee80211Mac. etc
        cPacket *frame = check_and_cast<cPacket *>(obj);
        INetworkDatagram *datagram = nullptr;
        if (false
#ifdef WITH_IEEE80211
            || dynamic_cast<ieee80211::Ieee80211Frame *>(frame)
#endif // ifdef WITH_IEEE80211
#ifdef WITH_IDEALWIRELESS
            || dynamic_cast<IdealMacFrame *>(frame)
#endif // ifdef WITH_IDEALWIRELESS
#ifdef WITH_CSMA
            || dynamic_cast<CSMAFrame *>(frame)
#endif // ifdef WITH_CSMA
#ifdef WITH_CSMACA
            || dynamic_cast<CsmaCaMacFrame *>(frame)
#endif // ifdef WITH_CSMACA
#ifdef WITH_LMAC
            || dynamic_cast<LMacFrame *>(frame)
#endif // ifdef WITH_LMAC
#ifdef WITH_BMAC
            || dynamic_cast<BMacFrame *>(frame)
#endif // ifdef WITH_BMAC
            )
            datagram = dynamic_cast<INetworkDatagram *>(frame->getEncapsulatedPacket());
        else
            throw cRuntimeError("Unknown packet type in NF_LINK_BREAK signal");
        if (datagram) {
            L3Address unreachableAddr = datagram->getDestinationAddress();
            if (unreachableAddr.getAddressType() == addressType) {
                // A node initiates processing for a RERR message in three situations:
                //
                //   (i)     if it detects a link break for the next hop of an active
                //           route in its routing table while transmitting data (and
                //           route repair, if attempted, was unsuccessful), or

                // TODO: Implement: local repair

                IRoute *route = routingTable->findBestMatchingRoute(unreachableAddr);

               // if (route && route->getSource() == this)
               //   handleLinkBreakSendRERR(route->getNextHopAsGeneric());
            }
        }
    }
}

void KHOPCARouting::clearState(){
    cancelEvent(selfMsg);
}
KHOPCARouting::KHOPCARouting() {
    // Temp
}
void KHOPCARouting::finish() {

    // Print out duration as cluster head at close
    if(w_n == MAX){
        clusterDuration.record(simTime() - headDur);
        clusterDurationStats.collect(simTime() - headDur);
    }

    EV << "Stats Recorded" << endl;
    // Cancel and delete remaining messages
    clearState();
    delete selfMsg;

}
KHOPCARouting::~KHOPCARouting(){
    clearState();
    delete selfMsg;
}



} // end namespace

