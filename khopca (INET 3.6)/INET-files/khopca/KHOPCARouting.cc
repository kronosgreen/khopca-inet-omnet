/*
 * KHOPCARouting.cc
 *
 *  Created on: Jun 28, 2017
 *      Author: chrst
 */

#include "inet/routing/khopca/KHOPCARouting.h"
#include "inet/common/statstracker/StatsTracker.h"
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
        stats = getModuleFromPar<StatsTracker>(par("statsModule"), this);

        MAX = par("MAX");
        MIN = par("MIN");
        w_n = par("w_n");
        updateInterval = par("updateInterval");
        updateInterval += (uniform(1,100)/100);
        packetLossProbability = par("packetLossProbability");
        //destAddress = par("destAddress");
        power = uniform(1,100)/100;

        khopcaUDPPort = par("udpPort");
        if(selfMsg != nullptr && selfMsg->isScheduled() && selfMsg->isSelfMessage()){
            cancelAndDelete(selfMsg);
        }
        selfMsg = new cMessage("sendTimer");

    }
    else if (stage == INITSTAGE_ROUTING_PROTOCOLS) {
        EV << "KHOPCA: Initializing Routing Protocol: " << stage << endl;
        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(host->getSubmodule("status"));
        isOperational = !nodeStatus || nodeStatus->getState() == NodeStatus::UP;

        addressType = getSelfIPAddress().getAddressType();
        address = this->getSelfIPAddress();
        IPSocket socket(gate("ipOut"));
        socket.registerProtocol(IP_PROT_MANET);
        host->subscribe(NF_LINK_BREAK, this);

        if(isOperational){
            scheduleAt(simTime() + updateInterval, selfMsg);
        }
    }
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

    delete kwPacket;

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
        stats -> recordRoleChange(w_n);
    }

    // Check if node has become clusterhead or has lost clusterhead position
    if (old_w_n != MAX && w_n == MAX){
        headDur = simTime().dbl();
    } else if (old_w_n == MAX && w_n != MAX) {
        stats -> recordDurationHead(simTime().dbl() - headDur);
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
        //printKHOPCAInfo("output.txt", ((float)timeToCalc)/CLOCKS_PER_SEC);
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
    if (!isOperational) {
        //if (msg->isSelfMessage())
          //  throw cRuntimeError("Model error: self msg '%s' received when isOperational is false", msg->getName());

        EV_ERROR << "Application is turned off, dropping '" << msg->getName() << "' message\n";
        delete msg;
        return;
    }
    if(msg->isSelfMessage()){
        if(msg == selfMsg){
            EV << "Handling Send Timer" << endl;
            KHOPCAWeight* kwPacket = CreateKHOPCAWeight(addressType->getBroadcastAddress());
            SendKHOPCAWeight(kwPacket, addressType->getBroadcastAddress());
            if(simTime() + updateInterval < SimTime::getMaxTime()){
                scheduleAt(simTime() + updateInterval, selfMsg);
            }

            ImplementRules();

        } else {
            throw cRuntimeError("Unknown Self Message");
        }
    } else if (ICMPMessage *icmpPacket = dynamic_cast<ICMPMessage *>(msg)) {
            // ICMP packet arrived, dropped
            delete icmpPacket;
    } else {

        EV << "Handling KHOPCA Packet" << endl;
        if(uniform(0,1) < packetLossProbability){

            EV << "Packet Lost" << endl;
            return;
        } else if(msg == nullptr){
            EV << "Message already deleted" << endl;
            return;
        }
        UDPPacket *udpPacket = check_and_cast<UDPPacket *>(msg);
        KHOPCAWeight *kwPacket = check_and_cast<KHOPCAWeight *>(udpPacket -> decapsulate());

        HandleKHOPCAWeight(kwPacket);

        delete udpPacket;

    }
}


L3Address KHOPCARouting:: getSelfIPAddress() const{
    return routingTable->getRouterIdAsGeneric();
}

// returns True if node is cluster head or independent
bool KHOPCARouting::isCluster() {
    return w_n == MAX || neighbors.size() == 0;
}

bool KHOPCARouting::isClusterHead() {
    return w_n == MAX;
}

bool KHOPCARouting::isUnconnectedNode() {
    return w_n == 1 && neighbors.size() == 0;
}

int KHOPCARouting::getClusterSize(){
    return connected.size();
}
KHOPCARouting::KHOPCARouting() {
    // Temp
}

bool KHOPCARouting::handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback){
    Enter_Method_Silent();
        if (dynamic_cast<NodeStartOperation *>(operation)) {
            if ((NodeStartOperation::Stage)stage == NodeStartOperation::STAGE_APPLICATION_LAYER) {
                isOperational = true;;
            }
        }
        else if (dynamic_cast<NodeShutdownOperation *>(operation)) {
            if ((NodeShutdownOperation::Stage)stage == NodeShutdownOperation::STAGE_APPLICATION_LAYER) {
                isOperational = false;
                clearState();
            }
        }
        else if (dynamic_cast<NodeCrashOperation *>(operation)) {
            if ((NodeCrashOperation::Stage)stage == NodeCrashOperation::STAGE_CRASH) {
                isOperational = false;
                clearState();
            }
        }
        else
            throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());

        return true;
}

void KHOPCARouting::clearState(){
    if(selfMsg->isScheduled()){
        cancelEvent(selfMsg);
    }
}

KHOPCARouting::~KHOPCARouting(){
    // Print out duration as cluster head at close
    if(w_n == MAX && isOperational){
        //stats -> recordDurationHead(simTime().dbl() - headDur);
    }

    EV << "Stats Recorded" << endl;
    clearState();
    delete selfMsg;

}



} // end namespace

