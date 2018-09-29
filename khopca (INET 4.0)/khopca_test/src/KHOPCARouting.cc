/*
 * KHOPCARouting.cc
 *
 *  Created on: Jun 28, 2017
 *      Author: Christopher Medrano
 */

#include "KHOPCARouting.h"

#include "inet/common/IProtocolRegistrationListener.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/common/packet/Packet.h"
#include "inet/networklayer/common/HopLimitTag_m.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/networklayer/common/L3Tools.h"
#include "inet/networklayer/ipv4/IcmpHeader.h"
#include "inet/networklayer/ipv4/Ipv4Route.h"
#include "inet/networklayer/ipv4/Ipv4Header_m.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/NodeOperations.h"
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
namespace khopca {

Define_Module(KHOPCARouting);

void KHOPCARouting::initialize(int stage){

    if (stage == INITSTAGE_LOCAL) {
        EV << "KHOPCA: Initializing in stage " << stage << endl;
        rebootTime = SIMTIME_ZERO;
        host = getContainingNode(this);
        routingTable = getModuleFromPar<IRoutingTable>(par("routingTableModule"), this);
        interfaceTable = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);
        networkProtocol = getModuleFromPar<INetfilter>(par("networkProtocolModule"), this);

        // Getting Parameters from NED Module
        MAX = par("MAX");
        MIN = par("MIN");
        w_n = par("w_n");
        updateInterval = par("updateInterval");
        updateInterval += (uniform(1,100)/50);
        //destAddress = par("destAddress");
        power = uniform(1,100)/100;

        // Naming Output Vectors for Statistics Collection
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

        addressType = getSelfIPAddress().getAddressType();
        address = this->getSelfIPAddress();

        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(host->getSubmodule("status"));
        isOperational = !nodeStatus || nodeStatus->getState() == NodeStatus::UP;
        addressType = getSelfIPAddress().getAddressType();
        registerService(Protocol::manet, nullptr, gate("ipIn"));
        registerProtocol(Protocol::manet, gate("ipOut"), nullptr);
        //networkProtocol->registerHook(0, this);
        //host->subscribe(linkBrokenSignal, this);

        scheduleAt(simTime() + updateInterval, selfMsg);
    }
}

bool KHOPCARouting::handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback)
{
    Enter_Method_Silent();
    if (dynamic_cast<NodeStartOperation *>(operation)) {
        if (static_cast<NodeStartOperation::Stage>(stage) == NodeStartOperation::STAGE_APPLICATION_LAYER) {
            isOperational = true;
            rebootTime = simTime();

//            if (useHelloMessages)
//                scheduleAt(simTime() + helloInterval - *periodicJitter, helloMsgTimer);
//
//            scheduleAt(simTime() + 1, counterTimer);
        }
    }
    else if (dynamic_cast<NodeShutdownOperation *>(operation)) {
        if (static_cast<NodeShutdownOperation::Stage>(stage) == NodeShutdownOperation::STAGE_APPLICATION_LAYER) {
            isOperational = false;
            clearState();
        }
    }
    else if (dynamic_cast<NodeCrashOperation *>(operation)) {
        if (static_cast<NodeCrashOperation::Stage>(stage) == NodeCrashOperation::STAGE_CRASH) {
            isOperational = false;
            clearState();
        }
    }
    else
        throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());

    return true;
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
    } else {

    }



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
const Ptr<KHOPCAWeight>& KHOPCARouting:: CreateKHOPCAWeight(const L3Address& destAddr){

    auto kwPacket = makeShared<KHOPCAWeight>();

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

void KHOPCARouting::SendKHOPCAWeight(const Ptr<KHOPCAWeight>& packet, const L3Address& destAddr){

    InterfaceEntry *ifEntry = interfaceTable->getInterfaceByName("wlan0");

    //auto className = packet->getClassName();
    Packet *udpPacket = new Packet("inet::khopca::KHOPCAWeight");
    udpPacket->insertAtBack(packet);
    auto udpHeader = makeShared<UdpHeader>();
    udpHeader->setSourcePort(khopcaUDPPort);
    udpHeader->setDestinationPort(khopcaUDPPort);
    udpPacket->insertAtFront(udpHeader);
    // TODO: was udpPacket->copyTags(*packet);
    udpPacket->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::manet);
    udpPacket->addTagIfAbsent<DispatchProtocolReq>()->setProtocol(addressType->getNetworkProtocol());
    udpPacket->addTagIfAbsent<InterfaceReq>()->setInterfaceId(ifEntry->getInterfaceId());
    auto addresses = udpPacket->addTagIfAbsent<L3AddressReq>();
    addresses->setSrcAddress(getSelfIPAddress());
    addresses->setDestAddress(destAddr);

    send(udpPacket, "ipOut");
}

void KHOPCARouting:: HandleKHOPCAWeight(const Ptr<KHOPCAWeight>& kwPacket){

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

void KHOPCARouting::ImplementRules(){

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

    // Displays weight at router/host level
    std::string s = "Weight: " + std::to_string(w_n);
    std::vector<char> v(s.begin(), s.end());
    v.push_back('\0');
    const char* w = &v[0];
    getDisplayString().setTagArg("t", 0, w);

    cModule *cmod = getContainingNode(this);
    cmod ->getDisplayString().setTagArg("t", 0, w);

    EV << this << " : Setting " << s << endl;

    if (w_n == MAX) {
        ratioClusterheads.record(1);
        ratioClusterheadsStats.collect(1);

        clock_t timeToCalc = clock() - startTime;

        printKHOPCAInfo("output.txt", ((float)timeToCalc)/CLOCKS_PER_SEC);
    } else {
        ratioClusterheads.record(0);
        ratioClusterheadsStats.collect(0);
    }

}

// Set up 2d vector of all nodes beneath this one
// 0 index of vector of layers is the one directly beneath the node calling
// the function, then continues down through each vector

void KHOPCARouting::SetConnected(){

    connected.clear();

    // If at minimum level, is not connected to anyone underneath
    // If alone, counts as own cluster
    if(w_n == MIN){
        if(neighbors.size() == 0){
            clusterSize.record(1);
            clusterSize.record(1);
            ratioUnconnectedNodes.record(1);
            ratioUnconnectedStats.collect(1);
        } else {
            ratioUnconnectedNodes.record(0);
            ratioUnconnectedStats.collect(0);
        }
        return;
    } else {
        ratioUnconnectedNodes.record(0);
        ratioUnconnectedStats.collect(0);
    }

    int clustSize = 0;

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
                if(std::find(connected[layerIndex].begin(), connected[layerIndex].end(), *nodesIt) == connected[layerIndex].end()){
                    clustSize++;
                    connected[layerIndex].push_back(*nodesIt);
                }
            }
            layerIndex++;
        }

    }
    if(w_n == MAX){
        clusterSize.record(clustSize);
        clusterSizeStats.collect(clustSize);
    }
}

//
// Converts string version of cluster information sent in message into a 2D Array
//
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
            auto kwPacket = CreateKHOPCAWeight(addressType->getBroadcastAddress());
            SendKHOPCAWeight(kwPacket, addressType->getBroadcastAddress());
            scheduleAt(simTime() + updateInterval, selfMsg);

            ImplementRules();
        }
        else{
            throw cRuntimeError("Unknown Self Message");
        }
    } else {
        auto packet = check_and_cast<Packet *>(msg);
        auto protocol = packet->getTag<PacketProtocolTag>()->getProtocol();

        if (protocol == &Protocol::icmpv4) {
            IcmpHeader *icmpPacket = check_and_cast<IcmpHeader *>(msg);
            // ICMP packet arrived, dropped
            delete icmpPacket;
        } else if (protocol == &Protocol::icmpv6) {
            IcmpHeader *icmpPacket = check_and_cast<IcmpHeader *>(msg);
            // ICMP packet arrived, dropped
            delete icmpPacket;
        } else {
            EV << "Handling KHOPCA Packet" << endl;
            Packet *udpPacket = check_and_cast<Packet *>(msg);
            udpPacket->popAtFront<UdpHeader>();
            L3Address sourceAddr = udpPacket->getTag<L3AddressInd>()->getSrcAddress();
            const auto& ctrlPacket = udpPacket->popAtFront<KHOPCAWeight>();

            HandleKHOPCAWeight(dynamicPtrCast<KHOPCAWeight>(ctrlPacket->dupShared()));

            delete packet;
        }
    }
}

L3Address KHOPCARouting:: getSelfIPAddress() const{
    return routingTable->getRouterIdAsGeneric();
}

void KHOPCARouting::clearState(){
    cancelEvent(selfMsg);
}

KHOPCARouting::KHOPCARouting(){
    EV << "@KHOPCARouting()" << endl;
}

void KHOPCARouting::finish() {

    // Print out duration as cluster head at close
    if(w_n == MAX)
        clusterDuration.record(simTime() - headDur);

    EV << "Stats Recorded" << endl;
    // Cancel and delete remaining messages
    clearState();
    delete selfMsg;

}

KHOPCARouting::~KHOPCARouting(){
    clearState();
}

}
} // end namespace

