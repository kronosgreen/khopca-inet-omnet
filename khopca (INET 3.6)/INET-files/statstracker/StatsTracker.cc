//
//  Stats Tracker Class
//      by Christopher Medrano-Berumen
//      9/30/18
//

#include "StatsTracker.h"
#include "inet/routing/khopca/KHOPCARouting.h"
#include "inet/common/lifecycle/NodeOperations.h"
#include "inet/common/ModuleAccess.h"

#include <fstream>
#include <iostream>

#include <vector>
#include <iterator>

namespace inet {

Define_Module(StatsTracker);

StatsTracker::StatsTracker() {

    // Naming Output Vectors for Statistics Collection
    roleChange.setName("Role Changes");
    clusterDuration.setName("Cluster Duration");
    clusterSize.setName("Cluster Size");
    numClusters.setName("Number of Clusters");
    ratioClusterheads.setName("Ratio of Clusterheads to Non-Clusterheads");
    numMsgExchanged.setName("Number of Messages Exchanged");
    RoCinClusterheads.setName("Rate of Change in # of Clusterheads");
    nodeDelay.setName("Delay");
    ratioUnconnectedNodes.setName("Ratio of Unconnected Nodes");

    roleChange.setRecordDuringWarmupPeriod(false);
    clusterDuration.setRecordDuringWarmupPeriod(false);
    clusterSize.setRecordDuringWarmupPeriod(false);
    numClusters.setRecordDuringWarmupPeriod(false);
    ratioClusterheads.setRecordDuringWarmupPeriod(false);
    numMsgExchanged.setRecordDuringWarmupPeriod(false);
    RoCinClusterheads.setRecordDuringWarmupPeriod(false);
    nodeDelay.setRecordDuringWarmupPeriod(false);
    ratioUnconnectedNodes.setRecordDuringWarmupPeriod(false);

}

void StatsTracker::initialize(int stage) {

    if (stage == INITSTAGE_LOCAL) {


        std::ofstream myfile;
        myfile.open("test.txt");

        numHosts = getParentModule() -> par("numHosts");

        for(cModule::SubmoduleIterator it(getParentModule()); !it.end(); ++it){
            cModule *submodule = *it;
            std::string substr{submodule->getFullName(), submodule->getFullName() + 4};
            if(substr == "host"){
                myfile << "Recording\n";
                KHOPCARouting *kPointer = check_and_cast<KHOPCARouting *>(submodule->getSubmodule("khopca"));
                khopcaNodes.push_back(kPointer);
            }
        }
        myfile.close();
        updateInterval = par("updateInterval");

        if(selfMsg != nullptr){
            cancelAndDelete(selfMsg);
        }
        selfMsg = new cMessage("sendTimer");

    } else if (stage == INITSTAGE_ROUTING_PROTOCOLS) {

        scheduleAt(simTime() + updateInterval, selfMsg);

    }

}

void StatsTracker::handleMessage(cMessage *msg){
    EV << "STATS HANDLE MESSAGE" << endl;
    if(msg -> isSelfMessage()){
        if(msg == selfMsg){

            getSimulationStats();

            scheduleAt(simTime() + updateInterval, selfMsg);


        } else {
            throw cRuntimeError("Unknown Self Message");
        }
    }
}

void StatsTracker::getSimulationStats(){
    int nClusters = 0, clusterHeads = 0, unconnected = 0;

    for(std::vector<KHOPCARouting *>::iterator it = khopcaNodes.begin(); it != khopcaNodes.end(); ++it){
        if((*it) -> isCluster()){
            nClusters++;
        }
        if((*it) -> isClusterHead()){
            clusterSize.record((*it) -> getClusterSize());
            clusterSizeStats.collect((*it) -> getClusterSize());
            clusterHeads++;
        } else if((*it) -> isUnconnectedNode()){
            unconnected++;
        }
    }

    numClusters.record(nClusters);
    numClustersStats.collect(nClusters);

    ratioClusterheads.record((double) clusterHeads/numHosts);
    ratioClusterheadsStats.collect((double) clusterHeads/numHosts);

    ratioUnconnectedNodes.record((double) unconnected/numHosts);
    ratioUnconnectedStats.collect((double) unconnected/numHosts);

}

void StatsTracker::getRateOfChangeHeads(){

}


void StatsTracker::recordDurationHead(double dur){
    try {
        clusterDuration.record(dur);
        clusterDurationStats.collect(dur);
    } catch(std::exception& e){
        EV << "SORRY: " << e.what() << endl;
    }
}

void StatsTracker::recordRoleChange(int rol){
    roleChange.record(rol);
    roleChangeStats.collect(rol);
}

void StatsTracker::recordExcMesg(int exMsg){
    numMsgExchanged.record(exMsg);
    numMsgExchangedStats.collect(exMsg);
}

void StatsTracker::recordDelay(double delay){
    nodeDelay.record(delay);
    nodeDelayStats.collect(delay);
}


void StatsTracker::clearState(){
    if(selfMsg->isScheduled()){
        cancelEvent(selfMsg);
    }
    //cancelEvent(selfMsg);
}

StatsTracker::~StatsTracker(){

    clearState();
    delete selfMsg;
}

} // end namespace
