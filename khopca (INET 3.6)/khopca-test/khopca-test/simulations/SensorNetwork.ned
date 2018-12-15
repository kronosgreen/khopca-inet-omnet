//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//
package khopca_test.simulations;

import inet.common.figures.DelegateSignalConfigurator;
import inet.networklayer.configurator.ipv4.IPv4NetworkConfigurator;
import inet.node.khopca.KHOPCARouter;
import inet.node.inet.INetworkNode;
import inet.physicallayer.contract.packetlevel.IRadioMedium;
import inet.visualizer.integrated.IntegratedCanvasVisualizer;
import inet.environment.common.PhysicalEnvironment;
import inet.common.lifecycle.LifecycleController;
import inet.common.statstracker.StatsTracker;

network SensorNetwork
{
    parameters:
        @isNetwork;
        @display("bgb=600,600;bgi=100,1,gray");
        int numHosts = default(0);
        string hostType = default("KHOPCARouter");
        string mediumType = default("IdealRadioMedium");
        @figure[title](type=label; pos=0,-1; anchor=sw; color=darkblue);
        
    submodules:
        
        stats: StatsTracker {
            @display("p=500,100");
        }

        visualizer: IntegratedCanvasVisualizer {
            @display("p=500,150");
        }
        configurator: IPv4NetworkConfigurator {
            @display("p=500,200");
        }
        radioMedium: <mediumType> like IRadioMedium {
            @display("p=500,250");
        }
        figureHelper: DelegateSignalConfigurator {
            @display("p=500,300");
        }
        lifecycleController: LifecycleController {
            @display("p=500,350");
        }

        host[numHosts]: <hostType> like INetworkNode {
        }

    connections allowunconnected:

}

 //##### (marker for doxygen)
