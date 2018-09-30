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

package inet.routing.khopca;



simple KHOPCARouting
{
    //code from AODV Routing - to be changed
    parameters:
        @display("i=block/network2");
        @display("t=WOW");
        string routingTableModule = default("^.routingTable");
        string interfaceTableModule = default("^.interfaceTable");
        string networkProtocolModule = default("^.networkLayer.ip");
		string energyStorage = default("^.energyStorage");
		string destAddress = default("");
        int udpPort = default(654);
		
		int MAX = default(5);
		int MIN = default(1);
		int w_n = default(MIN);
		double updateInterval = default(3);
		
    gates:
        input ipIn;
        output ipOut;
        
}