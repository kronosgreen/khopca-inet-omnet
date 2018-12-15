# khopca-inet-omnet
The K-Hop Clustering Algorithm implemented in OMNeT++ using INET's networking features to simulate ad-hoc communication.

## INET 4.0
Build error in .msg file with imports currently preventing this version from working. Trying to keep project files independent from INET folder.

## INET 3.6
Files go into inet folder, statstracker into common, khopca router into a khopca folder in the node folder, and all other khopca files into a khopca folder in the routing folder. "khopca-test" is then a project on its own that references INET
