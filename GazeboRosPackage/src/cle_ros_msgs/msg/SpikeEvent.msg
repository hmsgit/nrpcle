# This message contains the information whether a given neuron monitor received spikes at a given simulation time
# Multiple monitors with different names may be used to monitor different neurons
float32 simulationTime
int32 neuronCount
cle_ros_msgs/SpikeData[] spikes
string monitorName
