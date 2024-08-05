uint64 INVALID=0

# Set station_id to INVALID if the message is directly
# sent from the traffic signal, not from the RSUs
# In such a case, leave station_pose uninitialised
uint64 station_id

# Set sensor_type to INVALID if the data is
# directly sent from the traffic signal, and is not
# derived from sensors or by any type of perception via sensors
uint64 sensor_type

# Leave station_pose uninitialised if the message does
# not come from RSUs
geometry_msgs/PoseStamped station_pose

dm_signal_info_msgs/SignalInfoArray signal_info_array