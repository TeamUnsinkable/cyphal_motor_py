# cyphal_motor_py
This package runs a Myxa v1.0 motor controller over a CAN network through ROS. This implementation assumes that your telega environement has been sourced and all interfaces are ready to accept CAN traffic.

## Setting up NEW ESC
When recieved from factory, one must first launch yakut monitor with the plug and play functionality enabled. This can be done using the following command:

`y mon --plug-and-play allocation_table.db`

Once the node has obtained an ID it can be assigned to its desired value then restarted for the change to persist and take effect. ID 125 is the default value for a new node however a different ID may be issued if 125 is already taken.

- `y r 125 uavcan.node.id NEW_ID`
- `y cmd 125 restart`

Once the ESCs are set you must assign a few parameters
- `motor.mechanical_ratio: 7`
- `motor.current_max: 35`
- `uavcan.sub.setpoint_rat_torque.id: 65`

