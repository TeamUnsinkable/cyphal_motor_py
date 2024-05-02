import os
import pycyphal  # Importing PyCyphal will automatically install the import hook for DSDL compilation.
import pycyphal.application

# DSDL Message Types
import uavcan.node # noqa
import uavcan.node.heartbeat
import uavcan.si.sample.temperature  # noqa
import uavcan.si.unit.temperature  # noqa
import uavcan.si.unit.voltage  # noqa

import zubax

class CyphalMotor():
    def __init__(self, esc_id: int) -> None:
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=0, minor=1),
            name="org.amra.cyphal_motor_py.cyphal_motor",
        )
        self._node = pycyphal.application.make_node(node_info)
        
        # Create Subscribers
        self.heartbeat_sub = self._node.make_subscriber(uavcan.node.Heartbeat_1)
        self.heartbeat_sub.receive_in_background(self.heartbeat_cb)

        self._node.start()
    
    def heartbeat_cb(self, msg: uavcan.node.Heartbeat_1, transport_data: pycyphal.transport.TransferFrom) -> None:
        print(f"Got Message State: {msg.value}")
        print(transport_data)

    def close(self):
        self._node.close()

def main():
    CyphalMotor(5600)

if __name__ == "__main__":
    main()