import os
import asyncio
import pycyphal  # Importing PyCyphal will automatically install the import hook for DSDL compilation.
import pycyphal.application

# DSDL Message Types
import uavcan.node # noqa
import uavcan.si.sample.temperature  # noqa
import uavcan.si.unit.temperature  # noqa
import uavcan.si.unit.voltage  # noqa

import zubax
import zubax.service
import zubax.primitive
import zubax.service.actuator
import zubax.primitive.real16




class CyphalMotor():
    def __init__(self, status_id: int, rat_setpoint_id: int) -> None:
        # Generate information for Yakut
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=0, minor=1),
            name="org.amra.cyphal_motor_py.cyphal_motor",
        )
        # Create Node on network
        self._node = pycyphal.application.make_node(node_info)
        
        # Create Subscribers
        self.status_sub = self._node.make_subscriber(zubax.service.actuator.Status_1, status_id)
        # We use rat_torque
        self.setpoint_pub = self._node.make_publisher(zubax.primitive.real16.Vector31_1, rat_setpoint_id)

        # Start node
        self._node.start()

    async def recieve_status(self) -> zubax.service.actuator.Status_1:
        # Wait for message, returns None of not recieved
        result = await self.status_sub.receive_for(100)
        if result is not None:
            # result is actully tuple
            message, transfer_data = result
            print(f"I got STATUS: {message}")
            print(transfer_data)
        else:
            print("I did not get a message")        
        
    def close(self):
        self._node.close()

async def main():
    esc = CyphalMotor(155, 65)
    try:
        await esc.recieve_status()
    except KeyboardInterrupt:
        pass
    finally:
        esc.close()
    

if __name__ == "__main__":
    asyncio.run(main())
