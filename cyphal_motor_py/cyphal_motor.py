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
import zubax.service.actuator



class CyphalMotor():
    def __init__(self, status_id: int) -> None:
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=0, minor=1),
            name="org.amra.cyphal_motor_py.cyphal_motor",
        )
        self._node = pycyphal.application.make_node(node_info)
        
        # Create Subscribers
        self.status_sub = self._node.make_subscriber(zubax.service.actuator.Status_1, status_id)

        self._node.start()

    async def run(self):
        async for m, _metadata in self.status_sub:
            print(f"Recieved STATUS message: {m}")
            break
        
        
        

    async def status_cb(self, msg: uavcan.node.Heartbeat_1, transport_data: pycyphal.transport.TransferFrom) -> None:
        print(f"Got Message State: {msg.value}")
        print(transport_data)

    def close(self):
        self._node.close()

async def main():
    esc = CyphalMotor(65)
    try:
        await esc.run()
    except KeyboardInterrupt:
        pass
    finally:
        esc.close()
    

if __name__ == "__main__":
    asyncio.run(main())
