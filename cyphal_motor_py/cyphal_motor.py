import os
import asyncio
import pycyphal  # Importing PyCyphal will automatically install the import hook for DSDL compilation.
import pycyphal.application
import numpy as np

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
import zubax.primitive.scalar



class CyphalMotor():
    def __init__(self, motor_id: int, status_id: int, rat_setpoint_id: int, readiness_id: int) -> None:
        # Generate information for Yakut
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=0, minor=1),
            name="org.amra.cyphal_motor_py.cyphal_motor_"+str(motor_id),
        )
        # Create Node on network
        self._node = pycyphal.application.make_node(node_info)
        
        # Create Subscribers
        self.status_sub = self._node.make_subscriber(zubax.service.actuator.Status_1, status_id)
        self.status_sub.receive_in_background(self.recieve_status)
        # We use rat_torque
        self.setpoint_pub = self._node.make_publisher(zubax.primitive.real16.Vector31_1, rat_setpoint_id)
        self.readiness_pub = self._node.make_publisher(zubax.service.Readiness_1, readiness_id)

        # Start node
        self._node.start()

    @staticmethod
    async def recieve_status(msg, transfer_data):
        # Wait for message, returns None of not recieved
        print(f"I got STATUS: {msg}")
        

    
    async def publish_readiness(self, readiness: int):
        if readiness not in [0, 2, 3]:
            raise ValueError("Readiness is not within range")
        # Create Message
        readiness_msg = zubax.service.Readiness_1(readiness)
        # Return publish result
        return await self.setpoint_pub.publish(readiness_msg)
    
    
    async def publish_setpoint(self, setpoint: float, setpoint_index: int):
        # Check if within limits
        if setpoint < -1 or setpoint > 1:
            raise ValueError("RATIO Setpoint ust be between -1.0 and 1.0")
        # Create Values
        setpoint_values = [0.0]*31
        setpoint_values[setpoint_index] = setpoint
        # Create Message
        setpoint_msg = zubax.primitive.real16.Vector31_1(setpoint_values)
        # Return publish result
        return await self.setpoint_pub.publish(setpoint_msg)

        
        
    def close(self):
        self._node.close()

async def main():
    esc = CyphalMotor(1, 155, 65, 45)
    try:
        setpoint = input("Enter a ratio_setpoint: ") 
        setpoint = float(setpoint) # * max => scaled_setpoint
        await esc.publish_readiness(3) 
        while (1):
            # await esc.recieve_status()
            await esc.publish_setpoint(setpoint, 0) # type: ignore
        await esc.publish_readiness(0)
    except KeyboardInterrupt:
        pass
    finally:
        esc.close()
    

if __name__ == "__main__":
    asyncio.run(main())
