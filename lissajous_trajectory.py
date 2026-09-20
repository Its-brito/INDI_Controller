import asyncio
import math
import time
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw, AccelerationNed)

async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Connecting to PX4...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    print("Arming and taking off to hover...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(20.0)

    # Send an initial setpoint before requesting Offboard mode
    initial_pos = PositionNedYaw(0.0, 0.0, -15.0, 0.0)
    await drone.offboard.set_position_ned(initial_pos)

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard mode failed: {error}")
        return

    await asyncio.sleep(5)  # Allow drone to reach starting altitude

    print("Executing 3D Lissajous Figure-8...")
    
    # Trajectory parameters
    A = 20.0      # North amplitude (m)
    B = 20.0      # East amplitude (m)
    C = 10.0       # Down amplitude (m)
    omega = 0.5   # Angular frequency (rad/s)
    h0 = 20.0     # Base altitude (m)
    
    t0 = time.time()
    while True:
        t = time.time() - t0
        if t > 26.13:  # Run for two full periods (4*pi / 0.5)
            break
            
        # Kinematics Math
        x = A * math.sin(omega * t)
        y = B * math.sin(2 * omega * t)
        z = -h0 - C * math.sin(omega * t)
        
        vx = A * omega * math.cos(omega * t)
        vy = 2 * B * omega * math.cos(2 * omega * t)
        vz = -C * omega * math.cos(omega * t)
        
        ax = -A * omega**2 * math.sin(omega * t)
        ay = -4 * B * omega**2 * math.sin(2 * omega * t)
        az = C * omega**2 * math.sin(omega * t)
        
        # Align vehicle nose to the velocity vector
        yaw = math.degrees(math.atan2(vy, vx))
        
        await drone.offboard.set_position_velocity_acceleration_ned(
            PositionNedYaw(x, y, z, yaw),
            VelocityNedYaw(vx, vy, vz, yaw),
            AccelerationNed(ax, ay, az)
        )
        
        await asyncio.sleep(0.05) # 20 Hz update rate

    print("Mission complete. Landing...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -15.0, 0.0))
    await asyncio.sleep(3)
    await drone.offboard.stop()
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())