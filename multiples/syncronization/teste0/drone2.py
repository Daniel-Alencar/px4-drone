import asyncio
from mavsdk import System

async def connect_and_check(system_address):
    drone = System()
    await drone.connect(system_address=system_address)
    print(f"[Drone 2] Conectando a {system_address}...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[Drone 2] Conectado.")
            break

    async for health in drone.telemetry.health():
        if (
            health.is_gyrometer_calibration_ok and
            health.is_accelerometer_calibration_ok and
            health.is_magnetometer_calibration_ok
        ):
            print("[Drone 2] Sensores prontos.")
            break
    return drone

async def arm_takeoff(drone):
    print("[Drone 2] Armando...")
    await drone.action.arm()
    print("[Drone 2] Decolando...")
    await drone.action.takeoff()
    await asyncio.sleep(10)

async def land(drone):
    print("[Drone 2] Pousando...")
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("[Drone 2] Pousou com sucesso.")
            break

async def main():
    drone = await connect_and_check("udp://:14542")
    input("Aperte ENTER quando quiser iniciar decolagem do Drone 2...")
    await arm_takeoff(drone)
    await asyncio.sleep(10)
    await land(drone)

if __name__ == "__main__":
    asyncio.run(main())
