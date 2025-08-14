import asyncio
from mavsdk import System

# -----------------------
# COMANDOS BÁSICOS
# -----------------------
async def connect_and_check(system_address):
    drone = System()
    await drone.connect(system_address=system_address)

    print(f"🔌 Conectando ao drone em {system_address}...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"✅ Conectado ao drone em {system_address}")
            break

    print(f"⏳ Verificando sensores em {system_address}...")
    async for health in drone.telemetry.health():
        if (
            health.is_gyrometer_calibration_ok and
            health.is_accelerometer_calibration_ok and
            health.is_magnetometer_calibration_ok
        ):
            print(f"✅ Sensores prontos em {system_address}")
            break
    return drone

async def arm_takeoff(drone, label):
    print(f"🚁 [{label}] Armando...")
    await drone.action.arm()
    print(f"🚀 [{label}] Decolando...")
    await drone.action.takeoff()
    await asyncio.sleep(10)

async def land(drone, label):
    print(f"🛬 [{label}] Pousando...")
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print(f"✅ [{label}] Drone pousou com sucesso.")
            break
    await asyncio.sleep(10)

# -----------------------
# EXECUÇÃO PARALELA
# -----------------------
async def main():
    # Conecta aos dois drones
    drone1 = await connect_and_check("udp://:14541")
    drone2 = await connect_and_check("udp://:14542")

    # Decola os dois simultaneamente
    await asyncio.gather(
        arm_takeoff(drone1, "Drone 1"),
        arm_takeoff(drone2, "Drone 2")
    )

    # Espera 10 segundos no ar
    await asyncio.sleep(10)

    # Pousa os dois simultaneamente
    await asyncio.gather(
        land(drone1, "Drone 1"),
        land(drone2, "Drone 2")
    )

if __name__ == "__main__":
    asyncio.run(main())
