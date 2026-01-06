import asyncio
from mavsdk import System


async def connect_and_check(system_address):
    drone = System()
    await drone.connect(system_address=system_address)

    # Espera conexão
    print(f"🔌 Conectando ao drone em {system_address}...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"✅ Conectado ao drone em {system_address}")
            break

    # Espera sensores estarem prontos
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


async def arm_takeoff_land(drone, label):
    print(f"🚁 [{label}] Armando...")
    await drone.action.arm()
    
    print(f"🚀 [{label}] Decolando...")
    await drone.action.takeoff()
    await asyncio.sleep(15)  # Tempo no ar

    print(f"🛬 [{label}] Pousando...")
    await drone.action.land()

    # Aguarda pouso completo
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print(f"✅ [{label}] Drone pousou com sucesso.")
            break
            


async def main():
    # Conecte aos drones nas portas apropriadas
    drone1 = await connect_and_check("udp://:14540")

    # Decola e pousa ambos simultaneamente
    await asyncio.gather(
        arm_takeoff_land(drone1, "Drone 1")
    )

if __name__ == "__main__":
    asyncio.run(main())
