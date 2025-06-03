import asyncio
from mavsdk import System


# COMANDOS BÁSICOS
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


async def arm_takeoff(drone, label):
    print(f"🚁 [{label}] Armando...")
    await drone.action.arm()
    
    print(f"🚀 [{label}] Decolando...")
    await drone.action.takeoff()
    await asyncio.sleep(10)  


async def land(drone, label):
    print(f"🛬 [{label}] Pousando...")
    await drone.action.land()

    # Aguarda pouso completo
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print(f"✅ [{label}] Drone pousou com sucesso.")
            break
    # Simula algum tempo de operação
    await asyncio.sleep(10)


# EXECUÇÕES
async def main_drone1():
    # Conecte aos drones nas portas apropriadas
    drone1 = await connect_and_check("udp://:14541")

    # Decola e pousa ambos simultaneamente
    await arm_takeoff(drone1, "Drone 1")

async def main_drone2():
    # Conecte aos drones nas portas apropriadas
    drone2 = await connect_and_check("udp://:14542")

    # Decola e pousa ambos simultaneamente
    await arm_takeoff(drone2, "Drone 2")

async def land_drone1():
    # Conecte aos drones nas portas apropriadas
    drone1 = await connect_and_check("udp://:14541")

    await land(drone1, "Drone 1")

async def land_drone2():
    # Conecte aos drones nas portas apropriadas
    drone2 = await connect_and_check("udp://:14542")

    await land(drone2, "Drone 2")

if __name__ == "__main__":
    asyncio.run(main_drone1())
    asyncio.run(main_drone2())
    asyncio.run(asyncio.sleep(10))
    asyncio.run(land_drone1())
    asyncio.run(land_drone2())
