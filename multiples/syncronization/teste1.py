import asyncio
from mavsdk import System


# MATRIZ DE POSIÇÃO
MATRIX = [
    [0, 0, 1],
    [0, 0, 0],
    [2, 0, 0]
]

# Encontra posição de um drone na matriz
def find_drone_position(drone_id):
    for y, row in enumerate(MATRIX):
        for x, value in enumerate(row):
            if value == drone_id:
                return (x, y)
    return None


# ========================
# COMANDOS BÁSICOS
# ========================
async def connect_and_check(system_address):
    drone = System()
    await drone.connect(system_address=system_address)

    print(f"Conectando ao drone em {system_address}...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Conectado ao drone em {system_address}")
            break

    print(f"Verificando sensores em {system_address}...")
    async for health in drone.telemetry.health():
        if (
            health.is_gyrometer_calibration_ok and
            health.is_accelerometer_calibration_ok and
            health.is_magnetometer_calibration_ok
        ):
            print(f"Sensores prontos em {system_address}")
            break
    return drone


async def arm_takeoff(drone, label):
    print(f"[{label}] Armando...")
    await drone.action.arm()
    print(f"[{label}] Decolando...")
    await drone.action.takeoff()
    await asyncio.sleep(10)


async def move_relative(drone, label, x, y):
    # Obter posição atual
    async for pos in drone.telemetry.position():
        lat_base = pos.latitude_deg
        lon_base = pos.longitude_deg
        break  # pega só a primeira leitura

    # Escala do deslocamento (em graus)
    LAT_STEP = 0.00001  # ~1.11 m
    LON_STEP = 0.00001  # ~1.11 m (depende da latitude, mas ok para teste)

    lat_target = lat_base + (y * LAT_STEP)
    lon_target = lon_base + (x * LON_STEP)

    print(f"[{label}] Deslocando para posição relativa ({x}, {y}) -> GPS approx: ({lat_target}, {lon_target})")
    await drone.action.goto_location(lat_target, lon_target, pos.relative_altitude_m, 0)
    await asyncio.sleep(5)


async def land(drone, label):
    print(f"[{label}] Pousando...")
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print(f"[{label}] Drone pousou com sucesso.")
            break
    await asyncio.sleep(5)


# ========================
# EXECUÇÕES DE DRONES
# ========================
async def main_drone1():
    pos = find_drone_position(1)
    if not pos:
        print("Drone 1 não encontrado na matriz.")
        return
    drone1 = await connect_and_check("udp://:14541")
    await arm_takeoff(drone1, "Drone 1")
    await move_relative(drone1, "Drone 1", *pos)


async def main_drone2():
    pos = find_drone_position(2)
    if not pos:
        print("Drone 2 não encontrado na matriz.")
        return
    drone2 = await connect_and_check("udp://:14542")
    await arm_takeoff(drone2, "Drone 2")
    await move_relative(drone2, "Drone 2", *pos)


async def land_drone1():
    drone1 = await connect_and_check("udp://:14541")
    await land(drone1, "Drone 1")


async def land_drone2():
    drone2 = await connect_and_check("udp://:14542")
    await land(drone2, "Drone 2")


# ========================
# ROTEIRO
# ========================
if __name__ == "__main__":
    asyncio.run(main_drone1())
    asyncio.run(main_drone2())
    asyncio.run(asyncio.sleep(10))
    asyncio.run(land_drone1())
    asyncio.run(land_drone2())
