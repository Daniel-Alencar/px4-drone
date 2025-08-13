import asyncio
from mavsdk import System


# MATRIZ DE POSIÇÃO
# 0 0 1
# 0 0 0
# 2 0 0
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
                return (x, y)  # posição no plano
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


async def move_to_position(drone, label, x, y):
    # Conversão simples de coordenadas da matriz para lat/lon fictício
    # (Na prática, você deve usar a posição GPS inicial do drone e aplicar deslocamento)
    lat_base = -3.0
    lon_base = -60.0
    lat = lat_base + y * 0.00001
    lon = lon_base + x * 0.00001

    print(f"[{label}] Indo para posição ({x}, {y}) -> GPS approx: ({lat}, {lon})")
    await drone.action.goto_location(lat, lon, 10, 0)  # altitude 10m
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
    await move_to_position(drone1, "Drone 1", *pos)


async def main_drone2():
    pos = find_drone_position(2)
    if not pos:
        print("Drone 2 não encontrado na matriz.")
        return
    drone2 = await connect_and_check("udp://:14542")
    await arm_takeoff(drone2, "Drone 2")
    await move_to_position(drone2, "Drone 2", *pos)


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
