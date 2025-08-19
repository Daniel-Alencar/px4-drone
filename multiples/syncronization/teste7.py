import asyncio
from mavsdk import System
import math

# ========================
# CONFIGURAÇÃO
# ========================
MATRICES = [
    [
        [0, 0, 1],
        [0, 0, 0],
        [0, 0, 2]
    ],
    [
        [1, 0, 2],
        [0, 0, 0],
        [0, 0, 0]
    ],
    [
        [0, 0, 0],
        [0, 0, 0],
        [1, 0, 2]
    ]
]

LAT_STEP = 0.00001    # ~1.11 m
LON_STEP = 0.00001    # ~1.11 m
MIN_DISTANCE = 2.0    # metros
STEP_SIZE = 0.000001  # tamanho de cada passo (~0.11 m)
CRUISE_ALT = 3        # altura de cruzeiro

# Guarda posições de cada drone (drone_id -> (lat, lon, alt))
drone_positions = {}

REFERENCE_LAT = None
REFERENCE_LON = None

# ========================
# FUNÇÕES AUXILIARES
# ========================
def find_drone_position(matrix, drone_id):
    for y, row in enumerate(matrix):
        for x, value in enumerate(row):
            if value == drone_id:
                return (x, y)
    return None

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))

async def connect_and_check(system_address, grpc_port):
    drone = System(port=grpc_port)
    await drone.connect(system_address=system_address)
    print(f"Conectando ao drone em {system_address}...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Conectado ao drone em {system_address}")
            break
    print(f"Verificando sensores em {system_address}...")
    async for health in drone.telemetry.health():
        if (health.is_gyrometer_calibration_ok and
            health.is_accelerometer_calibration_ok and
            health.is_magnetometer_calibration_ok):
            print(f"Sensores prontos em {system_address}")
            break
    return drone

async def arm_takeoff(drone, label, target_alt):
    print(f"[{label}] Armando...")
    await drone.action.arm()
    print(f"[{label}] Decolando para {target_alt} m...")
    await drone.action.set_takeoff_altitude(target_alt)
    await drone.action.takeoff()
    await asyncio.sleep(10)

async def update_drone_position(drone_id, drone):
    async for pos in drone.telemetry.position():
        drone_positions[drone_id] = (pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m)
        break

# ========================
# FUNÇÃO DE MOVIMENTO POR PASSO
# ========================
async def move_step(drone_id, drone, target_x, target_y, cruise_alt):
    global REFERENCE_LAT, REFERENCE_LON
    if REFERENCE_LAT is None or REFERENCE_LON is None:
        async for pos in drone.telemetry.position():
            REFERENCE_LAT = pos.latitude_deg
            REFERENCE_LON = pos.longitude_deg
            print(f"[Drone {drone_id}] Definindo referência global: ({REFERENCE_LAT}, {REFERENCE_LON})")
            break

    target_lat = REFERENCE_LAT + (target_y * LAT_STEP)
    target_lon = REFERENCE_LON + (target_x * LON_STEP)

    await update_drone_position(drone_id, drone)
    lat, lon, alt = drone_positions[drone_id]

    dist_to_goal = haversine(lat, lon, target_lat, target_lon)
    if dist_to_goal < 0.3:
        return True  # já chegou

    dir_lat = target_lat - lat
    dir_lon = target_lon - lon
    length = math.sqrt(dir_lat**2 + dir_lon**2)
    dir_lat /= length
    dir_lon /= length

    next_lat = lat + dir_lat * STEP_SIZE
    next_lon = lon + dir_lon * STEP_SIZE
    next_alt = cruise_alt

    # prevenção de colisão
    for other_id, (olat, olon, oalt) in drone_positions.items():
        if other_id != drone_id:
            d = haversine(next_lat, next_lon, olat, olon)
            if d < MIN_DISTANCE and abs(oalt - next_alt) < 1.0:
                next_alt += 3

    await drone.action.goto_location(next_lat, next_lon, next_alt, 0)
    await asyncio.sleep(0.3)
    await update_drone_position(drone_id, drone)
    return False  # ainda não chegou

async def land(drone, label):
    print(f"[{label}] Pousando...")
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print(f"[{label}] Drone pousou com sucesso.")
            break
    await asyncio.sleep(5)

# ========================
# FUNÇÕES PRINCIPAIS DOS DRONES
# ========================
async def main_drone1():
    drone1 = await connect_and_check("udp://:14541", 50051)
    await arm_takeoff(drone1, "Drone 1", CRUISE_ALT)

async def main_drone2():
    drone2 = await connect_and_check("udp://:14542", 50052)
    await arm_takeoff(drone2, "Drone 2", CRUISE_ALT)

async def move_drones_sequence():
    # Para cada matriz
    for matrix in MATRICES:
        pos1 = find_drone_position(matrix, 1)
        pos2 = find_drone_position(matrix, 2)

        arrived1 = arrived2 = False
        # passo a passo até ambos chegarem
        while not (arrived1 and arrived2):
            if pos1 and not arrived1:
                drone1 = await connect_and_check("udp://:14541", 50051)
                arrived1 = await move_step(1, drone1, *pos1, CRUISE_ALT)
            if pos2 and not arrived2:
                drone2 = await connect_and_check("udp://:14542", 50052)
                arrived2 = await move_step(2, drone2, *pos2, CRUISE_ALT)

async def land_drone1():
    drone1 = await connect_and_check("udp://:14541", 50051)
    await land(drone1, "Drone 1")

async def land_drone2():
    drone2 = await connect_and_check("udp://:14542", 50052)
    await land(drone2, "Drone 2")

# ========================
# EXECUÇÃO
# ========================
if __name__ == "__main__":
    asyncio.run(main_drone1())
    asyncio.run(main_drone2())
    asyncio.run(move_drones_sequence())
    asyncio.run(asyncio.sleep(10))
    asyncio.run(land_drone1())
    asyncio.run(land_drone2())
