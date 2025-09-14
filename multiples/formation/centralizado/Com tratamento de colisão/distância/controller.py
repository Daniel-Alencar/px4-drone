import multiprocessing as mp
import asyncio
import time
from mavsdk import System

ALTITUDE = 3.0

grids = [
    [
        [0, 0, 1],
        [0, 0, 0],
        [2, 0, 0],
    ],
    [
        [1, 0, 0],
        [0, 0, 0],
        [0, 0, 2],
    ],
    [
        [2, 0, 0],
        [0, 0, 0],
        [0, 0, 1],
    ],
    [
        [0, 0, 1],
        [0, 0, 0],
        [2, 0, 0],
    ]
]

import math

def distancia(p1, p2):
    """Calcula distância Euclidiana em 3D"""
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

def ajustar_offsets(offsets, min_dist):
    """
    Verifica se os offsets geram colisão (distância < min_dist).
    Se sim, aumenta a altitude de um dos drones temporariamente.
    """
    o1 = offsets[1]
    o2 = offsets[2]

    if distancia(o1, o2) < min_dist:
        print("⚠️ Distância muito curta entre drones! Ajustando altitude...")
        # sobe o drone 2 um pouco
        o2 = (o2[0], o2[1], o2[2] + 1.5)  # sobe 1.5m
    return {1: o1, 2: o2}

# -----------------------
# Utilitários do worker
# -----------------------
async def connect_and_check(system_address: str, label: str, grpc_port: int) -> System:
    drone = System(port=grpc_port)
    await drone.connect(system_address=system_address)

    print(f"[{label}] 🔌 Conectando em {system_address} (gRPC {grpc_port})...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[{label}] ✅ Conectado.")
            break

    print(f"[{label}] ⏳ Verificando sensores...")
    async for health in drone.telemetry.health():
        if (health.is_gyrometer_calibration_ok
            and health.is_accelerometer_calibration_ok
            and health.is_magnetometer_calibration_ok):
            print(f"[{label}] ✅ Sensores prontos.")
            break
    return drone


async def arm_takeoff(drone: System, label: str, target_alt: float = ALTITUDE):
    print(f"[{label}] 🚁 Armando...")
    await drone.action.arm()
    print(f"[{label}] 🚀 Decolando...")
    await drone.action.takeoff()
    await asyncio.sleep(8)


async def get_position(drone: System):
    """ Pega posição global do drone (lat, lon, alt) """
    async for pos in drone.telemetry.position():
        return pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m


async def goto_relative(drone: System, label: str, ref_pos, offset_north, offset_east, offset_alt):
    """
    Move drone para posição relativa em relação a ref_pos (lat, lon, alt).
    """
    # Conversão aproximada: 1 grau de latitude ~ 111.320 m
    dlat = offset_north / 111320.0
    dlon = offset_east / (111320.0 * abs(ref_pos[0]) * 0.0174533 if abs(ref_pos[0]) > 1e-6 else 111320.0)

    target_lat = ref_pos[0] + dlat
    target_lon = ref_pos[1] + dlon
    target_alt = ref_pos[2] + offset_alt

    print(f"[{label}] ✈️ Indo para offset N={offset_north}m, E={offset_east}m, ALT={offset_alt}m")
    await drone.action.goto_location(target_lat, target_lon, target_alt, 0)


async def land(drone: System, label: str):
    print(f"[{label}] 🛬 Pousando...")
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print(f"[{label}] ✅ Pousou com sucesso.")
            break


# -----------------------
# Worker de um drone
# -----------------------
def drone_worker(system_address: str, label: str, grpc_port: int,
                 cmd_queue: mp.Queue, resp_queue: mp.Queue,
                 ready_evt: mp.Event, start_evt: mp.Event):
    async def runner():
        drone = await connect_and_check(system_address, label, grpc_port)
        ready_evt.set()

        while True:
            cmd = cmd_queue.get()
            if cmd[0] == "takeoff":
                start_evt.wait()
                await arm_takeoff(drone, label, target_alt=cmd[1])
            elif cmd[0] == "getpos":
                pos = await get_position(drone)
                # devolve posição na fila certa
                resp_queue.put(("pos_result", pos))
            elif cmd[0] == "goto":
                ref_pos, offset = cmd[1]
                await goto_relative(drone, label, ref_pos, *offset)
            elif cmd[0] == "land":
                await land(drone, label)
            elif cmd[0] == "exit":
                print(f"[{label}] Encerrando processo.")
                break

    asyncio.run(runner())



# -----------------------
# Função auxiliar para matriz
# -----------------------
def parse_grid(grid, cell_size=1, flight_alt=ALTITUDE):
    """
    Converte matriz em deslocamentos relativos.
    Retorna dict {drone_id: (north, east, alt)}.
    """
    offsets = {}
    rows = len(grid)
    cols = len(grid[0])
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] != 0:
                drone_id = grid[r][c]
                north = -r * cell_size
                east = c * cell_size
                offsets[drone_id] = (north, east, flight_alt)
    return offsets


# -----------------------
# Controlador
# -----------------------


if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)

    # Filas de comando e resposta
    q1, q2 = mp.Queue(), mp.Queue()
    r1, r2 = mp.Queue(), mp.Queue()
    ready1, ready2 = mp.Event(), mp.Event()
    start_both = mp.Event()

    DRONE1 = dict(system_address="udp://:14541", label="Drone 1", grpc_port=50051)
    DRONE2 = dict(system_address="udp://:14542", label="Drone 2", grpc_port=50052)

    p1 = mp.Process(target=drone_worker, args=(
        DRONE1["system_address"], DRONE1["label"], DRONE1["grpc_port"], q1, r1, ready1, start_both
    ))
    p2 = mp.Process(target=drone_worker, args=(
        DRONE2["system_address"], DRONE2["label"], DRONE2["grpc_port"], q2, r2, ready2, start_both
    ))

    p1.start()
    p2.start()

    print("⌛ Aguardando drones ficarem prontos...")
    ready1.wait()
    ready2.wait()

    # --- Decolagem simultânea ---
    print("\n🚀 Iniciando decolagem conjunta!\n")
    q1.put(("takeoff", ALTITUDE))
    q2.put(("takeoff", ALTITUDE))
    time.sleep(0.5)
    start_both.set()

    # --- Define posição de referência ---
    print("\n📍 Pegando posição de referência do Drone 1\n")
    q1.put(("getpos", None))
    ref_pos = None
    while ref_pos is None:
        # pega da fila de resposta
        msg = r1.get()
        if msg[0] == "pos_result":
            ref_pos = msg[1]

    for grid in grids:
        # --- Matriz de formação com checagem de colisão ---
        offsets = parse_grid(grid)
        offsets = ajustar_offsets(offsets, min_dist=5.0)

        # --- Move drones para offsets relativos ---
        print("\nMovendo drones para formação relativa\n")
        q1.put(("goto", (ref_pos, offsets[1])))
        q2.put(("goto", (ref_pos, offsets[2])))
        time.sleep(5)


    print("\n🛬 Pousando ambos!\n")
    q1.put(("land", None))
    q2.put(("land", None))

    time.sleep(5)
    q1.put(("exit", None))
    q2.put(("exit", None))

    p1.join()
    p2.join()
    print("✅ Missão finalizada.")
