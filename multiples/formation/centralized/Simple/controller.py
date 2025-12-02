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
        [0, 0, 2],
        [0, 0, 0],
        [1, 0, 0],
    ],
    [
        [2, 0, 0],
        [0, 0, 0],
        [0, 0, 1],
    ]
]

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
def parse_grid(grid, cell_size=2, flight_alt=ALTITUDE):
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

    # Número de drones
    n_drones = 5

    # --- Criação das filas e eventos ---
    queues_cmd = [mp.Queue() for _ in range(n_drones)]
    queues_resp = [mp.Queue() for _ in range(n_drones)]
    events_ready = [mp.Event() for _ in range(n_drones)]
    start_all = mp.Event()

    # --- Configuração dos drones ---
    base_system_port = 14540
    base_grpc_port = 50050
    drones = [
        dict(
            system_address=f"udp://:{base_system_port + i + 1}",
            label=f"Drone {i + 1}",
            grpc_port=base_grpc_port + i + 1,
        )
        for i in range(n_drones)
    ]

    # --- Criação e inicialização dos processos ---
    processes = []
    for i, drone in enumerate(drones):
        p = mp.Process(
            target=drone_worker,
            args=(
                drone["system_address"],
                drone["label"],
                drone["grpc_port"],
                queues_cmd[i],
                queues_resp[i],
                events_ready[i],
                start_all,
            ),
        )
        p.start()
        processes.append(p)

    print("⌛ Aguardando drones ficarem prontos...")
    for ev in events_ready:
        ev.wait()

    # --- Decolagem simultânea ---
    print("\n🚀 Iniciando decolagem conjunta!\n")
    for q in queues_cmd:
        q.put(("takeoff", ALTITUDE))
    time.sleep(0.5)
    start_all.set()

    # --- Define posição de referência ---
    print("\n📍 Pegando posição de referência do Drone 1\n")
    q1, r1 = queues_cmd[0], queues_resp[0]
    q2, r2 = queues_cmd[1], queues_resp[1]  # Apenas drone 1 e 2 usados abaixo

    q1.put(("getpos", None))
    ref_pos = None
    while ref_pos is None:
        msg = r1.get()
        if msg[0] == "pos_result":
            ref_pos = msg[1]

    # --- Movimentação em formação (apenas drones 1 e 2) ---
    for grid in grids:
        offsets = parse_grid(grid)
        print("\nMovendo drones para formação relativa\n")
        q1.put(("goto", (ref_pos, offsets[1])))
        q2.put(("goto", (ref_pos, offsets[2])))
        time.sleep(5)

    # --- Pouso e finalização ---
    print("\n🛬 Pousando!\n")
    for q in queues_cmd:
        q.put(("land", None))

    time.sleep(5)
    for q in queues_cmd:
        q.put(("exit", None))

    for p in processes:
        p.join()

    print("✅ Missão finalizada.")
