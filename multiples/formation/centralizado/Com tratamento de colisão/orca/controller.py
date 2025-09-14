import multiprocessing as mp
import asyncio
import time
from mavsdk import System

from mavsdk.offboard import OffboardError, VelocityNedYaw


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
import time

def latlon_to_local_ned(lat_ref, lon_ref, lat, lon):
    # Aproximação: 1 deg lat ~ 111_320 m; lon depende da latitude
    k_lat = 111_320.0
    k_lon = 111_320.0 * math.cos(math.radians(lat_ref))
    n = (lat - lat_ref) * k_lat
    e = (lon - lon_ref) * k_lon
    return n, e

def local_ned_to_latlon(lat_ref, lon_ref, n, e):
    k_lat = 111_320.0
    k_lon = 111_320.0 * math.cos(math.radians(lat_ref))
    lat = lat_ref + n / k_lat
    lon = lon_ref + e / k_lon
    return lat, lon

def vec_norm(x, y):
    return math.hypot(x, y)

def clamp_speed(vn, ve, vmax):
    speed = vec_norm(vn, ve)
    if speed <= vmax or speed < 1e-6:
        return vn, ve
    scale = vmax / speed
    return vn*scale, ve*scale

def orca_2d(v1_des, v2_des, p1, p2, r_min, tau):
    """
    ORCA-lite para 2 agentes no plano.
    v1_des, v2_des: (vn, ve) desejados
    p1, p2: posições atuais no plano N-E (m)
    r_min: raio de segurança (m)
    tau: horizonte (s)
    Retorna v1_new, v2_new (ajustadas)
    """
    # velocidade relativa que levaria colisão se (p2 - p1) / tau estiver dentro do disco r_min/tau
    # Construímos o VO como um disco no espaço de velocidades relativas
    rel_p = (p2[0]-p1[0], p2[1]-p1[1])
    rel_v_des = (v2_des[0]-v1_des[0], v2_des[1]-v1_des[1])

    dist = vec_norm(*rel_p)
    if dist < 1e-6:
        # mesmos pontos: empurra para direções opostas mínimas
        u = (r_min/tau, 0.0)
    else:
        # Centro do VO (vel que levaria colisão no horizonte)
        c_vo = (rel_p[0]/tau, rel_p[1]/tau)
        # raio do VO em espaço de velocidades
        r_vo = r_min / tau
        # vetor do centro até a vel relativa desejada
        d = (rel_v_des[0] - c_vo[0], rel_v_des[1] - c_vo[1])
        d_mag = vec_norm(*d)
        if d_mag >= r_vo:
            # já está fora do VO => sem ajuste
            return v1_des, v2_des
        # Precisamos “empurrar” rel_v_des para a borda do disco
        if d_mag < 1e-6:
            # empurra numa direção perpendicular arbitrária
            u = (r_vo, 0.0)
        else:
            scale = (r_vo - d_mag) / d_mag
            u = (d[0]*scale, d[1]*scale)

    # Dividimos o ajuste igualmente e em sentido oposto (reciprocal)
    v1_new = (v1_des[0] - 0.5*u[0], v1_des[1] - 0.5*u[1])
    v2_new = (v2_des[0] + 0.5*u[0], v2_des[1] + 0.5*u[1])
    return v1_new, v2_new


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

async def start_offboard(drone: System, label: str):
    # precisa de um setpoint inicial
    try:
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await drone.offboard.start()
        print(f"[{label}] 🟢 Offboard ON")
    except OffboardError as e:
        print(f"[{label}] ❌ Falha ao iniciar offboard: {e._result.result}")
        raise

async def stop_offboard(drone: System, label: str):
    try:
        await drone.offboard.stop()
        print(f"[{label}] 🔴 Offboard OFF")
    except OffboardError as e:
        print(f"[{label}] ⚠️ Falha ao parar offboard: {e._result.result}")

async def set_velocity(drone: System, vn: float, ve: float, vd: float, yaw_deg: float = 0.0):
    # vd é positivo para baixo no frame NED; mantenha vd=0 para plano horizontal
    await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, yaw_deg))

async def arm_takeoff(drone: System, label: str, target_alt: float = ALTITUDE):
    print(f"[{label}] 🚁 Armando...")
    await drone.action.arm()
    print(f"[{label}] 🚀 Decolando...")
    await drone.action.takeoff()
    await asyncio.sleep(15)


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

def run_formation_with_orca(q1, r1, q2, r2, ref_pos, offsets,
                            vmax=1.0, dt=0.1, tau=3.0, r_min=1.5,
                            pos_tol=0.25, max_time=30.0):
    """
    Executa uma transição de formação com evitação de colisão em 2D (plano horizontal).
    - vmax: velocidade máxima (m/s)
    - dt: período de controle (s)
    - tau: horizonte de predição para ORCA (s)
    - r_min: distância mínima segura (m)
    - pos_tol: tolerância para considerar que chegou (m)
    - max_time: timeout por formação (s)
    """
    lat_ref, lon_ref, alt_ref = ref_pos

    # Objetivos locais (N,E) a partir do ref_pos
    n1_goal, e1_goal, alt = offsets[1]
    n2_goal, e2_goal, _   = offsets[2]  # mesma altitude “alt”

    # Ligar Offboard
    q1.put(("offboard_start", None))
    q2.put(("offboard_start", None))

    t0 = time.time()
    while True:
        # Posição atual dos dois
        q1.put(("stream_pos_once", None))
        q2.put(("stream_pos_once", None))
        p1 = p2 = None
        for _ in range(2):
            tag, pos = r1.get() if _ == 0 else r2.get()
            if tag != "pos":
                continue
            lat, lon, _alt = pos
            n, e = latlon_to_local_ned(lat_ref, lon_ref, lat, lon)
            if _ == 0:
                p1 = (n, e)
            else:
                p2 = (n, e)

        if p1 is None or p2 is None:
            continue

        # Erro até o alvo (N,E)
        e1 = (n1_goal - p1[0], e1_goal - p1[1])
        e2 = (n2_goal - p2[0], e2_goal - p2[1])

        # Chegou?
        if vec_norm(*e1) < pos_tol and vec_norm(*e2) < pos_tol:
            break

        # Velocidade desejada (proporcional simples)
        k = 0.9  # ganho
        v1_des = (k*e1[0], k*e1[1])
        v2_des = (k*e2[0], k*e2[1])
        v1_des = clamp_speed(*v1_des, vmax)
        v2_des = clamp_speed(*v2_des, vmax)

        # Ajuste ORCA-lite
        v1_cmd, v2_cmd = orca_2d(v1_des, v2_des, p1, p2, r_min=r_min, tau=tau)
        v1_cmd = clamp_speed(*v1_cmd, vmax)
        v2_cmd = clamp_speed(*v2_cmd, vmax)

        print(f"Ajustando velocidades:")
        print(f"Drone 1: {v1_cmd[0]} {v1_cmd[1]}")
        print(f"Drone 2: {v2_cmd[0]} {v2_cmd[1]}")

        # Comanda velocidades (somente horizontal; vd=0 para manter plano)
        q1.put(("vel", (v1_cmd[0], v1_cmd[1], 0.0, 0.0)))
        q2.put(("vel", (v2_cmd[0], v2_cmd[1], 0.0, 0.0)))

        # timing
        elapsed = time.time() - t0
        if elapsed > max_time:
            print("⏱️ Timeout desta formação (continuando com o que deu).")
            break
        time.sleep(dt)

    # Zera velocidades e mantém plano
    q1.put(("vel", (0.0, 0.0, 0.0, 0.0)))
    q2.put(("vel", (0.0, 0.0, 0.0, 0.0)))
    q1.put(("offboard_stop", None))
    q2.put(("offboard_stop", None))


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
            elif cmd[0] == "offboard_start":
                await start_offboard(drone, label)
            elif cmd[0] == "offboard_stop":
                await stop_offboard(drone, label)
            elif cmd[0] == "vel":
                vn, ve, vd, yaw = cmd[1]
                await set_velocity(drone, vn, ve, vd, yaw)
            elif cmd[0] == "stream_pos_once":
                pos = await get_position(drone)
                resp_queue.put(("pos", pos))
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
    time.sleep(5)
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
        offsets = parse_grid(grid)
        print("\n🚧 Formação com evitação (ORCA-lite)\n")
        run_formation_with_orca(
            q1, r1, q2, r2,
            ref_pos=ref_pos,
            offsets=offsets,
            vmax=0.4,       # velocidade máx (m/s)
            dt=0.08,        # 10 Hz
            tau=3.5,        # horizonte de predição
            r_min=1.0,      # separação mínima (m)
            pos_tol=0.25,   # tolerância de chegada (m)
            max_time=120.0  # timeout por formação (s)
        )


    print("\n🛬 Pousando ambos!\n")
    q1.put(("land", None))
    q2.put(("land", None))

    time.sleep(5)
    q1.put(("exit", None))
    q2.put(("exit", None))

    p1.join()
    p2.join()
    print("✅ Missão finalizada.")
