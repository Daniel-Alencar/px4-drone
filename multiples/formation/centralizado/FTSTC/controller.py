import multiprocessing as mp
import asyncio
import time
import math
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw

# -----------------------
# Parâmetros gerais
# -----------------------
ALTITUDE = 3.0
CONTROL_HZ = 50.0
DT = 1.0 / CONTROL_HZ
MAX_VEL = 2.0          # m/s (saturacao de velocidade)
MAX_ACC = 2.0          # m/s^2 (saturacao de aceleracao integrada)
POS_TOL = 0.20         # convergencia (m)
FTSTC_TIMEOUT = 25.0   # tempo maximo para convergencia por formacao (s)

# Matrizes de formação (como no seu codigo)
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

# -----------------------
# Utilitários
# -----------------------
def parse_grid(grid, cell_size=2, flight_alt=ALTITUDE):
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

def latlon_to_ned(ref_lat, ref_lon, lat, lon, alt, ref_alt):
    """
    Aproximação simples lat/lon -> NED (metros) relativo a ref.
    Retorna (north, east, up) com up positivo (altitude acima referencia).
    """
    # 1 deg lat ~= 111320 m, mais preciso usando raio da Terra:
    R = 6378137.0
    dlat = math.radians(lat - ref_lat)
    dlon = math.radians(lon - ref_lon)
    north = dlat * R
    east = dlon * R * math.cos(math.radians(ref_lat))
    up = alt - ref_alt
    return (north, east, up)

def ned_to_latlon(ref_lat, ref_lon, ref_alt, north, east, up):
    R = 6378137.0
    dlat = north / R
    lat = ref_lat + math.degrees(dlat)
    meters_per_lon_degree = R * math.cos(math.radians(ref_lat))
    dlon = east / (meters_per_lon_degree if abs(meters_per_lon_degree) > 1e-9 else R)
    lon = ref_lon + math.degrees(dlon)
    alt = ref_alt + up
    return lat, lon, alt

# -----------------------
# FTSTC - função de controle (por eixo)
# -----------------------
def ftstc_acceleration(e, v, params):
    """
    e: tuple (ex, ey, ez) erro de posicao (m) -> p_i - p_L - delta_i (no mesmo frame)
    v: tuple (vx, vy, vz) velocidade atual [m/s] (se não disponível, passe (0,0,0))
    params: dict com k1,k2,alpha,beta,kc (kc para acoplamento se quiser)
    Retorna: aceleração desejada (ax, ay, az)
    """
    k1 = params.get("k1", 1.0)
    k2 = params.get("k2", 0.6)
    alpha = params.get("alpha", 0.5)
    beta = params.get("beta", 1.5)
    # term de acoplamento não é calculado individualmente aqui:
    # acoplamento será tratado pela troca de offsets via processo central (ou pode ser extendido)
    ax, ay, az = 0.0, 0.0, 0.0
    for i, (ei, vi) in enumerate(zip(e, v)):
        term1 = -k1 * math.copysign(abs(ei) ** alpha, ei) if abs(ei) > 1e-9 else 0.0
        term2 = -k2 * math.copysign(abs(vi) ** beta, vi) if abs(vi) > 1e-9 else 0.0
        val = term1 + term2
        if i == 0:
            ax = val
        elif i == 1:
            ay = val
        else:
            az = val
    return (ax, ay, az)

# -----------------------
# Worker do drone (rodando em processo separado)
# -----------------------
def drone_worker(system_address: str, label: str, grpc_port: int,
                 cmd_queue: mp.Queue, resp_queue: mp.Queue,
                 ready_evt: mp.Event, start_evt: mp.Event):
    """
    worker: conecta via MAVSDK, responde a comandos:
      - ("takeoff", alt)
      - ("getpos", None) -> devolve ("pos_result", (lat,lon,alt))
      - ("ftstc_start", (ref_pos, desired_delta, params, timeout)) -> inicia offboard FTSTC loop
      - ("ftstc_stop", None) -> interrompe loop FTSTC
      - ("land", None)
      - ("exit", None)
    """
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
        await asyncio.sleep(6)

    async def get_position_async(drone: System):
        async for pos in drone.telemetry.position():
            # retorna (lat, lon, absolute_altitude_m)
            return pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m

    async def get_velocity_ned(drone: System):
        """Tenta pegar velocidade NED (m/s) - retorna (vn, ve, vd) with vd positive down."""
        try:
            async for v in drone.telemetry.velocity_ned():
                # velocity_ned returns north_m_s, east_m_s, down_m_s
                return v.north_m_s, v.east_m_s, -v.down_m_s  # convert to up positive
        except Exception:
            return (0.0, 0.0, 0.0)

    async def start_offboard_with_initial(drone: System):
        # envia alguns setpoints iniciais antes de start (prática comum)
        for _ in range(5):
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.02)
        await drone.offboard.start()
        print(f"[{label}] 🟢 Offboard iniciado.")

    async def stop_offboard(drone: System):
        try:
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.05)
            await drone.offboard.stop()
            print(f"[{label}] 🔴 Offboard parado.")
        except Exception:
            # alguns firmwares podem não suportar stop; ao menos enviar zeros garante segurança
            pass

    async def ftstc_loop(drone: System, ref_pos, desired_delta, params, timeout, stop_flag):
        """
        Loop FTSTC que envia velocities em OFFBOARD.
        ref_pos: (lat, lon, alt) referencia para conversoes NED
        desired_delta: (north, east, alt) onde deseja estar relativo a ref_pos
        params: parametros do ftstc
        timeout: tempo maximo em s
        stop_flag: mp.Event alternativa (se setado interrompe)
        """
        # inicializa velocidade estimada local (integracao simples)
        vel = [0.0, 0.0, 0.0]  # north, east, up

        # garante offboard iniciado
        await start_offboard_with_initial(drone)

        t_start = time.time()
        converged = False

        while True:
            # checa stop por fila sem bloquear muito:
            try:
                # tenta pegar comando sem bloquear: se houver comando 'ftstc_stop' devolve True
                if not cmd_queue.empty():
                    peek = cmd_queue.get_nowait()
                    # se for um comando de controle, devolve para ser tratado externamente
                    if peek[0] == "ftstc_stop":
                        print(f"[{label}] ✋ Recebeu ftstc_stop.")
                        # termine FTSTC
                        break
                    else:
                        # se não for stop, re-enfileira para processamento na iteração principal
                        cmd_queue.put(peek)
            except Exception:
                pass

            # leitura posição e velocidade
            try:
                pos = await get_position_async(drone)
            except Exception:
                print(f"[{label}] ⚠️ Falha ao ler posicao, repetindo...")
                await asyncio.sleep(DT)
                continue

            vx_tele, vy_tele, vz_tele = 0.0, 0.0, 0.0
            # tenta pegar velocity_ned (se disponível)
            try:
                async for v in drone.telemetry.velocity_ned():
                    vx_tele, vy_tele, vd = v.north_m_s, v.east_m_s, v.down_m_s
                    vz_tele = -vd  # up positive
                    break
            except Exception:
                # não obteve velocidade, manter 0
                vx_tele, vy_tele, vz_tele = 0.0, 0.0, 0.0

            # converte pos -> NED relativo a ref_pos
            p_ned = latlon_to_ned(ref_pos[0], ref_pos[1], pos[0], pos[1], pos[2], ref_pos[2])
            # p_ned = (north, east, up)
            # erro e = p_i - p_L - delta_i
            # assumimos líder = drone 1 no sistema central (mas aqui o desired_delta já é relativo ao líder)
            e = (p_ned[0] - desired_delta[0],
                 p_ned[1] - desired_delta[1],
                 p_ned[2] - (desired_delta[2] - ALTITUDE))

            # velocidade atual (use telemetria se disponível)
            v_curr = (vx_tele, vy_tele, vz_tele)

            # aceleração desejada pelo FTSTC
            ax, ay, az = ftstc_acceleration(e, v_curr, params)

            # saturar aceleracao
            a_mag = math.sqrt(ax*ax + ay*ay + az*az)
            if a_mag > MAX_ACC:
                s = MAX_ACC / a_mag
                ax *= s; ay *= s; az *= s

            # integrar aceleracao para velocidade (Euler simples)
            vel[0] += ax * DT
            vel[1] += ay * DT
            vel[2] += az * DT

            # saturar velocidade
            vmag = math.sqrt(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2])
            if vmag > MAX_VEL:
                s = MAX_VEL / vmag
                vel[0] *= s; vel[1] *= s; vel[2] *= s

            # enviar comando de velocidade (down_m_s é negativo de up)
            try:
                await drone.offboard.set_velocity_ned(
                    VelocityNedYaw(vel[0], vel[1], -vel[2], 0.0)
                )
            except Exception as ex:
                print(f"[{label}] ⚠️ Erro ao set_velocity_ned: {ex}")

            # verifica convergencia
            err_norm = math.sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2])
            if err_norm < POS_TOL:
                converged = True
                print(f"[{label}] ✔️ FTSTC convergiu (err {err_norm:.3f} m).")
                # zera velocidade e sai
                try:
                    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                except Exception:
                    pass
                break

            # timeout / stop events
            if (time.time() - t_start) > timeout:
                print(f"[{label}] ⏱ FTSTC timeout ({timeout}s).")
                break
            if stop_flag.is_set():
                print(f"[{label}] ✋ stop_flag set, saindo do FTSTC.")
                break

            await asyncio.sleep(DT)

        # garante parada de velocidade
        try:
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.05)
        except Exception:
            pass

        # opcional: podemos parar offboard (ou manter)
        # await stop_offboard(drone)
        return converged

    # -----------------------
    # Runner principal do worker
    # -----------------------
    async def runner():
        drone = await connect_and_check(system_address, label, grpc_port)
        ready_evt.set()

        # stop flag (MP event não é awaitable; usamos como sinal)
        stop_flag = mp.Event()

        while True:
            cmd = cmd_queue.get()  # bloqueante - aguardando ordens do processo principal
            if cmd[0] == "takeoff":
                start_evt.wait()
                await arm_takeoff(drone, label, target_alt=cmd[1])
            elif cmd[0] == "getpos":
                pos = await get_position_async(drone)
                resp_queue.put(("pos_result", pos))
            elif cmd[0] == "ftstc_start":
                # payload: (ref_pos, desired_delta, params, timeout)
                ref_pos, desired_delta, params, timeout = cmd[1]
                print(f"[{label}] ▶️ Iniciando FTSTC -> desired_delta={desired_delta}, timeout={timeout}")
                converged = await ftstc_loop(drone, ref_pos, desired_delta, params, timeout, stop_flag)
                resp_queue.put(("ftstc_done", (label, converged)))
            elif cmd[0] == "ftstc_stop":
                print(f"[{label}] ✋ Comando ftstc_stop recebido no runner.")
                stop_flag.set()
            elif cmd[0] == "land":
                print(f"[{label}] 🛬 Pousando...")
                await drone.action.land()
                async for in_air in drone.telemetry.in_air():
                    if not in_air:
                        print(f"[{label}] ✅ Pousou com sucesso.")
                        break
            elif cmd[0] == "exit":
                print(f"[{label}] Encerrando processo.")
                break
            else:
                print(f"[{label}] ⚠️ Comando desconhecido: {cmd[0]}")

    # roda o loop asyncio do worker
    asyncio.run(runner())


# -----------------------
# Processo principal (orquestrador)
# -----------------------
if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)

    # filas
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

    # --- Define posição de referência (pegar do drone 1) ---
    print("\n📍 Pegando posição de referência do Drone 1\n")
    q1.put(("getpos", None))
    ref_pos = None
    while ref_pos is None:
        msg = r1.get()
        if msg[0] == "pos_result":
            ref_pos = msg[1]
    print(f"Referência obtida: {ref_pos}")

    # parâmetros FTSTC (padrão, pode afinar)
    ftstc_params = {
        "k1": 1.2,
        "k2": 0.9,
        "alpha": 0.6,
        "beta": 1.2
    }

    try:
        for grid in grids:
            offsets = parse_grid(grid)
            desired1 = offsets.get(1, (0.0, 0.0, ALTITUDE))
            desired2 = offsets.get(2, (0.0, 0.0, ALTITUDE))

            print("\n➡️ Iniciando formação (FTSTC) para grid:")
            print(grid)
            # envia comando para cada drone iniciar FTSTC
            q1.put(("ftstc_start", (ref_pos, desired1, ftstc_params, FTSTC_TIMEOUT)))
            q2.put(("ftstc_start", (ref_pos, desired2, ftstc_params, FTSTC_TIMEOUT)))

            # espera respostas dos dois drones (ou timeout global)
            done1 = False
            done2 = False
            start_wait = time.time()
            while not (done1 and done2) and (time.time() - start_wait) < (FTSTC_TIMEOUT + 5.0):
                # lê filas sem bloquear demais
                if not r1.empty():
                    m = r1.get()
                    if m[0] == "ftstc_done":
                        _, payload = m
                        label, conv = payload
                        print(f"[Main] Recebido ftstc_done de {label}, convergiu={conv}")
                        done1 = True
                if not r2.empty():
                    m = r2.get()
                    if m[0] == "ftstc_done":
                        _, payload = m
                        label, conv = payload
                        print(f"[Main] Recebido ftstc_done de {label}, convergiu={conv}")
                        done2 = True
                time.sleep(0.05)

            if not (done1 and done2):
                print("⚠️ Nem todos os drones confirmaram convergência dentro do tempo. Enviando ftstc_stop e prosseguindo.")
                q1.put(("ftstc_stop", None))
                q2.put(("ftstc_stop", None))
                # consumir possíveis mensagens pendentes
                time.sleep(0.5)

            # pequena pausa antes da próxima formação
            time.sleep(1.0)

        # pouso final
        print("\n🛬 Pousando ambos!\n")
        q1.put(("land", None))
        q2.put(("land", None))

        time.sleep(5)
    finally:
        q1.put(("exit", None))
        q2.put(("exit", None))
        p1.join()
        p2.join()
        print("✅ Missão finalizada.")
