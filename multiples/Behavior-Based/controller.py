import multiprocessing as mp
import asyncio
import time
import math
import numpy as np
from mavsdk import System

# -----------------------
# Configurações do Behavior-Based
# -----------------------
NUM_DRONES = 3
ALTITUDE = 12.0

# Pesos dos Comportamentos (Equação 10 do TCC)
# Ajustar esses valores muda completamente a "personalidade" do enxame
W_SEP = 8.0   # w_s: Peso da Separação (Evita colisões)
W_COH = 0.5   # w_c: Peso da Coesão (Mantém o grupo unido)
W_ALI = 1.0   # w_a: Peso do Alinhamento (Sincroniza velocidades)
W_GOAL = 1.5  # w_g: Peso da Busca por Objetivo (Atração ao alvo)

MAX_STEP = 3.5  # Limita o tamanho do "passo" a cada iteração (metros)

# -----------------------
# Funções Matemáticas / GPS
# -----------------------
EARTH_RADIUS = 6378137.0

def gps_to_ned(lat, lon, ref_lat, ref_lon):
    lat_rad, lon_rad = math.radians(lat), math.radians(lon)
    ref_lat_rad, ref_lon_rad = math.radians(ref_lat), math.radians(ref_lon)
    n = (lat_rad - ref_lat_rad) * EARTH_RADIUS
    e = (lon_rad - ref_lon_rad) * EARTH_RADIUS * math.cos(ref_lat_rad)
    return np.array([n, e])

def ned_to_gps(n, e, ref_lat, ref_lon):
    ref_lat_rad = math.radians(ref_lat)
    d_lat = n / EARTH_RADIUS
    d_lon = e / (EARTH_RADIUS * math.cos(ref_lat_rad))
    return ref_lat + math.degrees(d_lat), ref_lon + math.degrees(d_lon)

# -----------------------
# Funções do MAVSDK (Worker)
# -----------------------
async def worker_setup(sys_addr, label, grpc_port):
    drone = System(port=grpc_port)
    await drone.connect(system_address=sys_addr)
    async for state in drone.core.connection_state():
        if state.is_connected: break
    async for health in drone.telemetry.health():
        if health.is_global_position_ok: break
    return drone

async def worker_loop(drone_id, sys_addr, grpc_port, cmd_queue, resp_queue, ready_evt, start_evt):
    label = f"Drone {drone_id}"
    drone = await worker_setup(sys_addr, label, grpc_port)
    ready_evt.set()

    while True:
        cmd = cmd_queue.get()
        if cmd[0] == "takeoff":
            start_evt.wait()
            await drone.action.arm()
            await drone.action.set_takeoff_altitude(cmd[1])
            await drone.action.takeoff()
            
        elif cmd[0] == "get_state":
            # Para o Behavior-Based, precisamos da posição (para Coesão/Separação) 
            # e da velocidade (para o Alinhamento)
            lat, lon, alt = 0, 0, 0
            vn, ve = 0, 0
            
            async for pos in drone.telemetry.position():
                lat, lon, alt = pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m
                break
                
            async for vel in drone.telemetry.velocity_ned():
                vn, ve = vel.north_m_s, vel.east_m_s
                break
                
            resp_queue.put(("state_result", (lat, lon, alt, vn, ve)))
            
        elif cmd[0] == "goto":
            lat, lon, alt = cmd[1]
            await drone.action.goto_location(lat, lon, alt, 0)
            
        elif cmd[0] == "land":
            await drone.action.land()
            
        elif cmd[0] == "exit":
            break

def process_runner(*args):
    asyncio.run(worker_loop(*args))

# -----------------------
# Controlador Principal - Behavior-Based
# -----------------------
if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)
    
    queues_cmd = [mp.Queue() for _ in range(NUM_DRONES)]
    queues_resp = [mp.Queue() for _ in range(NUM_DRONES)]
    events_ready = [mp.Event() for _ in range(NUM_DRONES)]
    start_all = mp.Event()

    print("--- Inicializando Arquitetura Behavior-Based (Bando de Aves) ---")
    processes = []
    for i in range(NUM_DRONES):
        p = mp.Process(target=process_runner, args=(i, f"udp://:{14540+i}", 50050+i, queues_cmd[i], queues_resp[i], events_ready[i], start_all))
        p.start()
        processes.append(p)

    for ev in events_ready: ev.wait()

    print("\n🚀 Decolando enxame...")
    for q in queues_cmd: q.put(("takeoff", ALTITUDE))
    start_all.set()
    time.sleep(15)

    # Pegar Referência GPS
    queues_cmd[0].put(("get_state", None))
    ref_lat, ref_lon, _, _, _ = queues_resp[0].get()[1]

    # Define o Objetivo Global (p_goal) para onde o bando deve migrar (Ex: 50m ao Norte)
    P_GOAL = np.array([50.0, 0.0])

    print("\n🦅 INICIANDO COMPORTAMENTO DE BANDO")
    print(f"Objetivo: Voar para N={P_GOAL[0]}m. O enxame se auto-organizará no caminho.")

    # Loop de Controle
    chegaram = False
    
    while not chegaram:
        estados = {} # Guarda (Posição_NED, Velocidade_NED) de cada drone
        
        # 1. Coleta os estados de todos
        for i in range(NUM_DRONES):
            queues_cmd[i].put(("get_state", None))
            while queues_resp[i].qsize() > 1: queues_resp[i].get_nowait()
            
            lat, lon, _, vn, ve = queues_resp[i].get()[1]
            pos_ned = gps_to_ned(lat, lon, ref_lat, ref_lon)
            vel_ned = np.array([vn, ve])
            
            estados[i] = {"p": pos_ned, "v": vel_ned}

        # Verifica se o centro do bando chegou ao objetivo
        centro_de_massa = np.mean([estados[i]["p"] for i in range(NUM_DRONES)], axis=0)
        if np.linalg.norm(P_GOAL - centro_de_massa) < 3.0:
            chegaram = True
            break

        # 2. Calcula as 4 Forças (Equações 11 do TCC)
        novas_posicoes = {}
        
        for i in range(NUM_DRONES):
            p_i = estados[i]["p"]
            v_i = estados[i]["v"]
            
            f_sep = np.array([0.0, 0.0])
            f_coh = np.array([0.0, 0.0])
            f_ali = np.array([0.0, 0.0])
            
            vizinhos = [j for j in range(NUM_DRONES) if j != i]
            
            if len(vizinhos) > 0:
                for j in vizinhos:
                    p_j = estados[j]["p"]
                    v_j = estados[j]["v"]
                    
                    vetor_dist = p_i - p_j
                    dist = np.linalg.norm(vetor_dist)
                    
                    # Evita divisão por zero
                    if dist < 0.1: dist = 0.1 
                    
                    # Separação: (p_i - p_j) / ||p_i - p_j||^2
                    f_sep += vetor_dist / (dist**2)
                    
                    # Coesão (soma): (p_j - p_i)
                    f_coh += (p_j - p_i)
                    
                    # Alinhamento (soma): (v_j - v_i)
                    f_ali += (v_j - v_i)
                
                # Coesão e Alinhamento são médias (dividir por |N_i|)
                f_coh = f_coh / len(vizinhos)
                f_ali = f_ali / len(vizinhos)

            # Busca por objetivo: (p_goal - p_i)
            f_goal = P_GOAL - p_i
            # Normaliza f_goal para que a atração não seja esmagadora de longe
            norm_goal = np.linalg.norm(f_goal)
            if norm_goal > 0: f_goal = f_goal / norm_goal

            # 3. Força Resultante u_i (Equação 10 do TCC)
            u_i = (W_SEP * f_sep) + (W_COH * f_coh) + (W_ALI * f_ali) + (W_GOAL * f_goal)
            
            # Limita a magnitude do vetor resultante para movimentos seguros
            norm_u = np.linalg.norm(u_i)
            if norm_u > MAX_STEP:
                u_i = (u_i / norm_u) * MAX_STEP
                
            # A nova posição desejada é a posição atual + o vetor de comportamento
            novas_posicoes[i] = p_i + u_i

        # 4. Enviar os Comandos de Movimento
        for i in range(NUM_DRONES):
            prox_ned = novas_posicoes[i]
            prox_lat, prox_lon = ned_to_gps(prox_ned[0], prox_ned[1], ref_lat, ref_lon)
            queues_cmd[i].put(("goto", (prox_lat, prox_lon, ALTITUDE)))
            
        time.sleep(0.5)

    print("\n✅ O Bando alcançou o objetivo de migração!")
    print("🛬 Pousando...")
    for q in queues_cmd: q.put(("land", None))
    time.sleep(10)
    for q in queues_cmd: q.put(("exit", None))
    for p in processes: p.terminate()