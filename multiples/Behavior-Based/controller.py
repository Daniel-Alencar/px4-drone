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

NUM_DRONES = 3
ALTITUDE = 12.0

# Novos Pesos: O Efeito Funil Resolvido
W_SEP = 15.0   # AUMENTADO: Respeito máximo pelo espaço do colega!
W_OBS = 15.0   # Mantido: Medo da parede igual ao medo de bater no amigo
W_COH = 0.1    # REDUZIDO: Relaxa a vontade de andar grudado na hora do aperto
W_ALI = 2.0    # AUMENTADO: Se um acelerar para desviar, o outro acompanha
W_GOAL = 5.0   # Mantido: Vontade de ir para a frente

MAX_STEP = 1.0        # Passos curtos para reflexos rápidos
RAIO_CILINDRO = 1.5   
RAIO_VISAO_OBS = 3.0

# Obstáculos Virtuais no plano NED (Norte, Leste)
OBSTACULOS_ESTATICOS = [
    np.array([20.0, 0.0]),
    np.array([20.0, 7.0]),
    np.array([20.0, -7.0]),
    np.array([24.0, 3.5]),
    np.array([24.0, -3.5]),
    np.array([24.0, 10.5]),
    np.array([24.0, -10.5]),
    np.array([28.0, 0.0]),
    np.array([28.0, 7.0]),
    np.array([28.0, -7.0])
]

# -----------------------
# Coordenadas Absolutas do Gazebo
# -----------------------
REF_LAT = 47.397971057728974
REF_LON = 8.5461637398001464
EARTH_RADIUS = 6371000.0 

def gps_to_ned(lat, lon):
    lat_rad, lon_rad = math.radians(lat), math.radians(lon)
    ref_lat_rad, ref_lon_rad = math.radians(REF_LAT), math.radians(REF_LON)
    n = (lat_rad - ref_lat_rad) * EARTH_RADIUS
    e = (lon_rad - ref_lon_rad) * EARTH_RADIUS * math.cos(ref_lat_rad)
    return np.array([n, e])

def ned_to_gps(n, e):
    ref_lat_rad = math.radians(REF_LAT)
    d_lat = n / EARTH_RADIUS
    d_lon = e / (EARTH_RADIUS * math.cos(ref_lat_rad))
    return REF_LAT + math.degrees(d_lat), REF_LON + math.degrees(d_lon)

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

    print("--- Inicializando Arquitetura Behavior-Based com Obstáculos ---")
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

    # Define o Objetivo Global (p_goal) para onde o bando deve migrar (50m ao Norte)
    P_GOAL = np.array([50.0, 0.0])

    print("\n🦅 INICIANDO COMPORTAMENTO DE BANDO")
    print(f"Objetivo: N={P_GOAL[0]}m. O enxame vai fluir pelos obstáculos.")

    # Loop de Controle
    chegaram = False
    
    while not chegaram:
        estados = {} 
        
        # 1. Coleta os estados de todos
        for i in range(NUM_DRONES):
            queues_cmd[i].put(("get_state", None))
            while queues_resp[i].qsize() > 1: queues_resp[i].get_nowait()
            
            lat, lon, _, vn, ve = queues_resp[i].get()[1]
            pos_ned = gps_to_ned(lat, lon)
            vel_ned = np.array([vn, ve])
            
            estados[i] = {"p": pos_ned, "v": vel_ned}

        # Verifica se o centro de massa do bando chegou ao objetivo
        centro_de_massa = np.mean([estados[i]["p"] for i in range(NUM_DRONES)], axis=0)
        if np.linalg.norm(P_GOAL - centro_de_massa) < 3.0:
            chegaram = True
            break

        # 2. Calcula as Forças (Equações 11 do TCC)
        novas_posicoes = {}
        
        for i in range(NUM_DRONES):
            p_i = estados[i]["p"]
            v_i = estados[i]["v"]
            
            f_sep = np.array([0.0, 0.0])
            f_obs = np.array([0.0, 0.0]) # Nova força exclusiva para obstáculos
            f_coh = np.array([0.0, 0.0])
            f_ali = np.array([0.0, 0.0])
            
            # --- SEPARAÇÃO DOS OBSTÁCULOS (Considerando a BORDA) ---
            for p_obs in OBSTACULOS_ESTATICOS:
                vetor_dist_obs = p_i - p_obs
                
                # Truque da colinearidade
                if abs(vetor_dist_obs[1]) < 0.2 and abs(vetor_dist_obs[0]) > 0:
                    vetor_dist_obs[1] += 1.0
                    
                dist_centro = np.linalg.norm(vetor_dist_obs)
                
                # Desconta o raio físico do cilindro para achar a distância até a PAREDE
                dist_borda = dist_centro - RAIO_CILINDRO 
                
                if dist_borda < 0.1:
                    dist_borda = 0.1 # Evita erro de divisão por zero se ele "ralar" na parede
                    
                if dist_centro < RAIO_VISAO_OBS:
                    # A força agora cresce ao infinito quando ele encosta na BORDA
                    # Usamos dist_borda ao cubo (**3) para a força explodir de forma muito abrupta 
                    # apenas quando ele chega muito perto, criando uma parede invisível dura.
                    f_obs += (vetor_dist_obs / (dist_borda**3))
                    
            # --- CÁLCULOS COM OS VIZINHOS (OUTROS DRONES) ---
            vizinhos = [j for j in range(NUM_DRONES) if j != i]
            if len(vizinhos) > 0:
                for j in vizinhos:
                    p_j = estados[j]["p"]
                    v_j = estados[j]["v"]
                    
                    vetor_dist = p_i - p_j
                    dist = np.linalg.norm(vetor_dist)
                    if dist < 0.1: dist = 0.1 
                    
                    f_sep += vetor_dist / (dist**2)
                    f_coh += (p_j - p_i)
                    f_ali += (v_j - v_i)
                
                # Coesão e Alinhamento são médias
                f_coh = f_coh / len(vizinhos)
                f_ali = f_ali / len(vizinhos)

            # --- BUSCA PELO OBJETIVO ---
            f_goal = P_GOAL - p_i
            norm_goal = np.linalg.norm(f_goal)
            if norm_goal > 0: f_goal = f_goal / norm_goal

            # 3. Força Resultante u_i (Agora com o W_OBS isolado)
            u_i = (W_SEP * f_sep) + (W_OBS * f_obs) + (W_COH * f_coh) + (W_ALI * f_ali) + (W_GOAL * f_goal)
            
            norm_u = np.linalg.norm(u_i)
            if norm_u > MAX_STEP:
                u_i = (u_i / norm_u) * MAX_STEP
                
            novas_posicoes[i] = p_i + u_i

        # 4. Enviar os Comandos de Movimento
        for i in range(NUM_DRONES):
            prox_ned = novas_posicoes[i]
            prox_lat, prox_lon = ned_to_gps(prox_ned[0], prox_ned[1])
            queues_cmd[i].put(("goto", (prox_lat, prox_lon, ALTITUDE)))
            
        time.sleep(0.5)

    print("\n✅ O Bando alcançou o objetivo de migração ileso!")
    print("🛬 Pousando...")
    for q in queues_cmd: q.put(("land", None))
    time.sleep(10)
    for q in queues_cmd: q.put(("exit", None))
    for p in processes: p.terminate()