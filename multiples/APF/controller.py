import multiprocessing as mp
import asyncio
import time
import math
import numpy as np
from mavsdk import System

# -----------------------
# Configurações do APF
# -----------------------
# Mude este número para a mesma quantidade do seu script Bash!
NUM_DRONES = 10
ALTITUDE = 10.0

# Ganhos do Campo de Potencial Balanceados
K_ATT = 5.0      # Força constante para o objetivo
K_REP = 500.0    # Força de repulsão (obstáculos)
D_OBS = 3.0      # Raio de influência dos obstáculos (metros)
MAX_STEP = 1.0   # Passo máximo (suaviza o movimento)

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
# Coordenadas Absolutas do Gazebo (Origem X=0, Y=0)
# -----------------------
REF_LAT = 47.397971057728974
REF_LON = 8.5461637398001464
EARTH_RADIUS = 6371000.0 

def gps_to_ned(lat, lon):
    """ Converte Lat/Lon do Drone para Metros (N, E) a partir do centro do Gazebo """
    lat_rad, lon_rad = math.radians(lat), math.radians(lon)
    ref_lat_rad, ref_lon_rad = math.radians(REF_LAT), math.radians(REF_LON)
    
    n = (lat_rad - ref_lat_rad) * EARTH_RADIUS
    e = (lon_rad - ref_lon_rad) * EARTH_RADIUS * math.cos(ref_lat_rad)
    return np.array([n, e])

def ned_to_gps(n, e):
    """ Converte Metros (N, E) para Lat/Lon exato para o MAVSDK """
    ref_lat_rad = math.radians(REF_LAT)
    
    d_lat = n / EARTH_RADIUS
    d_lon = e / (EARTH_RADIUS * math.cos(ref_lat_rad))
    
    new_lat = REF_LAT + math.degrees(d_lat)
    new_lon = REF_LON + math.degrees(d_lon)
    return new_lat, new_lon

def calcular_forcas_apf(pos_atual, pos_objetivo, outras_posicoes, obstaculos):
    # 1. Força de Atração Constante (Potencial Cônico)
    vetor_atracao = pos_objetivo - pos_atual
    distancia_alvo = np.linalg.norm(vetor_atracao)
    
    if distancia_alvo > 0.1:
        f_att = K_ATT * (vetor_atracao / distancia_alvo)
    else:
        f_att = np.array([0.0, 0.0])
        
    f_rep = np.array([0.0, 0.0])
    
    # 2. Repulsão dos OBSTÁCULOS (Bolha Grande de Proteção: D_OBS)
    for p_obs in obstaculos:
        vetor_distancia = pos_atual - p_obs
        
        # Truque da Colinearidade
        if abs(vetor_distancia[1]) < 0.2 and abs(vetor_distancia[0]) > 0:
            vetor_distancia[1] += 0.5
            
        distancia = np.linalg.norm(vetor_distancia)
        
        if 0.1 < distancia < D_OBS: 
            termo1 = (1.0 / distancia) - (1.0 / D_OBS)
            termo2 = 1.0 / (distancia**3)
            f_rep += K_REP * termo1 * termo2 * vetor_distancia

    # 3. Repulsão dos OUTROS DRONES 
    # Bolha reduzida para 1.8m para permitir que eles voem a 2m de distância sem conflito
    D_DRONE = 1.8 
    for p_drone in outras_posicoes:
        vetor_distancia = pos_atual - p_drone
        distancia = np.linalg.norm(vetor_distancia)
        
        if 0.1 < distancia < D_DRONE: 
            termo1 = (1.0 / distancia) - (1.0 / D_DRONE)
            termo2 = 1.0 / (distancia**3)
            f_rep += (K_REP * 0.5) * termo1 * termo2 * vetor_distancia

    # 4. Força Resultante
    f_total = f_att + f_rep
    
    # Limita o passo máximo
    norma_f = np.linalg.norm(f_total)
    if norma_f > MAX_STEP:
        f_total = (f_total / norma_f) * MAX_STEP
        
    return f_total

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
        elif cmd[0] == "getpos":
            async for pos in drone.telemetry.position():
                resp_queue.put(("pos_result", (pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m)))
                break
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
# Controlador Principal APF
# -----------------------
if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)
    
    queues_cmd = [mp.Queue() for _ in range(NUM_DRONES)]
    queues_resp = [mp.Queue() for _ in range(NUM_DRONES)]
    events_ready = [mp.Event() for _ in range(NUM_DRONES)]
    start_all = mp.Event()

    print(f"--- Inicializando Arquitetura APF para {NUM_DRONES} Drones ---")
    processes = []
    for i in range(NUM_DRONES):
        p = mp.Process(target=process_runner, args=(i, f"udp://:{14540+i}", 50050+i, queues_cmd[i], queues_resp[i], events_ready[i], start_all))
        p.start()
        processes.append(p)

    for ev in events_ready: ev.wait()

    print("\n🚀 Decolando enxame...")
    for q in queues_cmd: q.put(("takeoff", ALTITUDE))
    start_all.set()
    
    # Dá um pouco mais de tempo para garantir que todos subam
    time.sleep(15) 

    # --- Definição Dinâmica de Objetivos ---
    # Todos vão 40 metros para frente (Norte), mantendo o espaçamento lateral (Leste) original do bash
    objetivos_ned = {}
    for i in range(NUM_DRONES):
        objetivos_ned[i] = np.array([40.0, float(i * 2)]) 

    print("\n🏁 INICIANDO NAVEGAÇÃO POR CAMPOS POTENCIAIS (APF)")
    print(f"Alvos definidos para {NUM_DRONES} drones a 40m de distância.")

    # Loop de Controle
    chegaram = [False] * NUM_DRONES
    
    while not all(chegaram):
        posicoes_atuais_ned = {}
        
        # 1. Obter a posição GPS atual de todos e converter para NED
        for i in range(NUM_DRONES):
            queues_cmd[i].put(("getpos", None))
            while queues_resp[i].qsize() > 1: queues_resp[i].get_nowait()
            
            lat, lon, _ = queues_resp[i].get()[1]
            posicoes_atuais_ned[i] = gps_to_ned(lat, lon)
            
            # Margem de erro para chegar ao alvo
            dist_ao_alvo = np.linalg.norm(objetivos_ned[i] - posicoes_atuais_ned[i])
            if dist_ao_alvo < 1.5:
                chegaram[i] = True

        # 2. Calcular Campos de Potencial
        for i in range(NUM_DRONES):
            if chegaram[i]:
                continue 
                
            p_i = posicoes_atuais_ned[i]
            p_goal = objetivos_ned[i]
            
            # Extrai a posição de todos os OUTROS drones para calcular a repulsão
            outros_drones = [posicoes_atuais_ned[j] for j in range(NUM_DRONES) if j != i]
            
            forca_resultante = calcular_forcas_apf(p_i, p_goal, outros_drones, OBSTACULOS_ESTATICOS)
            
            proxima_pos_ned = p_i + forca_resultante
            prox_lat, prox_lon = ned_to_gps(proxima_pos_ned[0], proxima_pos_ned[1])
            
            queues_cmd[i].put(("goto", (prox_lat, prox_lon, ALTITUDE)))
            
            # Print simplificado para não poluir muito a tela com muitos drones
            print(f"D{i} -> N:{p_i[0]:.1f} | E:{p_i[1]:.1f}")

        print("-" * 30)
        time.sleep(0.5)

    print("\n✅ Todos os drones alcançaram seus objetivos desviando dos obstáculos!")
    
    print("🛬 Pousando...")
    for q in queues_cmd: q.put(("land", None))
    time.sleep(10)
    for q in queues_cmd: q.put(("exit", None))
    for p in processes: p.terminate()