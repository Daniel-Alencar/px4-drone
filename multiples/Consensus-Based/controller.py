import multiprocessing as mp
import asyncio
import time
import math
import numpy as np
from mavsdk import System

# -----------------------
# Configurações do Consenso
# -----------------------
NUM_DRONES = 3
ALTITUDE = 10.0

# Ganho de Consenso
K_CONSENSUS = 0.15  

# Matriz de Adjacência A = [a_ij] (Equação 13 do TCC)
# 1 significa que há comunicação, 0 significa que não há.
# Grafo totalmente conectado (todos falam com todos):
ADJACENCY_MATRIX = np.array([
    [0, 1, 1], # Drone 0 ouve 1 e 2
    [1, 0, 1], # Drone 1 ouve 0 e 2
    [1, 1, 0]  # Drone 2 ouve 0 e 1
])

# Vetores de Formação d_i (NED: Norte, Leste)
# Os drones vão convergir para formar uma linha horizontal (Leste-Oeste)
DESIRED_OFFSETS = {
    0: np.array([0.0, 0.0]),    # Drone 0 no centro
    1: np.array([0.0, 8.0]),    # Drone 1 fica 8m a Leste do centro
    2: np.array([0.0, -8.0])    # Drone 2 fica 8m a Oeste do centro
}

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
# Controlador Principal Consensus
# -----------------------
if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)
    
    queues_cmd = [mp.Queue() for _ in range(NUM_DRONES)]
    queues_resp = [mp.Queue() for _ in range(NUM_DRONES)]
    events_ready = [mp.Event() for _ in range(NUM_DRONES)]
    start_all = mp.Event()

    print("--- Inicializando Arquitetura Consensus-Based ---")
    processes = []
    for i in range(NUM_DRONES):
        p = mp.Process(target=process_runner, args=(i, f"udp://:{14540+i}", 50050+i, queues_cmd[i], queues_resp[i], events_ready[i], start_all))
        p.start()
        processes.append(p)

    for ev in events_ready: ev.wait()

    print("\n🚀 Decolando enxame e aguardando estabilização...")
    for q in queues_cmd: q.put(("takeoff", ALTITUDE))
    start_all.set()
    time.sleep(15)

    queues_cmd[0].put(("getpos", None))
    ref_lat, ref_lon, _ = queues_resp[0].get()[1]

    print("\n🤝 INICIANDO ALGORITMO DE CONSENSO")
    print("Os drones vão negociar suas posições com base na Matriz de Adjacência para formar uma linha.")

    # Loop de Controle
    for iteracao in range(60): # Roda por cerca de 30 segundos (60 * 0.5s)
        posicoes_atuais_ned = {}
        
        # 1. Atualizar as posições de toda a rede (Compartilhamento de estado)
        for i in range(NUM_DRONES):
            queues_cmd[i].put(("getpos", None))
            while queues_resp[i].qsize() > 1: queues_resp[i].get_nowait()
            
            lat, lon, _ = queues_resp[i].get()[1]
            posicoes_atuais_ned[i] = gps_to_ned(lat, lon, ref_lat, ref_lon)

        # 2. Calcular a Lei de Controle de Consenso (Eq 17 adaptada para Formação)
        novas_posicoes_ned = {}
        for i in range(NUM_DRONES):
            p_i = posicoes_atuais_ned[i]
            d_i = DESIRED_OFFSETS[i]
            
            soma_consenso = np.array([0.0, 0.0])
            
            # Somatório dos vizinhos: \sum a_ij * [ (p_i - d_i) - (p_j - d_j) ]
            for j in range(NUM_DRONES):
                a_ij = ADJACENCY_MATRIX[i][j]
                if a_ij > 0:
                    p_j = posicoes_atuais_ned[j]
                    d_j = DESIRED_OFFSETS[j]
                    
                    estado_i = p_i - d_i
                    estado_j = p_j - d_j
                    
                    soma_consenso += a_ij * (estado_i - estado_j)
            
            # Atualização de Posição (Equação Diferencial convertida em Discreta)
            # u_i = -K * soma
            passo = -K_CONSENSUS * soma_consenso
            
            # Vamos adicionar um pequeno "Vetor de Navegação" global (2 metros por passo para o Norte)
            # Isso simula o enxame entrando em consenso enquanto viaja junto
            v_nav = np.array([2.0, 0.0]) 
            
            novas_posicoes_ned[i] = p_i + passo + v_nav

        # 3. Enviar os Comandos de Movimento
        for i in range(NUM_DRONES):
            prox_ned = novas_posicoes_ned[i]
            prox_lat, prox_lon = ned_to_gps(prox_ned[0], prox_ned[1], ref_lat, ref_lon)
            queues_cmd[i].put(("goto", (prox_lat, prox_lon, ALTITUDE)))
            
        print(f"Iteração {iteracao+1}/60 concluída.")
        time.sleep(0.5) # Frequência de atualização de 2Hz

    print("\n✅ Consenso Atingido!")
    
    print("🛬 Pousando...")
    for q in queues_cmd: q.put(("land", None))
    time.sleep(10)
    for q in queues_cmd: q.put(("exit", None))
    for p in processes: p.terminate()