import multiprocessing as mp
import asyncio
import time
import math
from mavsdk import System

# -----------------------
# Configurações do Enxame
# -----------------------
NUM_DRONES = 5
ALTITUDE = 15.0

# Define a geometria da "Estrutura Virtual" (r_i^{VS})
# Formato de Seta/V. Tupla: (Offset_Norte, Offset_Leste) em metros
OFFSETS_VS = {
    0: (0.0, 0.0),      # Posição 0: Bico da Seta (Centro Virtual)
    1: (-5.0, 5.0),     # Posição 1: Asa Direita
    2: (-5.0, -5.0),    # Posição 2: Asa Esquerda
    3: (-10.0, 10.0),   # Posição 3: Asa Direita
    4: (-10.0, -10.0),  # Posição 4: Asa Esquerda
    5: (-15.0, 15.0),   # Posição 5: Asa Direita
    6: (-15.0, -15.0),  # Posição 6: Asa Esquerda
    7: (-20.0, 0.0)     # Posição 7: Cauda da Seta (Centro-atrás)
}

# -----------------------
# Funções Matemáticas (Equação 5 do TCC)
# -----------------------
def rotate_offset(n, e, yaw_deg):
    """
    Representa a multiplicação R_v(t) * r_i^{VS}.
    Rotaciona os offsets locais da estrutura baseada no 'yaw' (orientação) do centro virtual.
    """
    rad = math.radians(yaw_deg)
    n_rot = n * math.cos(rad) - e * math.sin(rad)
    e_rot = n * math.sin(rad) + e * math.cos(rad)
    return n_rot, e_rot

def add_offset_to_coords(lat, lon, offset_north, offset_east):
    """ Adiciona deslocamentos métricos (NED) a uma coordenada GPS (Lat/Lon) """
    earth_radius = 6378137.0
    dLat = offset_north / earth_radius
    dLon = offset_east / (earth_radius * math.cos(math.pi * lat / 180.0))
    
    new_lat = lat + (dLat * 180.0 / math.pi)
    new_lon = lon + (dLon * 180.0 / math.pi)
    return new_lat, new_lon

# -----------------------
# Funções do MAVSDK (Worker)
# -----------------------
async def connect_and_check(sys_addr, label, grpc_port):
    drone = System(port=grpc_port)
    print(f"[{label}] 🔌 Conectando em {sys_addr}...")
    await drone.connect(system_address=sys_addr)

    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    print(f"[{label}] ⏳ Verificando GPS...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print(f"[{label}] ✅ GPS OK.")
            break
    return drone

async def get_position(drone: System):
    async for pos in drone.telemetry.position():
        return pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m

async def arm_takeoff(drone, label, target_alt):
    print(f"[{label}] 🚁 Decolando...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(target_alt)
    await drone.action.takeoff()
    
async def goto_location_now(drone, lat, lon, alt, yaw):
    # Comando de posição absoluta com orientação
    await drone.action.goto_location(lat, lon, alt, yaw)

# -----------------------
# Worker do Drone (Processo Isolado)
# -----------------------
def drone_worker(drone_id, sys_addr, grpc_port, cmd_queue, resp_queue, ready_evt, start_evt):
    label = f"Drone {drone_id}"
    
    async def runner():
        try:
            drone = await connect_and_check(sys_addr, label, grpc_port)
            ready_evt.set()
        except Exception as e:
            print(f"[{label}] ❌ Erro: {e}")
            return

        while True:
            cmd = cmd_queue.get()
            try:
                if cmd[0] == "takeoff":
                    start_evt.wait()
                    await arm_takeoff(drone, label, cmd[1])
                    await asyncio.sleep(15) # Dá tempo para todos os 8 subirem
                
                elif cmd[0] == "getpos":
                    pos = await get_position(drone)
                    resp_queue.put(("pos_result", pos))
                
                elif cmd[0] == "goto":
                    lat, lon, alt, yaw = cmd[1]
                    await goto_location_now(drone, lat, lon, alt, yaw)
                
                elif cmd[0] == "land":
                    print(f"[{label}] 🛬 Pousando...")
                    await drone.action.land()
                
                elif cmd[0] == "exit":
                    break
            except Exception as e:
                print(f"[{label}] Erro no cmd {cmd[0]}: {e}")

    asyncio.run(runner())

# -----------------------
# MAIN - O "Centro da Estrutura Virtual"
# -----------------------
if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)
    
    queues_cmd = [mp.Queue() for _ in range(NUM_DRONES)]
    queues_resp = [mp.Queue() for _ in range(NUM_DRONES)]
    events_ready = [mp.Event() for _ in range(NUM_DRONES)]
    start_all = mp.Event()

    base_port = 14540
    grpc_base = 50050
    processes = []

    print("--- Inicializando Arquitetura Virtual Structure ---")
    
    for i in range(NUM_DRONES):
        sys_addr = f"udp://:{base_port + i}"
        p = mp.Process(
            target=drone_worker,
            args=(i, sys_addr, grpc_base + i, queues_cmd[i], queues_resp[i], events_ready[i], start_all)
        )
        p.start()
        processes.append(p)

    print("⌛ Aguardando as 8 conexões (Pode levar alguns segundos)...")
    for ev in events_ready:
        ev.wait()

    # --- Decolagem Conjunta ---
    print("\n🚀 Iniciando decolagem dos 8 drones...")
    for q in queues_cmd:
        q.put(("takeoff", ALTITUDE))
    
    time.sleep(2)
    start_all.set()
    
    print("⏳ Aguardando drones estabilizarem no ar (20s)...")
    time.sleep(20) 

    # --- Pega a posição de origem para o Centro Virtual (p_v) ---
    queues_cmd[0].put(("getpos", None))
    # Esvazia fila antes de ler
    while not queues_resp[0].empty(): queues_resp[0].get()
    
    origem_lat, origem_lon, origem_alt = queues_resp[0].get()[1]
    print(f"\n📍 Origem da Estrutura Virtual: Lat {origem_lat:.5f}, Lon {origem_lon:.5f}")

    # --- Definição da Trajetória do Centro Virtual ---
    # Formato: (Deslocamento Global Norte, Deslocamento Global Leste, Yaw da Estrutura)
    # A estrutura vai para a frente, depois vira 90 graus para a direita e segue,
    # demonstrando a rotação de corpo rígido.
    trajetoria_virtual = [
        (0, 0, 0),       # Passo 1: Apenas entra na formação Seta (Yaw 0 = Apontando pro Norte)
        (30, 0, 0),      # Passo 2: Move a estrutura inteira 30m para o Norte
        (30, 0, 90),     # Passo 3: Rotaciona a estrutura inteira 90 graus (Aponta Leste) no mesmo lugar
        (30, 30, 90),    # Passo 4: Move a estrutura inteira 30m para o Leste
        (0, 30, 180),    # Passo 5: Volta 30m para o Sul com a estrutura apontando pro Sul
        (0, 0, 0)        # Passo 6: Retorna à origem e aponta para o Norte
    ]

    print("\n🏁 INICIANDO MOVIMENTAÇÃO DA ESTRUTURA RÍGIDA\n")

    for passo, wp in enumerate(trajetoria_virtual):
        v_norte, v_leste, v_yaw = wp
        print(f"📐 Passo {passo+1}: Estrutura p_v=({v_norte}N, {v_leste}E), R_v(Yaw)={v_yaw}°")
        
        # O Centro Virtual (p_v) global atualizado
        pv_lat, pv_lon = add_offset_to_coords(origem_lat, origem_lon, v_norte, v_leste)

        # Para cada drone no enxame, calcular p_i^d(t)
        for i in range(NUM_DRONES):
            # 1. Pega a posição relativa fixa (r_i^{VS})
            offset_n, offset_e = OFFSETS_VS[i]
            
            # 2. Rotaciona o offset baseado no Yaw da estrutura (R_v * r_i)
            rot_n, rot_e = rotate_offset(offset_n, offset_e, v_yaw)
            
            # 3. Adiciona a posição do centro virtual (p_i^d = p_v + R_v * r_i)
            target_lat, target_lon = add_offset_to_coords(pv_lat, pv_lon, rot_n, rot_e)
            
            # Envia comando de ir para lá, apontando para o mesmo lado da estrutura
            queues_cmd[i].put(("goto", (target_lat, target_lon, ALTITUDE, v_yaw)))
            
        # Espera o enxame físico inteiro viajar/rotacionar e se acomodar na estrutura geométrica
        time.sleep(15)

    # --- Pouso ---
    print("\n✅ Missão Virtual Structure Concluída. Pousando enxame...")
    for q in queues_cmd:
        q.put(("land", None))

    time.sleep(15)
    for q in queues_cmd:
        q.put(("exit", None))
    
    for p in processes:
        p.terminate()
    print("Simulação encerrada.")