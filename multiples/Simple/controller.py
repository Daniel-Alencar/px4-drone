import multiprocessing as mp
import asyncio
import time
from mavsdk import System

ALTITUDE = 5.0 # Aumentei um pouco para segurança

# Matriz simplificada para teste
grids = [
    [
        [0, 0, 1],
        [0, 0, 0],
        [2, 0, 0],
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
    # Define porta gRPC local única para não dar conflito entre processos
    drone = System(port=grpc_port)
    
    print(f"[{label}] 🔌 Tentando conectar em {system_address}...")
    await drone.connect(system_address=system_address)

    # Aguarda conexão
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[{label}] ✅ Conectado ao backend!")
            break

    print(f"[{label}] ⏳ Verificando Global Position (GPS)...")
    # Em SITL, as vezes health check falha falso-positivo, 
    # checar se tem posição é mais garantido para 'estar pronto'.
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(f"[{label}] ✅ GPS/Home OK. Pronto.")
            break
            
    return drone

async def arm_takeoff(drone: System, label: str, target_alt: float):
    print(f"[{label}] 🚁 Armando e Decolando para {target_alt}m...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(target_alt)
    await drone.action.takeoff()
    await asyncio.sleep(10) # Espera física para subir

async def get_position(drone: System):
    async for pos in drone.telemetry.position():
        return pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m

async def goto_relative(drone: System, label: str, ref_pos, offset_north, offset_east, offset_alt):
    # Conversão simples NED para LatLon
    dlat = offset_north / 111320.0
    # Correção da longitude baseada na latitude atual
    dlon = offset_east / (111320.0 * 0.99) # Aproximação para latitude ~0 (Equador/Petrolina?) ou usar math.cos
    
    target_lat = ref_pos[0] + dlat
    target_lon = ref_pos[1] + dlon
    target_alt = ref_pos[2] + offset_alt # Mantém altitude absoluta relativa à referência

    print(f"[{label}] ✈️ Indo: N={offset_north} E={offset_east}")
    await drone.action.goto_location(target_lat, target_lon, target_alt, 0)

async def land(drone: System, label: str):
    print(f"[{label}] 🛬 Pousando...")
    await drone.action.land()

# -----------------------
# Worker
# -----------------------
def drone_worker(system_address, label, grpc_port, cmd_queue, resp_queue, ready_evt, start_evt):
    async def runner():
        try:
            drone = await connect_and_check(system_address, label, grpc_port)
            ready_evt.set() # Avisa o main que este drone está pronto
        except Exception as e:
            print(f"[{label}] ❌ Erro na conexão: {e}")
            return

        while True:
            cmd = cmd_queue.get()
            try:
                if cmd[0] == "takeoff":
                    start_evt.wait() # Espera sinal de "todos prontos"
                    await arm_takeoff(drone, label, cmd[1])
                elif cmd[0] == "getpos":
                    pos = await get_position(drone)
                    resp_queue.put(("pos_result", pos))
                elif cmd[0] == "goto":
                    ref_pos, offset = cmd[1]
                    await goto_relative(drone, label, ref_pos, *offset)
                elif cmd[0] == "land":
                    await land(drone, label)
                elif cmd[0] == "exit":
                    break
            except Exception as e:
                print(f"[{label}] Erro ao executar {cmd[0]}: {e}")

    asyncio.run(runner())

# -----------------------
# Auxiliares
# -----------------------
def parse_grid(grid, cell_size=3):
    offsets = {}
    rows = len(grid)
    cols = len(grid[0])
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] != 0:
                d_id = grid[r][c]
                # No grid: linha avança para baixo (Norte negativo?), coluna para direita (Leste positivo)
                # Vamos assumir: r=0 é topo (Norte), r aumenta para Sul
                north = -r * cell_size 
                east = c * cell_size
                offsets[d_id] = (north, east, 0) # 0 de offset vertical relativo
    return offsets

# -----------------------
# Main
# -----------------------
if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)

    # CORREÇÃO 1: Número exato de drones rodando no simulador
    n_drones = 2 
    
    queues_cmd = [mp.Queue() for _ in range(n_drones)]
    queues_resp = [mp.Queue() for _ in range(n_drones)]
    events_ready = [mp.Event() for _ in range(n_drones)]
    start_all = mp.Event()

    base_system_port = 14540
    base_grpc_port = 50050

    processes = []
    print("--- Inicializando Processos Python ---")
    
    for i in range(n_drones):
        # CORREÇÃO 2: Cálculo da porta exato (sem o +1 extra no loop)
        # Bash i=0 -> 14540. Python i=0 -> 14540.
        sys_addr = f"udp://:{base_system_port + i}"
        
        # ID lógico para print (1 e 2)
        lbl = f"Drone {i+1}" 
        
        p = mp.Process(
            target=drone_worker,
            args=(sys_addr, lbl, base_grpc_port + i, queues_cmd[i], queues_resp[i], events_ready[i], start_all)
        )
        p.start()
        processes.append(p)

    print("⌛ Aguardando conexões (MAVSDK)...")
    # O loop principal vai travar aqui até que os 2 drones conectem.
    for ev in events_ready:
        ev.wait()
    print("✅ Todos drones conectados e prontos!")

    # --- Decolagem ---
    print("\n🚀 Preparando decolagem...")
    for q in queues_cmd:
        q.put(("takeoff", ALTITUDE))
    
    time.sleep(1)
    start_all.set() # Libera os workers para decolar
    
    # Dá tempo para eles subirem (o worker já tem sleep, mas o main precisa esperar a lógica fluir)
    time.sleep(12) 

    # --- Posição Referência ---
    # Vamos usar o Drone 1 (índice 0) como referência
    print("\n📍 Obtendo posição do Líder (Drone 1)...")
    queues_cmd[0].put(("getpos", None))
    
    ref_pos = None
    # Loop simples de espera com timeout para não travar
    attempts = 0
    while ref_pos is None and attempts < 20:
        if not queues_resp[0].empty():
            msg = queues_resp[0].get()
            if msg[0] == "pos_result":
                ref_pos = msg[1]
        else:
            time.sleep(0.5)
            attempts += 1
            
    if ref_pos:
        print(f"   Referência: Lat {ref_pos[0]:.6f}, Lon {ref_pos[1]:.6f}, Alt {ref_pos[2]:.2f}")
    else:
        print("❌ Falha ao obter posição referência.")
        # Lógica de abortar aqui se necessário

    # --- Execução da Formação ---
    if ref_pos:
        for idx, grid in enumerate(grids):
            print(f"\n--- Formação {idx+1} ---")
            offsets = parse_grid(grid, cell_size=4)
            
            # Mapeamento: ID 1 -> queues_cmd[0], ID 2 -> queues_cmd[1]
            # Envia comando se o ID existir no grid atual
            if 1 in offsets:
                queues_cmd[0].put(("goto", (ref_pos, offsets[1])))
            if 2 in offsets:
                queues_cmd[1].put(("goto", (ref_pos, offsets[2])))
            
            # Espera drones chegarem na posição
            time.sleep(8)

    # --- Pouso ---
    print("\n🛬 Pousando...")
    for q in queues_cmd:
        q.put(("land", None))

    time.sleep(10)
    
    print("Encerrando...")
    for q in queues_cmd:
        q.put(("exit", None))

    for p in processes:
        p.terminate() # Força bruta se join demorar
        
    print("Fim.")