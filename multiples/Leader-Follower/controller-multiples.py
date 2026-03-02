import multiprocessing as mp
import asyncio
import time
import math
import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, PositionNedYaw

# -----------------------
# Parâmetros
# -----------------------
ALTITUDE = 10.0
KP = 1.0  # Ganho Proporcional
NUM_DRONES = 5

# -----------------------
# Definição da Formação (Geometria em V)
# -----------------------
# O ID 1 é o Líder. Os outros são seguidores.
# Referencial NED: [Norte, Leste, Baixo]
OFFSETS = {
    1: np.array([0.0, 0.0, 0.0]),    # Líder (Ponta do V)
    2: np.array([-3.0, -3.0, 0.0]),  # Seguidor 1 (Atrás Esquerda)
    3: np.array([-3.0, 3.0, 0.0]),   # Seguidor 2 (Atrás Direita)
    4: np.array([-6.0, -6.0, 0.0]),  # Seguidor 3 (Mais atrás Esquerda)
    5: np.array([-6.0, 6.0, 0.0])    # Seguidor 4 (Mais atrás Direita)
}

# -----------------------
# Lógica de Controle
# -----------------------
async def run_leader_logic(drone, shared_telemetry):
    print("[Líder] 🔄 Iniciando trajetória circular...")
    t = 0
    dt = 0.1
    radius = 15.0 # Aumentei o raio para caber 5 drones virando
    omega = 0.2

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"[Líder] ❌ Falha no Offboard: {error}")
        return

    while True:
        # Cinemática
        vn = radius * omega * math.cos(omega * t)
        ve = radius * omega * math.sin(omega * t)
        yaw = math.degrees(math.atan2(ve, vn))

        # Comando
        await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, 0.0, yaw))

        # Escrita na Memória Compartilhada
        async for odom in drone.telemetry.position_velocity_ned():
            shared_telemetry[0] = odom.position.north_m
            shared_telemetry[1] = odom.position.east_m
            shared_telemetry[2] = odom.position.down_m
            shared_telemetry[3] = odom.velocity.north_m_s
            shared_telemetry[4] = odom.velocity.east_m_s
            shared_telemetry[5] = odom.velocity.down_m_s
            break

        t += dt
        await asyncio.sleep(dt)

async def run_follower_logic(drone, drone_id, shared_telemetry):
    print(f"[Drone {drone_id}] 🎯 Iniciando perseguição...")
    
    # Pega o offset específico deste drone
    offset = OFFSETS[drone_id]

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"[Drone {drone_id}] ❌ Falha no Offboard: {error}")
        return

    while True:
        # 1. Ler Líder
        p_L = np.array([shared_telemetry[0], shared_telemetry[1], shared_telemetry[2]])
        v_L = np.array([shared_telemetry[3], shared_telemetry[4], shared_telemetry[5]])

        # 2. Ler Próprio
        p_i = np.array([0.0, 0.0, 0.0])
        async for odom in drone.telemetry.position_velocity_ned():
            p_i = np.array([odom.position.north_m, odom.position.east_m, odom.position.down_m])
            break
        
        # 3. Lei de Controle (Leader-Follower)
        # p_d = p_L + d_i
        p_d = p_L + offset
        
        # Erro = p_d - p_i
        error = p_d - p_i
        
        # u = v_L + Kp * error
        u_cmd = v_L + (KP * error)

        # 4. Atuação
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(u_cmd[0], u_cmd[1], u_cmd[2], 0.0)
        )

        await asyncio.sleep(0.1)

# -----------------------
# Utilitários
# -----------------------
async def connect_and_prep(system_address, label, grpc_port):
    drone = System(port=grpc_port)
    print(f"[{label}] 🔌 Conectando em {system_address}...")
    await drone.connect(system_address=system_address)

    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    print(f"[{label}] 🚁 Decolando...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(ALTITUDE)
    await drone.action.takeoff()
    
    # Espera subir (tempo fixo ou lógica de altitude)
    await asyncio.sleep(15) 
    return drone

# -----------------------
# Worker
# -----------------------
def drone_worker(d_id, sys_addr, grpc_port, cmd_queue, ready_evt, start_evt, shared_mem):
    label = f"Drone {d_id}"
    
    async def runner():
        try:
            drone = await connect_and_prep(sys_addr, label, grpc_port)
            ready_evt.set() 
            
            print(f"[{label}] ⏳ Aguardando sinal...")
            start_evt.wait()
            
            if d_id == 1:
                await run_leader_logic(drone, shared_mem)
            else:
                await run_follower_logic(drone, d_id, shared_mem)
                
        except Exception as e:
            print(f"[{label}] ❌ Erro: {e}")

    asyncio.run(runner())

# -----------------------
# Main
# -----------------------
if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)

    # Memória Compartilhada para o Líder
    leader_telemetry = mp.Array('d', 6)

    queues_cmd = [mp.Queue() for _ in range(NUM_DRONES)]
    events_ready = [mp.Event() for _ in range(NUM_DRONES)]
    start_all = mp.Event()

    base_port = 14540
    grpc_base = 50050
    processes = []

    print(f"--- Inicializando Enxame de {NUM_DRONES} Drones ---")

    for i in range(NUM_DRONES):
        d_id = i + 1
        sys_addr = f"udp://:{base_port + i}"
        
        p = mp.Process(
            target=drone_worker,
            args=(d_id, sys_addr, grpc_base + i, queues_cmd[i], events_ready[i], start_all, leader_telemetry)
        )
        p.start()
        processes.append(p)

    print("⌛ Aguardando decolagem...")
    for ev in events_ready:
        ev.wait()
    
    print("\n✅ Todos no ar! Iniciando Formação em 5 segundos...")
    time.sleep(5)
    
    start_all.set()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 Encerrando...")
        for p in processes:
            p.terminate()