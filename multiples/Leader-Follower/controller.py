import multiprocessing as mp
import asyncio
import time
import math
import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, PositionNedYaw

# -----------------------
# Parâmetros e Matemática (Baseado no TCC)
# -----------------------
ALTITUDE = 10.0
KP = 1.0  # Ganho Proporcional (Eq. 4 do TCC simplificada)

# Offsets: Distância desejada (d_i) no referencial NED
# Drone 1 (Líder): [0, 0, 0]
# Drone 2 (Seguidor): [-3m (atrás), -3m (esquerda), 0m (mesma altura)]
OFFSETS = {
    1: np.array([0.0, 0.0, 0.0]),
    2: np.array([-3.0, -3.0, 0.0]) 
}

# -----------------------
# Lógica de Controle
# -----------------------
async def run_leader_logic(drone, shared_telemetry):
    """
    Implementa a trajetória do líder e 'transmite' o estado via memória compartilhada.
    """
    print("[Líder] 🔄 Iniciando trajetória circular (Offboard)...")
    
    # Parâmetros da curva
    t = 0
    dt = 0.1
    radius = 8.0
    omega = 0.3 # Velocidade angular

    # Prepara Offboard (envia setpoint inicial antes de iniciar)
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"[Líder] ❌ Falha no Offboard: {error}")
        return

    while True:
        # 1. Cinemática da Trajetória (Líder dita o ritmo)
        vn = radius * omega * math.cos(omega * t)
        ve = radius * omega * math.sin(omega * t)
        yaw = math.degrees(math.atan2(ve, vn))

        # 2. Envia comando ao piloto automático
        await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, 0.0, yaw))

        # 3. Atualiza Memória Compartilhada (Simula transmissão de rede)
        # O Líder escreve: [N, E, D, Vn, Ve, Vd]
        async for odom in drone.telemetry.position_velocity_ned():
            shared_telemetry[0] = odom.position.north_m
            shared_telemetry[1] = odom.position.east_m
            shared_telemetry[2] = odom.position.down_m
            shared_telemetry[3] = odom.velocity.north_m_s
            shared_telemetry[4] = odom.velocity.east_m_s
            shared_telemetry[5] = odom.velocity.down_m_s
            break # Lê apenas uma vez por loop

        t += dt
        await asyncio.sleep(dt)

async def run_follower_logic(drone, drone_id, shared_telemetry):
    """
    Implementa a Lei de Controle Leader-Follower (Eq. 4 do TCC).
    u_i = v_L + Kp * (p_d - p_i)
    """
    print(f"[Seguidor {drone_id}] 🎯 Iniciando perseguição...")
    
    offset = OFFSETS[drone_id]

    # Prepara Offboard
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"[Seguidor] ❌ Falha no Offboard: {error}")
        return

    while True:
        # 1. Percepção: Ler estado do Líder (Memória Compartilhada)
        p_L = np.array([shared_telemetry[0], shared_telemetry[1], shared_telemetry[2]])
        v_L = np.array([shared_telemetry[3], shared_telemetry[4], shared_telemetry[5]])

        # 2. Percepção: Ler estado próprio (GPS/IMU)
        p_i = np.array([0.0, 0.0, 0.0])
        async for odom in drone.telemetry.position_velocity_ned():
            p_i = np.array([odom.position.north_m, odom.position.east_m, odom.position.down_m])
            break
        
        # 3. Cálculo do Controle (Matemática do TCC)
        # Posição Desejada (Eq. 2): p_d = p_L + d_i
        p_d = p_L + offset
        
        # Erro de Posição (Eq. 3): e = p_d - p_i
        error = p_d - p_i
        
        # Lei de Controle (Eq. 4): u = v_L + Kp * error
        # v_L é o feed-forward (copia velocidade do líder)
        # Kp * error corrige o desvio de posição
        u_cmd = v_L + (KP * error)

        # 4. Atuação
        # Enviamos 0.0 no Yaw para focar apenas na translação por enquanto
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(u_cmd[0], u_cmd[1], u_cmd[2], 0.0)
        )

        await asyncio.sleep(0.1) # 10Hz

# -----------------------
# Utilitários de Conexão
# -----------------------
async def connect_and_prep(system_address, label, grpc_port):
    # Porta gRPC única é crucial para estabilidade em paralelo
    drone = System(port=grpc_port)
    print(f"[{label}] 🔌 Conectando em {system_address}...")
    await drone.connect(system_address=system_address)

    # Espera conectar
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[{label}] ✅ Conectado.")
            break

    print(f"[{label}] 🚁 Armando e Decolando...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(ALTITUDE)
    await drone.action.takeoff()
    
    # Espera estabilizar no ar
    await asyncio.sleep(12)
    return drone

# -----------------------
# Worker (Processo Isolado)
# -----------------------
def drone_worker(d_id, sys_addr, grpc_port, cmd_queue, ready_evt, start_evt, shared_mem):
    label = f"Drone {d_id}"
    
    async def runner():
        try:
            # 1. Conecta e Decola
            drone = await connect_and_prep(sys_addr, label, grpc_port)
            ready_evt.set() # Avisa main que está no ar
            
            # 2. Espera sinal para iniciar formação (sincronia)
            print(f"[{label}] ⏳ Aguardando sinal de formação...")
            start_evt.wait()
            
            # 3. Entra no loop de controle contínuo
            if d_id == 1:
                await run_leader_logic(drone, shared_mem)
            else:
                await run_follower_logic(drone, d_id, shared_mem)
                
        except Exception as e:
            print(f"[{label}] ❌ Erro Crítico: {e}")

    asyncio.run(runner())

# -----------------------
# Main Controller
# -----------------------
if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)

    # Memória Compartilhada (Array de doubles)
    # Guarda o estado do Líder: [PosN, PosE, PosD, VelN, VelE, VelD]
    leader_telemetry = mp.Array('d', 6)

    n_drones = 2
    queues_cmd = [mp.Queue() for _ in range(n_drones)]
    events_ready = [mp.Event() for _ in range(n_drones)]
    start_all = mp.Event()

    base_port = 14540
    grpc_base = 50050
    processes = []

    print("--- Inicializando Arquitetura Leader-Follower ---")

    for i in range(n_drones):
        d_id = i + 1
        sys_addr = f"udp://:{base_port + i}"
        
        p = mp.Process(
            target=drone_worker,
            args=(d_id, sys_addr, grpc_base + i, queues_cmd[i], events_ready[i], start_all, leader_telemetry)
        )
        p.start()
        processes.append(p)

    print("⌛ Aguardando decolagem de todos os drones...")
    for ev in events_ready:
        ev.wait()
    
    print("\n✅ Todos no ar! Iniciando Formação em 5 segundos...")
    time.sleep(5)
    
    # Libera os workers para entrarem no loop de controle ao mesmo tempo
    start_all.set()

    try:
        # Mantém o script principal rodando enquanto a simulação acontece
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 Encerrando simulação...")
        for p in processes:
            p.terminate()