import multiprocessing as mp
import asyncio
import time
import math
import numpy as np
import csv
import os
import psutil
from datetime import datetime
from mavsdk import System

# -----------------------
# Configurações da Estrutura Virtual
# -----------------------
NUM_DRONES = 8
ALTITUDE = 10.0

# Obstáculos Virtuais no plano NED (Apenas para o Logger medir a colisão)
RAIO_CILINDRO = 1.5   
OBSTACULOS_ESTATICOS = [
    np.array([20.0, 0.0]), np.array([20.0, 7.0]), np.array([20.0, -7.0]),
    np.array([24.0, 3.5]), np.array([24.0, -3.5]), np.array([24.0, 10.5]),
    np.array([24.0, -10.5]), np.array([28.0, 0.0]), np.array([28.0, 7.0]),
    np.array([28.0, -7.0])
]

# Define a geometria da "Estrutura Virtual" (r_i^{VS})
# Formato de Seta/V. Tupla: (Offset_Norte, Offset_Leste) em metros
# Norte negativo = atrás do líder
OFFSETS_VS = {
    0: (0.0, 0.0),      # Posição 0: Bico da Seta (Centro Virtual)
    1: (-3.0, 3.0),     # Posição 1: Asa Direita
    2: (-3.0, -3.0),    # Posição 2: Asa Esquerda
    3: (-6.0, 6.0),     # Posição 3: Asa Direita
    4: (-6.0, -6.0),    # Posição 4: Asa Esquerda
    5: (-9.0, 9.0),     # Posição 5: Asa Direita
    6: (-9.0, -9.0),    # Posição 6: Asa Esquerda
    7: (-12.0, -12.0)
}

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

def rotate_offset(n, e, yaw_deg):
    """ Rotaciona os offsets locais da estrutura baseada no 'yaw' do centro virtual. """
    rad = math.radians(yaw_deg)
    n_rot = n * math.cos(rad) - e * math.sin(rad)
    e_rot = n * math.sin(rad) + e * math.cos(rad)
    return n_rot, e_rot

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
            lat, lon, alt, yaw = cmd[1]
            await drone.action.goto_location(lat, lon, alt, yaw)
        elif cmd[0] == "land":
            await drone.action.land()
        elif cmd[0] == "exit":
            break

def process_runner(*args):
    asyncio.run(worker_loop(*args))

# -----------------------
# Controlador Principal - Virtual Structure
# -----------------------
if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)
    
    queues_cmd = [mp.Queue() for _ in range(NUM_DRONES)]
    queues_resp = [mp.Queue() for _ in range(NUM_DRONES)]
    events_ready = [mp.Event() for _ in range(NUM_DRONES)]
    start_all = mp.Event()

    print(f"--- Inicializando Arquitetura Virtual Structure para {NUM_DRONES} Drones ---")
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

    # ---------------------------------------------------------
    # FASE 1: MONTAGEM DA ESTRUTURA (Corpo Rígido na Origem)
    # ---------------------------------------------------------
    print("\n📐 FASE 1: Montando a Formação em 'V' na origem...")
    PV_ATUAL = np.array([0.0, 0.0]) # Centro virtual parado no zero
    VIRTUAL_YAW = 0.0

    for i in range(NUM_DRONES):
        offset_n, offset_e = OFFSETS_VS[i]
        rot_n, rot_e = rotate_offset(offset_n, offset_e, VIRTUAL_YAW)
        alvo_montagem_n = PV_ATUAL[0] + rot_n
        alvo_montagem_e = PV_ATUAL[1] + rot_e
        
        t_lat, t_lon = ned_to_gps(alvo_montagem_n, alvo_montagem_e)
        queues_cmd[i].put(("goto", (t_lat, t_lon, ALTITUDE, VIRTUAL_YAW)))

    print("⏳ Aguardando os drones assumirem suas posições na geometria (15s)...")
    time.sleep(15)
    print("✅ Estrutura Rígida formada!")

    # ---------------------------------------------------------
    # FASE 2: MIGRAÇÃO (O Corpo Rígido voa até o destino)
    # ---------------------------------------------------------
    P_GOAL_NED = np.array([80.0, 0.0])
    VIRTUAL_STEP = 0.8 # Velocidade do Centro Virtual (metros por iteração)

    # Pre-calcula os alvos absolutos finais de cada drone para checar o fim da missão
    ALVOS_FINAIS = {}
    for i in range(NUM_DRONES):
        off_n, off_e = OFFSETS_VS[i]
        r_n, r_e = rotate_offset(off_n, off_e, VIRTUAL_YAW)
        ALVOS_FINAIS[i] = P_GOAL_NED + np.array([r_n, r_e])

    print(f"\n🚀 FASE 2: Movendo a Estrutura em bloco para {P_GOAL_NED}...")

    # Variáveis de Log e Benchmark
    dados_log = [] 
    distancia_percorrida = {i: 0.0 for i in range(NUM_DRONES)}
    posicao_anterior = {i: None for i in range(NUM_DRONES)}
    menor_distancia_obstaculo = {i: float('inf') for i in range(NUM_DRONES)}
    
    processo_atual = psutil.Process(os.getpid())
    processo_atual.cpu_percent() 
    tempos_de_calculo_ms = []
    cpu_usada_list = []
    ram_usada_list = []

    tempo_inicio_missao = time.time()
    chegaram = [False] * NUM_DRONES

    # Loop de Controle
    while not all(chegaram):
        tempo_atual = time.time() - tempo_inicio_missao
        posicoes_atuais_ned = {}
        
        # 1. Coletar e logar posições atuais
        for i in range(NUM_DRONES):
            queues_cmd[i].put(("getpos", None))
            while queues_resp[i].qsize() > 1: queues_resp[i].get_nowait()
            
            lat, lon, _ = queues_resp[i].get()[1]
            pos_ned = gps_to_ned(lat, lon)
            posicoes_atuais_ned[i] = pos_ned

            if posicao_anterior[i] is not None:
                distancia_percorrida[i] += np.linalg.norm(pos_ned - posicao_anterior[i])
            posicao_anterior[i] = pos_ned
            
            min_dist_neste_instante = float('inf')
            for obs in OBSTACULOS_ESTATICOS:
                d = np.linalg.norm(pos_ned - obs)
                if d < min_dist_neste_instante:
                    min_dist_neste_instante = d
            
            if min_dist_neste_instante < menor_distancia_obstaculo[i]:
                menor_distancia_obstaculo[i] = min_dist_neste_instante

            dados_log.append([tempo_atual, i, pos_ned[0], pos_ned[1], distancia_percorrida[i], min_dist_neste_instante])

            # Condição de chegada baseada no alvo final absoluto do drone
            dist_ao_fim = np.linalg.norm(ALVOS_FINAIS[i] - pos_ned)
            if dist_ao_fim < 1.5:
                chegaram[i] = True

        # 2. Cinemática da Estrutura Virtual (Com Cronômetro)
        novas_posicoes_ned = {}
        tempo_matematica_iteracao = 0.0

        # --- INÍCIO DO CRONÔMETRO (MATEMÁTICA VS) ---
        t_inicio = time.perf_counter()
        
        # O Centro Virtual avança em direção ao objetivo Global (40,0)
        f_goal_virtual = P_GOAL_NED - PV_ATUAL
        dist_virtual = np.linalg.norm(f_goal_virtual)
        
        if dist_virtual > 0.1:
            PV_ATUAL += (f_goal_virtual / dist_virtual) * min(VIRTUAL_STEP, dist_virtual)
        else:
            PV_ATUAL = P_GOAL_NED # Cravou no destino

        # Projeta a posição instantânea de cada drone baseada no PV_ATUAL
        for i in range(NUM_DRONES):
            offset_n, offset_e = OFFSETS_VS[i]
            rot_n, rot_e = rotate_offset(offset_n, offset_e, VIRTUAL_YAW)
            novas_posicoes_ned[i] = PV_ATUAL + np.array([rot_n, rot_e])

        t_fim = time.perf_counter()
        # --- FIM DO CRONÔMETRO ---

        tempo_matematica_iteracao = (t_fim - t_inicio) * 1000.0
        if tempo_matematica_iteracao > 0:
            tempos_de_calculo_ms.append(tempo_matematica_iteracao)
            cpu_usada_list.append(processo_atual.cpu_percent())
            ram_usada_list.append(processo_atual.memory_info().rss / (1024 * 1024))

        # 3. Enviar os Comandos de Movimento Atualizados
        for i in range(NUM_DRONES):
            # Se ele já chegou no destino final, não precisa mais de atualizações
            if chegaram[i]: continue 
            
            prox_ned = novas_posicoes_ned[i]
            prox_lat, prox_lon = ned_to_gps(prox_ned[0], prox_ned[1])
            queues_cmd[i].put(("goto", (prox_lat, prox_lon, ALTITUDE, VIRTUAL_YAW)))
            
        time.sleep(0.2) # A cada 0.2s a estrutura "pula" 0.8 metros (Velocidade = 4m/s)

    tempo_total = time.time() - tempo_inicio_missao
    print("\n✅ Estrutura Virtual inteira chegou ao destino final!")
    
    # ---------------------------------------------------------
    # CÁLCULO DAS MÉTRICAS COMPUTACIONAIS
    # ---------------------------------------------------------
    media_calc_ms = sum(tempos_de_calculo_ms) / len(tempos_de_calculo_ms) if tempos_de_calculo_ms else 0
    max_calc_ms = max(tempos_de_calculo_ms) if tempos_de_calculo_ms else 0
    media_cpu = sum(cpu_usada_list) / len(cpu_usada_list) if cpu_usada_list else 0
    max_ram = max(ram_usada_list) if ram_usada_list else 0

    NOME_ALGORITMO = "Virtual Structure"
    arquivo_benchmark = "comparativo_computacional.csv"
    arquivo_existe = os.path.isfile(arquivo_benchmark)

    with open(arquivo_benchmark, mode='a', newline='', encoding='utf-8') as f:
        escritor_bench = csv.writer(f)
        if not arquivo_existe:
            escritor_bench.writerow(["Algoritmo", "Num_Drones", "Media_Calc_ms", "Pico_Calc_ms", "Media_CPU_Perc", "Pico_RAM_MB", "Tempo_Missao_s"])
        escritor_bench.writerow([NOME_ALGORITMO, NUM_DRONES, round(media_calc_ms, 4), round(max_calc_ms, 4), round(media_cpu, 2), round(max_ram, 2), round(tempo_total, 2)])

    # ---------------------------------------------------------
    # EXPORTAÇÃO DOS DADOS E RELATÓRIO FINAL
    # ---------------------------------------------------------
    nome_arquivo_timestamp = f"resultados.csv"
    with open(nome_arquivo_timestamp, mode='w', newline='') as arquivo_csv:
        escritor = csv.writer(arquivo_csv)
        escritor.writerow(["Tempo_s", "Drone_ID", "Pos_N_Metros", "Pos_E_Metros", "Distancia_Percorrida_Acumulada_m", "Distancia_Minima_Obstaculo_Instante_m"])
        escritor.writerows(dados_log)

    nome_arquivo_txt = f"relatorio.txt"

    relatorio = "\n" + "="*50 + "\n"
    relatorio += "📊 RELATÓRIO DE MÉTRICAS DA MISSÃO (VIRTUAL STRUCTURE)\n"
    relatorio += "="*50 + "\n"
    relatorio += f"⏱️ Tempo Total de Convergência: {tempo_total:.2f} segundos\n"
    relatorio += f"💾 Arquivos salvos: {nome_arquivo_timestamp} e {nome_arquivo_txt}\n"
    relatorio += f"📈 Benchmark Global atualizado: {arquivo_benchmark}\n"
    relatorio += "-" * 50 + "\n"
    
    distancia_media = 0
    for i in range(NUM_DRONES):
        distancia_media += distancia_percorrida[i]
        relatorio += f"Drone {i}: Voou {distancia_percorrida[i]:.2f}m | Margem Segurança (Obs): {menor_distancia_obstaculo[i]:.2f}m\n"
    
    relatorio += "-" * 50 + "\n"
    relatorio += f"📏 Distância média percorrida pelo enxame: {(distancia_media/NUM_DRONES):.2f}m\n"
    relatorio += "-" * 50 + "\n"
    relatorio += "💻 CUSTO COMPUTACIONAL DO ALGORITMO\n"
    relatorio += "-" * 50 + "\n"
    relatorio += f"⏱️ Tempo Médio de Cálculo por Loop: {media_calc_ms:.4f} ms\n"
    relatorio += f"⚠️ Pico Máximo de Cálculo (Gargalo): {max_calc_ms:.4f} ms\n"
    relatorio += f"🧠 Uso Médio de CPU do Algoritmo: {media_cpu:.2f}%\n"
    relatorio += f"📦 Consumo de RAM (Pico): {max_ram:.2f} MB\n"
    relatorio += "="*50 + "\n"

    print(relatorio)
    with open(nome_arquivo_txt, "w", encoding="utf-8") as arquivo_txt:
        arquivo_txt.write(relatorio)

    print("🛬 Pousando em formação...")
    for q in queues_cmd: q.put(("land", None))
    time.sleep(10)
    for q in queues_cmd: q.put(("exit", None))
    for p in processes: p.terminate()