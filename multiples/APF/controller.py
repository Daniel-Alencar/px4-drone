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
# Configurações do APF
# -----------------------
NUM_DRONES = 8
ALTITUDE = 10.0

# Ganhos do Campo de Potencial Balanceados
K_ATT = 5.0      # Força constante para o objetivo
K_REP = 500.0    # Força de repulsão (obstáculos)
D_OBS = 3.0      # Raio de influência dos obstáculos (metros)
MAX_STEP = 1.0   # Passo máximo (suaviza o movimento)

# Obstáculos Virtuais no plano NED (Norte, Leste)
OBSTACULOS_ESTATICOS = [
   # np.array([20.0, 0.0]),
   # np.array([20.0, 7.0]),
   # np.array([20.0, -7.0]),
   # np.array([24.0, 3.5]),
   # np.array([24.0, -3.5]),
   # np.array([24.0, 10.5]),
   # np.array([24.0, -10.5]),
   # np.array([28.0, 0.0]),
   # np.array([28.0, 7.0]),
   # np.array([28.0, -7.0])
]

# -----------------------
# Coordenadas Absolutas do Gazebo (Origem X=0, Y=0)
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
    new_lat = REF_LAT + math.degrees(d_lat)
    new_lon = REF_LON + math.degrees(d_lon)
    return new_lat, new_lon

def calcular_forcas_apf(pos_atual, pos_objetivo, outras_posicoes, obstaculos):
    vetor_atracao = pos_objetivo - pos_atual
    distancia_alvo = np.linalg.norm(vetor_atracao)
    
    if distancia_alvo > 0.1:
        f_att = K_ATT * (vetor_atracao / distancia_alvo)
    else:
        f_att = np.array([0.0, 0.0])
        
    f_rep = np.array([0.0, 0.0])
    
    for p_obs in obstaculos:
        vetor_distancia = pos_atual - p_obs
        if abs(vetor_distancia[1]) < 0.2 and abs(vetor_distancia[0]) > 0:
            vetor_distancia[1] += 0.5
            
        distancia = np.linalg.norm(vetor_distancia)
        
        if 0.1 < distancia < D_OBS: 
            termo1 = (1.0 / distancia) - (1.0 / D_OBS)
            termo2 = 1.0 / (distancia**3)
            f_rep += K_REP * termo1 * termo2 * vetor_distancia

    D_DRONE = 1.8 
    for p_drone in outras_posicoes:
        vetor_distancia = pos_atual - p_drone
        distancia = np.linalg.norm(vetor_distancia)
        
        if 0.1 < distancia < D_DRONE: 
            termo1 = (1.0 / distancia) - (1.0 / D_DRONE)
            termo2 = 1.0 / (distancia**3)
            f_rep += (K_REP * 0.5) * termo1 * termo2 * vetor_distancia

    f_total = f_att + f_rep
    
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
    time.sleep(15) 

    objetivos_ned = {}
    for i in range(NUM_DRONES):
        objetivos_ned[i] = np.array([40.0, float(i * 2)]) 

    print("\n🏁 INICIANDO NAVEGAÇÃO POR CAMPOS POTENCIAIS (APF)")
    
    # ---------------------------------------------------------
    # INICIALIZAÇÃO DO RASTREAMENTO DE MÉTRICAS (DATA LOGGER E BENCHMARK)
    # ---------------------------------------------------------
    dados_log = [] 
    distancia_percorrida = {i: 0.0 for i in range(NUM_DRONES)}
    posicao_anterior = {i: None for i in range(NUM_DRONES)}
    menor_distancia_obstaculo = {i: float('inf') for i in range(NUM_DRONES)}
    
    # Medidores Computacionais
    processo_atual = psutil.Process(os.getpid())
    processo_atual.cpu_percent() # Primeira chamada para calibrar o psutil
    tempos_de_calculo_ms = []
    cpu_usada_list = []
    ram_usada_list = []

    tempo_inicio_missao = time.time()
    chegaram = [False] * NUM_DRONES
    
    # Loop de Controle
    while not all(chegaram):
        tempo_atual = time.time() - tempo_inicio_missao
        posicoes_atuais_ned = {}
        
        # 1. Obter a posição GPS atual de todos e converter para NED
        for i in range(NUM_DRONES):
            queues_cmd[i].put(("getpos", None))
            while queues_resp[i].qsize() > 1: queues_resp[i].get_nowait()
            
            lat, lon, _ = queues_resp[i].get()[1]
            pos_ned = gps_to_ned(lat, lon)
            posicoes_atuais_ned[i] = pos_ned
            
            # Atualização de métricas de voo
            if posicao_anterior[i] is not None:
                dist_passo = np.linalg.norm(pos_ned - posicao_anterior[i])
                distancia_percorrida[i] += dist_passo
            posicao_anterior[i] = pos_ned
            
            min_dist_neste_instante = float('inf')
            for obs in OBSTACULOS_ESTATICOS:
                d = np.linalg.norm(pos_ned - obs)
                if d < min_dist_neste_instante:
                    min_dist_neste_instante = d
            
            if min_dist_neste_instante < menor_distancia_obstaculo[i]:
                menor_distancia_obstaculo[i] = min_dist_neste_instante

            dados_log.append([tempo_atual, i, pos_ned[0], pos_ned[1], distancia_percorrida[i], min_dist_neste_instante])

            dist_ao_alvo = np.linalg.norm(objetivos_ned[i] - pos_ned)
            if dist_ao_alvo < 1.5:
                chegaram[i] = True

        # 2. Calcular Campos de Potencial (Com Cronômetro)
        tempo_matematica_iteracao = 0.0
        
        for i in range(NUM_DRONES):
            if chegaram[i]:
                continue 
                
            p_i = posicoes_atuais_ned[i]
            p_goal = objetivos_ned[i]
            outros_drones = [posicoes_atuais_ned[j] for j in range(NUM_DRONES) if j != i]
            
            # --- INÍCIO DO CRONÔMETRO (APENAS A MATEMÁTICA) ---
            t_inicio = time.perf_counter()
            forca_resultante = calcular_forcas_apf(p_i, p_goal, outros_drones, OBSTACULOS_ESTATICOS)
            t_fim = time.perf_counter()
            # --- FIM DO CRONÔMETRO ---
            
            tempo_matematica_iteracao += (t_fim - t_inicio) * 1000.0
            
            proxima_pos_ned = p_i + forca_resultante
            prox_lat, prox_lon = ned_to_gps(proxima_pos_ned[0], proxima_pos_ned[1])
            
            queues_cmd[i].put(("goto", (prox_lat, prox_lon, ALTITUDE)))

        # Registra as métricas computacionais do loop (se houve cálculo)
        if tempo_matematica_iteracao > 0:
            tempos_de_calculo_ms.append(tempo_matematica_iteracao)
            cpu_usada_list.append(processo_atual.cpu_percent())
            ram_usada_list.append(processo_atual.memory_info().rss / (1024 * 1024)) # Em MB

        time.sleep(0.2)

    tempo_total = time.time() - tempo_inicio_missao
    print("\n✅ Todos os drones alcançaram seus objetivos!")
    
    # ---------------------------------------------------------
    # CÁLCULO DAS MÉTRICAS COMPUTACIONAIS
    # ---------------------------------------------------------
    media_calc_ms = sum(tempos_de_calculo_ms) / len(tempos_de_calculo_ms) if tempos_de_calculo_ms else 0
    max_calc_ms = max(tempos_de_calculo_ms) if tempos_de_calculo_ms else 0
    media_cpu = sum(cpu_usada_list) / len(cpu_usada_list) if cpu_usada_list else 0
    max_ram = max(ram_usada_list) if ram_usada_list else 0

    # Exportar Benchmark Global
    NOME_ALGORITMO = "APF"
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

    # Montando o relatório em uma variável de texto
    relatorio = "\n" + "="*50 + "\n"
    relatorio += "📊 RELATÓRIO DE MÉTRICAS DA MISSÃO (APF)\n"
    relatorio += "="*50 + "\n"
    relatorio += f"⏱️  Tempo Total de Convergência: {tempo_total:.2f} segundos\n"
    relatorio += f"💾 Arquivo de log (CSV) salvo como: {nome_arquivo_timestamp}\n"
    relatorio += f"📝 Relatório salvo como: {nome_arquivo_txt}\n"
    relatorio += f"📈 Benchmark Global atualizado: {arquivo_benchmark}\n"
    relatorio += "-" * 50 + "\n"
    
    distancia_media = 0
    for i in range(NUM_DRONES):
        distancia_media += distancia_percorrida[i]
        relatorio += f"Drone {i}: Voou {distancia_percorrida[i]:.2f}m | Margem Segurança (Obs): {menor_distancia_obstaculo[i]:.2f}m\n"
    
    relatorio += "-" * 50 + "\n"
    relatorio += f"📏 Distância média percorrida pelo enxame: {(distancia_media/NUM_DRONES):.2f}m\n"
    
    # Adicionando o Custo Computacional no .txt
    relatorio += "-" * 50 + "\n"
    relatorio += "💻 CUSTO COMPUTACIONAL DO ALGORITMO\n"
    relatorio += "-" * 50 + "\n"
    relatorio += f"⏱️ Tempo Médio de Cálculo por Loop: {media_calc_ms:.4f} ms\n"
    relatorio += f"⚠️ Pico Máximo de Cálculo (Gargalo): {max_calc_ms:.4f} ms\n"
    relatorio += f"🧠 Uso Médio de CPU do Algoritmo: {media_cpu:.2f}%\n"
    relatorio += f"📦 Consumo de RAM (Pico): {max_ram:.2f} MB\n"
    relatorio += "="*50 + "\n"

    # Imprime no terminal para você ver
    print(relatorio)

    # Salva o texto no arquivo .txt
    with open(nome_arquivo_txt, "w", encoding="utf-8") as arquivo_txt:
        arquivo_txt.write(relatorio)

    print("🛬 Pousando...")
    for q in queues_cmd: q.put(("land", None))
    time.sleep(10)
    for q in queues_cmd: q.put(("exit", None))
    for p in processes: p.terminate()