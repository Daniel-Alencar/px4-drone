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
# Configurações do Enxame (VS <-> APF)
# -----------------------
NUM_DRONES = 8
ALTITUDE = 10.0

# Gatilho da Máquina de Estados AUMENTADO (Eles reagem mais cedo)
DISTANCIA_GATILHO_APF = 5.0  

# Ganhos do APF (Mais agressivo para evitar o emaranhado)
K_ATT = 3.0      # Atração suave para o assento virtual
K_REP = 2000.0   # Repulsão MUITO forte contra a parede
D_OBS = 5.0      # Raio de influência da parede igual ao gatilho
MAX_STEP = 1.0   

# Obstáculos Virtuais no plano NED
RAIO_CILINDRO = 1.5   
OBSTACULOS_ESTATICOS = [
<<<<<<< HEAD
    np.array([20.0, 0.0]), 
    np.array([20.0, 7.0]), 
    np.array([20.0, -7.0]),
    np.array([24.0, 3.5]), 
    np.array([24.0, -3.5]), 
    np.array([24.0, 10.5]),
    np.array([24.0, -10.5]), 
    np.array([28.0, 0.0]), 
=======
    np.array([20.0, 0.0]),
    np.array([20.0, 7.0]),
    np.array([20.0, -7.0]),
    np.array([24.0, 3.5]),
    np.array([24.0, -3.5]),
    np.array([24.0, 10.5]),
    np.array([24.0, -10.5]),
    np.array([28.0, 0.0]),
>>>>>>> 13ab0e3bdac14760de0a0b5b2f805d98414085e3
    np.array([28.0, 7.0]),
    np.array([28.0, -7.0])
]

# Geometria da Estrutura Virtual (Formação em V)
OFFSETS_VS = {
    0: np.array([0.0, 0.0]),      # Bico da Seta
    1: np.array([-3.0, 3.0]),     # Asa Direita
    2: np.array([-3.0, -3.0]),    # Asa Esquerda
    3: np.array([-6.0, 6.0]),     
    4: np.array([-6.0, -6.0]),    
    5: np.array([-9.0, 9.0]),     
    6: np.array([-9.0, -9.0]),    
    7: np.array([-12.0, -12.0])
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

def calcular_forca_apf_puro(pos_atual, pos_alvo, outras_posicoes, obstaculos):
    """ Matemática APF turbinada para evitar emaranhados """
    vetor_atracao = pos_alvo - pos_atual
    distancia_alvo = np.linalg.norm(vetor_atracao)
    
    if distancia_alvo > 0.1:
        f_att = K_ATT * (vetor_atracao / distancia_alvo)
    else:
        f_att = np.array([0.0, 0.0])
        
    f_rep = np.array([0.0, 0.0])
    
    # Repulsão das Paredes
    for p_obs in obstaculos:
        vetor_distancia = pos_atual - p_obs
        if abs(vetor_distancia[1]) < 0.2 and abs(vetor_distancia[0]) > 0:
            vetor_distancia[1] += 0.5 
            
        distancia = np.linalg.norm(vetor_distancia)
        
        if 0.1 < distancia < D_OBS: 
            termo1 = (1.0 / distancia) - (1.0 / D_OBS)
            termo2 = 1.0 / (distancia**3)
            f_rep += K_REP * termo1 * termo2 * vetor_distancia

    # Repulsão Inter-Robôs AUMENTADA para Drones 4 e 7 não baterem
    D_DRONE = 2.5 
    for p_drone in outras_posicoes:
        vetor_distancia = pos_atual - p_drone
        distancia = np.linalg.norm(vetor_distancia)
        if 0.1 < distancia < D_DRONE: 
            termo1 = (1.0 / distancia) - (1.0 / D_DRONE)
            termo2 = 1.0 / (distancia**3)
            f_rep += (K_REP * 0.8) * termo1 * termo2 * vetor_distancia

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
# Controlador Principal
# -----------------------
if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)
    
    queues_cmd = [mp.Queue() for _ in range(NUM_DRONES)]
    queues_resp = [mp.Queue() for _ in range(NUM_DRONES)]
    events_ready = [mp.Event() for _ in range(NUM_DRONES)]
    start_all = mp.Event()

    print(f"--- Inicializando Arquitetura CHAVEADA (VS <-> APF) para {NUM_DRONES} Drones ---")
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

    print("\n📐 FASE 1: Montando a Formação em 'V' na origem com Virtual Structure...")
    PV_ATUAL = np.array([0.0, 0.0])
    
    for i in range(NUM_DRONES):
        alvo_montagem = PV_ATUAL + OFFSETS_VS[i]
        t_lat, t_lon = ned_to_gps(alvo_montagem[0], alvo_montagem[1])
        queues_cmd[i].put(("goto", (t_lat, t_lon, ALTITUDE)))

    time.sleep(10)

    P_GOAL_NED = np.array([40.0, 0.0])
    VIRTUAL_STEP = 0.8 
    ALVOS_FINAIS = {i: P_GOAL_NED + OFFSETS_VS[i] for i in range(NUM_DRONES)}

    print(f"\n🛸 FASE 2: Movendo Estrutura para {P_GOAL_NED}. Modos: VS ou APF dependendo do ambiente.")
    
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
    
    while not all(chegaram):
        tempo_atual = time.time() - tempo_inicio_missao
        posicoes_atuais_ned = {}
        erros_de_rastreamento = [] 
        
        # 1. Obter posições
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

            # Verifica chegada no alvo absoluto FINAL do drone
            if np.linalg.norm(ALVOS_FINAIS[i] - pos_ned) < 1.5:
                chegaram[i] = True
                
            assento_virtual = PV_ATUAL + OFFSETS_VS[i]
            erros_de_rastreamento.append(np.linalg.norm(pos_ned - assento_virtual))

        # --- A ESTEIRA ROLANTE ADAPTATIVA ---
        # A Estrutura Virtual NUNCA para. Se o trânsito nos obstáculos ficar feio,
        # ela apenas desacelera. Isso garante que os assentos continuem deslizando 
        # para a frente, "puxando" os drones para fora da zona de perigo.
        erro_medio_formacao = np.mean(erros_de_rastreamento)
        if erro_medio_formacao < 3.0:
            velocidade_fantasma = VIRTUAL_STEP
        elif erro_medio_formacao < 8.0:
            velocidade_fantasma = VIRTUAL_STEP * 0.5  # Desacelera para esperar a equipe
        else:
            velocidade_fantasma = VIRTUAL_STEP * 0.2  # Quase parando, mas ainda puxando para frente

        f_goal_virtual = P_GOAL_NED - PV_ATUAL
        dist_virtual = np.linalg.norm(f_goal_virtual)
        if dist_virtual > 0.1:
            PV_ATUAL += (f_goal_virtual / dist_virtual) * min(velocidade_fantasma, dist_virtual)
        else:
            PV_ATUAL = P_GOAL_NED

        # 2. CHAVEAMENTO DE MODOS (MÁQUINA DE ESTADOS)
        tempo_matematica_iteracao = 0.0
        novas_posicoes_ned = {}
        
        # --- INÍCIO DO CRONÔMETRO ---
        t_inicio = time.perf_counter()
        
        for i in range(NUM_DRONES):
            if chegaram[i]:
                novas_posicoes_ned[i] = posicoes_atuais_ned[i]
                continue 
                
            p_i = posicoes_atuais_ned[i]
            assento_virtual_i = PV_ATUAL + OFFSETS_VS[i]
            
            # Distância para o obstáculo mais próximo
            dist_obstaculo_mais_proximo = min([np.linalg.norm(p_i - obs) for obs in OBSTACULOS_ESTATICOS]) if OBSTACULOS_ESTATICOS else float('inf')

            # --- O "INTERRUPTOR" DE ALGORITMOS ---
            if dist_obstaculo_mais_proximo < DISTANCIA_GATILHO_APF:
                # 🛑 MODO APF: O alvo do APF agora é o ASSENTO VIRTUAL (que está em movimento)
                outros_drones = [posicoes_atuais_ned[j] for j in range(NUM_DRONES) if j != i]
                forca = calcular_forca_apf_puro(p_i, assento_virtual_i, outros_drones, OBSTACULOS_ESTATICOS)
                novas_posicoes_ned[i] = p_i + forca
            else:
                # 🟩 MODO VIRTUAL STRUCTURE: Caminho livre, seja um corpo rígido!
                novas_posicoes_ned[i] = assento_virtual_i

        t_fim = time.perf_counter()
        # --- FIM DO CRONÔMETRO ---
        
        tempo_matematica_iteracao += (t_fim - t_inicio) * 1000.0

        if tempo_matematica_iteracao > 0:
            tempos_de_calculo_ms.append(tempo_matematica_iteracao)
            cpu_usada_list.append(processo_atual.cpu_percent())
            ram_usada_list.append(processo_atual.memory_info().rss / (1024 * 1024))

        # 3. Enviar Comandos
        for i in range(NUM_DRONES):
            if chegaram[i]: continue
            prox_ned = novas_posicoes_ned[i]
            prox_lat, prox_lon = ned_to_gps(prox_ned[0], prox_ned[1])
            queues_cmd[i].put(("goto", (prox_lat, prox_lon, ALTITUDE)))

        time.sleep(0.2)

    tempo_total = time.time() - tempo_inicio_missao
    print("\n✅ Sucesso! A formação quebrou para desviar e se reconstruiu LOGO APÓS os obstáculos.")
    
    # ---------------------------------------------------------
    # CÁLCULO DAS MÉTRICAS COMPUTACIONAIS E EXPORTAÇÃO
    # ---------------------------------------------------------
    media_calc_ms = sum(tempos_de_calculo_ms) / len(tempos_de_calculo_ms) if tempos_de_calculo_ms else 0
    max_calc_ms = max(tempos_de_calculo_ms) if tempos_de_calculo_ms else 0
    media_cpu = sum(cpu_usada_list) / len(cpu_usada_list) if cpu_usada_list else 0
    max_ram = max(ram_usada_list) if ram_usada_list else 0

    NOME_ALGORITMO = "Switching (VS -> APF -> VS)"
    arquivo_benchmark = "comparativo_computacional.csv"
    arquivo_existe = os.path.isfile(arquivo_benchmark)

    with open(arquivo_benchmark, mode='a', newline='', encoding='utf-8') as f:
        escritor_bench = csv.writer(f)
        if not arquivo_existe:
            escritor_bench.writerow(["Algoritmo", "Num_Drones", "Media_Calc_ms", "Pico_Calc_ms", "Media_CPU_Perc", "Pico_RAM_MB", "Tempo_Missao_s"])
        escritor_bench.writerow([NOME_ALGORITMO, NUM_DRONES, round(media_calc_ms, 4), round(max_calc_ms, 4), round(media_cpu, 2), round(max_ram, 2), round(tempo_total, 2)])

    nome_arquivo_timestamp = f"resultados.csv"
    with open(nome_arquivo_timestamp, mode='w', newline='') as arquivo_csv:
        escritor = csv.writer(arquivo_csv)
        escritor.writerow(["Tempo_s", "Drone_ID", "Pos_N_Metros", "Pos_E_Metros", "Distancia_Percorrida_Acumulada_m", "Distancia_Minima_Obstaculo_Instante_m"])
        escritor.writerows(dados_log)

    nome_arquivo_txt = f"relatorio.txt"

    relatorio = "\n" + "="*50 + "\n"
    relatorio += "📊 RELATÓRIO DE MÉTRICAS DA MISSÃO (SWITCHING VS-APF)\n"
    relatorio += "="*50 + "\n"
    relatorio += f"⏱️ Tempo Total de Convergência: {tempo_total:.2f} segundos\n"
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