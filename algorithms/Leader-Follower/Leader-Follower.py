import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd

# --------------------------
# 1. Parâmetros da Simulação
# --------------------------
# Configurações de tempo e simulação
dt = 0.1
steps = 600
N_followers = 3  # Número de seguidores

# Ganhos do controlador (Eq. 4 no texto )
Kp = 1.5  # Ganho proporcional (corrige erro de posição)
Kd = 0.5  # Ganho derivativo (corrige erro de velocidade)

# Limites físicos
max_speed = 3.0  # Velocidade máxima dos drones

# --------------------------
# 2. Definição da Formação (Geometria)
# --------------------------
# Deslocamentos desejados (d_i) no referencial do Líder (Local Frame)
# Formação em "V" invertido ou Delta atrás do líder
# [x (frente), y (esquerda)]
offsets_local = np.array([
    [-1.5,  1.5], # Seguidor 1: Atrás à esquerda
    [-1.5, -1.5], # Seguidor 2: Atrás à direita
    [-3.0,  0.0]  # Seguidor 3: Mais atrás no centro
])

# --------------------------
# 3. Inicialização de Estados
# --------------------------
# Líder (Inicia na origem)
pos_leader = np.array([0.0, 0.0])
vel_leader = np.array([0.0, 0.0])

# Seguidores (Iniciam em posições aleatórias para provar convergência)
np.random.seed(42)
pos_followers = np.random.uniform(-5, 0, (N_followers, 2))
vel_followers = np.zeros((N_followers, 2))

# Histórico para plotagem
# [step, leader + N_followers, x/y]
traj_hist = np.zeros((steps + 1, N_followers + 1, 2))
traj_hist[0, 0, :] = pos_leader
traj_hist[0, 1:, :] = pos_followers

# --------------------------
# 4. Funções Auxiliares
# --------------------------

def get_leader_state(t):
    """
    Gera uma trajetória circular para o líder.
    Retorna: posição, velocidade e ângulo de orientação (psi).
    """
    # Raio e velocidade angular da trajetória
    R = 8.0
    omega = 0.15 
    
    # Cinemática circular
    x = R * np.cos(omega * t)
    y = R * np.sin(omega * t)
    vx = -R * omega * np.sin(omega * t)
    vy = R * omega * np.cos(omega * t)
    
    # Orientação (heading) baseada no vetor velocidade
    psi = np.arctan2(vy, vx)
    
    return np.array([x, y]), np.array([vx, vy]), psi

def rotation_matrix(psi):
    """
    Matriz de rotação 2D R_L(t) baseada no ângulo psi.
    Referência: Texto menciona R_L(t) pertencente a SO(3), adaptamos para SO(2) 
    para simulação planar.
    """
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s], [s, c]])

# --------------------------
# 5. Loop de Simulação
# --------------------------
print("Iniciando simulação Leader-Follower...")

for k in range(steps):
    t = k * dt
    
    # A. Atualiza Líder (Trajetória pré-definida)
    p_L, v_L, psi_L = get_leader_state(t)
    R_L = rotation_matrix(psi_L)
    
    # Atualiza posição do líder na simulação atual
    pos_leader = p_L
    vel_leader = v_L
    
    # B. Controle dos Seguidores
    for i in range(N_followers):
        # 1. Calcular Posição Desejada Global (Eq. 2 )
        # p_d = p_L + R_L * d_i
        d_i = offsets_local[i] # Vetor de deslocamento fixo
        p_desired = p_L + R_L @ d_i 
        
        # 2. Calcular Erro de Posição (Eq. 3 [cite: 123])
        # e_i = p_desired - p_atual
        p_current = pos_followers[i]
        error_pos = p_desired - p_current
        
        # 3. Lei de Controle de Velocidade (Eq. 4 )
        # u_i = v_L + Kp * erro + Kd * (v_L - v_i)
        v_current = vel_followers[i]
        
        # Termo derivativo (erro de velocidade)
        error_vel = v_L - v_current
        
        # Comando de velocidade
        u_i = v_L + Kp * error_pos + Kd * error_vel
        
        # Saturação de velocidade (simulação física básica)
        speed = np.linalg.norm(u_i)
        if speed > max_speed:
            u_i = u_i * (max_speed / speed)
            
        # 4. Atualizar dinâmica do Seguidor (Simulação de 1ª ordem com inércia simples)
        # Assumindo que o drone ajusta sua velocidade para u_i
        vel_followers[i] = u_i
        pos_followers[i] += vel_followers[i] * dt
        
    # Salvar histórico
    traj_hist[k+1, 0, :] = pos_leader
    traj_hist[k+1, 1:, :] = pos_followers

# --------------------------
# 6. Configuração da Animação
# --------------------------
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-15, 15)
ax.set_ylim(-15, 15)
ax.set_title("Simulação Leader-Follower (Baseado em [cite: 108])")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.grid(True, linestyle='--', alpha=0.6)

# Elementos gráficos
# Líder: Marcador Vermelho maior
leader_dot, = ax.plot([], [], 'r^', markersize=10, label='Líder')
leader_trail, = ax.plot([], [], 'r--', lw=1, alpha=0.5)

# Seguidores: Marcadores Azuis
follower_dots = [ax.plot([], [], 'bo', markersize=6)[0] for _ in range(N_followers)]
follower_trails = [ax.plot([], [], 'b-', lw=1, alpha=0.3)[0] for _ in range(N_followers)]

# Linhas conectando Líder aos Seguidores (Visualizar a estrutura rígida virtual)
lines_formation = [ax.plot([], [], 'k-', lw=0.5, alpha=0.3)[0] for _ in range(N_followers)]

# Legenda (apenas para o primeiro handle de cada tipo para não duplicar)
ax.legend([leader_dot, follower_dots[0]], ['Líder', 'Seguidores'])

def init():
    leader_dot.set_data([], [])
    leader_trail.set_data([], [])
    for fd, ft, fl in zip(follower_dots, follower_trails, lines_formation):
        fd.set_data([], [])
        ft.set_data([], [])
        fl.set_data([], [])
    return [leader_dot, leader_trail] + follower_dots + follower_trails + lines_formation

def update(frame):
    # Recupera posições do histórico
    curr_leader = traj_hist[frame, 0]
    curr_followers = traj_hist[frame, 1:]
    
    # Atualiza Líder
    leader_dot.set_data([curr_leader[0]], [curr_leader[1]])
    
    # Rastro do Líder (últimos 50 pontos para não poluir)
    start_trail = max(0, frame - 50)
    leader_trail.set_data(traj_hist[start_trail:frame, 0, 0], traj_hist[start_trail:frame, 0, 1])
    
    artists = [leader_dot, leader_trail]
    
    # Atualiza Seguidores
    for i in range(N_followers):
        # Ponto
        follower_dots[i].set_data([curr_followers[i, 0]], [curr_followers[i, 1]])
        
        # Rastro
        follower_trails[i].set_data(traj_hist[start_trail:frame, i+1, 0], traj_hist[start_trail:frame, i+1, 1])
        
        # Linha de Formação (Líder -> Seguidor)
        lines_formation[i].set_data([curr_leader[0], curr_followers[i, 0]], 
                                    [curr_leader[1], curr_followers[i, 1]])
        
        artists.append(follower_dots[i])
        artists.append(follower_trails[i])
        artists.append(lines_formation[i])
        
    return artists

# Criar animação
ani = animation.FuncAnimation(
    fig, update, frames=range(0, steps, 2), # Pula frames para renderizar mais rápido
    init_func=init, interval=30, blit=True
)

# Salvar vídeo
print("Salvando animação 'leader_follower.mp4'...")
ani.save("leader_follower.mp4", writer='ffmpeg', fps=30)
print("Vídeo salvo com sucesso.")

# --------------------------
# 7. Análise de Erro Final
# --------------------------
# Cálculo do erro de formação no último passo
final_time = (steps-1)*dt
p_L_final, _, psi_L_final = get_leader_state(final_time)
R_L_final = rotation_matrix(psi_L_final)

results_data = []
for i in range(N_followers):
    d_i = offsets_local[i]
    p_desired = p_L_final + R_L_final @ d_i
    p_actual = traj_hist[-1, i+1, :]
    error_norm = np.linalg.norm(p_desired - p_actual)
    
    results_data.append({
        "drone_id": i+1,
        "offset_x": d_i[0],
        "offset_y": d_i[1],
        "final_error": error_norm
    })

df_res = pd.DataFrame(results_data)
print("\nResultados Finais (Erro de Formação):")
print(df_res)
df_res.to_csv("leader_follower_results.csv", index=False)

plt.show()