import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd

# --------------------------
# 1. Parâmetros da Simulação
# --------------------------
dt = 0.1
steps = 800
N_drones = 10  # Enxame com 10 drones

# Parâmetros do Modelo Behavior-Based (Pesos - Eq. 10)
W_sep = 1.5   # Peso da Separação (Evitar colisão é prioridade)
W_coh = 0.8   # Peso da Coesão (Manter grupo unido)
W_ali = 1.0   # Peso do Alinhamento (Voar suavemente)
W_goal = 0.5  # Peso do Objetivo (Ir para o destino)

# Raio de Percepção (Vizinhança)
# O texto menciona "vizinhos mais próximos" (Eq. 11)
sensing_radius = 4.0 

# Limites Físicos
max_speed = 2.5
max_force = 5.0 # Limita a aceleração brusca

# --------------------------
# 2. Inicialização
# --------------------------
np.random.seed(50)

# Posições iniciais aleatórias (espalhadas)
pos = np.random.uniform(0, 5, (N_drones, 2))
# Velocidades iniciais aleatórias
vel = np.random.uniform(-1, 1, (N_drones, 2))

# Objetivo (Goal) - Vamos fazê-lo se mover para o enxame perseguir
goal_pos = np.array([10.0, 10.0])

# Histórico
traj_hist = np.zeros((steps, N_drones, 2))
goal_hist = np.zeros((steps, 2))

# --------------------------
# 3. Funções de Força (Eq. 11)
# --------------------------

def get_neighbors(agent_idx, all_pos, radius):
    """Retorna índices dos vizinhos dentro do raio de percepção."""
    distances = np.linalg.norm(all_pos - all_pos[agent_idx], axis=1)
    # Vizinhos são aqueles dentro do raio, excluindo o próprio agente (dist > 0)
    return np.where((distances < radius) & (distances > 0))[0]

def compute_forces(p_i, v_i, neighbors_idx, all_pos, all_vel, goal):
    """Calcula os 4 vetores de comportamento para o agente i."""
    
    # 1. Busca por Objetivo (Goal Seeking) [cite: 176]
    # f_goal = p_goal - p_i
    f_goal = goal - p_i
    
    # Se não houver vizinhos, o drone só segue o objetivo
    if len(neighbors_idx) == 0:
        return np.zeros(2), np.zeros(2), np.zeros(2), f_goal

    # Dados dos vizinhos
    pos_neighbors = all_pos[neighbors_idx]
    vel_neighbors = all_vel[neighbors_idx]
    
    # 2. Separação (Separation) [cite: 175]
    # Soma de (p_i - p_j) / ||p_i - p_j||^2
    sep_vec = np.zeros(2)
    for p_j in pos_neighbors:
        diff = p_i - p_j
        dist = np.linalg.norm(diff)
        if dist > 0:
            # Ponderado pelo inverso do quadrado da distância (repulsão forte se perto)
            sep_vec += (diff / dist) / (dist**2) # normaliza e divide por dist
            
    # 3. Coesão (Cohesion) [cite: 175]
    # Média das posições dos vizinhos - posição atual
    # f_coh = (Sum p_j / N) - p_i
    center_mass = np.mean(pos_neighbors, axis=0)
    coh_vec = center_mass - p_i
    
    # 4. Alinhamento (Alignment) [cite: 175]
    # Média das velocidades dos vizinhos - velocidade atual
    avg_vel = np.mean(vel_neighbors, axis=0)
    ali_vec = avg_vel - v_i
    
    return sep_vec, coh_vec, ali_vec, f_goal

def limit_vector(vec, max_val):
    """Limita a magnitude de um vetor."""
    mag = np.linalg.norm(vec)
    if mag > max_val:
        return vec * (max_val / mag)
    return vec

# --------------------------
# 4. Loop de Simulação
# --------------------------
print("Iniciando simulação Behavior-Based...")

for k in range(steps):
    # Atualiza posição do Objetivo (movimento circular lento para o enxame seguir)
    angle = 0.02 * k
    goal_pos = np.array([10 + 5*np.cos(angle), 10 + 5*np.sin(angle)])
    goal_hist[k] = goal_pos
    
    # Arrays temporários para armazenar as acelerações
    # (Importante calcular tudo antes de atualizar para garantir sincronia)
    all_accelerations = np.zeros((N_drones, 2))
    
    for i in range(N_drones):
        # Identificar vizinhos locais
        neighbors = get_neighbors(i, pos, sensing_radius)
        
        # Calcular componentes (Eq. 11)
        f_s, f_c, f_a, f_g = compute_forces(pos[i], vel[i], neighbors, pos, vel, goal_pos)
        
        # Normalizar vetores (opcional, mas ajuda na estabilidade numérica)
        # Aqui aplicamos os pesos diretamente (Eq. 10)
        u_total = (W_sep * f_s) + (W_coh * f_c) + (W_ali * f_a) + (W_goal * f_g)
        
        # Limitar a força de controle (aceleração máxima)
        u_total = limit_vector(u_total, max_force)
        
        all_accelerations[i] = u_total

    # Atualização da Física (Eq. 12)
    # v = v + u*dt
    vel += all_accelerations * dt
    
    # Limitar velocidade máxima
    for i in range(N_drones):
        vel[i] = limit_vector(vel[i], max_speed)
        
    # p = p + v*dt
    pos += vel * dt
    
    # Salvar histórico
    traj_hist[k] = pos

# --------------------------
# 5. Visualização
# --------------------------
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(0, 20)
ax.set_ylim(0, 20)
ax.set_title("Simulação Behavior-Based (Swarm Intelligence)")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.grid(True, linestyle='--', alpha=0.5)

# Elementos Gráficos
goal_marker, = ax.plot([], [], 'r*', markersize=15, label='Objetivo')
drone_dots = [ax.plot([], [], 'bo', markersize=5)[0] for _ in range(N_drones)]
# Adicionar "cauda" para ver a direção
drone_trails = [ax.plot([], [], 'b-', lw=0.5, alpha=0.3)[0] for _ in range(N_drones)]

# Círculo de percepção para o Drone 0 (apenas visualização didática)
sensor_circle = plt.Circle((0,0), sensing_radius, color='g', fill=False, alpha=0.3, linestyle='--')
ax.add_patch(sensor_circle)

ax.legend(loc='upper right')

def init():
    goal_marker.set_data([], [])
    sensor_circle.center = (-10, -10) # Esconde fora da tela
    for dot, trail in zip(drone_dots, drone_trails):
        dot.set_data([], [])
        trail.set_data([], [])
    return [goal_marker, sensor_circle] + drone_dots + drone_trails

def update(frame):
    # Atualiza Objetivo
    gx, gy = goal_hist[frame]
    goal_marker.set_data([gx], [gy])
    
    # Atualiza Drones
    curr_pos = traj_hist[frame]
    
    # Atualiza círculo de sensor do Drone 0
    sensor_circle.center = (curr_pos[0, 0], curr_pos[0, 1])
    
    trail_len = 20
    start = max(0, frame - trail_len)
    
    for i in range(N_drones):
        # Ponto atual
        drone_dots[i].set_data([curr_pos[i, 0]], [curr_pos[i, 1]])
        # Rastro
        drone_trails[i].set_data(traj_hist[start:frame, i, 0], traj_hist[start:frame, i, 1])
        
    return [goal_marker, sensor_circle] + drone_dots + drone_trails

# Criar Animação
ani = animation.FuncAnimation(
    fig, update, frames=range(0, steps, 2),
    init_func=init, interval=30, blit=True
)

# Salvar
print("Salvando animação 'behavior_based.mp4'...")
ani.save("behavior_based.mp4", writer='ffmpeg', fps=30)
print("Vídeo salvo com sucesso.")

# --------------------------
# 6. Métricas Finais
# --------------------------
# Calcular a "coesão média" (distância média entre todos os drones) no final
final_pos = traj_hist[-1]
distances = []
for i in range(N_drones):
    for j in range(i+1, N_drones):
        distances.append(np.linalg.norm(final_pos[i] - final_pos[j]))

avg_separation = np.mean(distances)
print(f"\n--- Resultados Finais ---")
print(f"Separação média entre drones: {avg_separation:.3f} m")

# Salvar dados
df = pd.DataFrame(final_pos, columns=['x', 'y'])
df['drone_id'] = range(N_drones)
df.to_csv("behavior_based_results.csv", index=False)