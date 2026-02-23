import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd

# --------------------------
# 1. Parâmetros da Simulação
# --------------------------
dt = 0.05
steps = 800
N_drones = 6

# Parâmetros dos Campos de Potencial (Ganhos)
k_att = 2.0        # Atração ao objetivo
k_rep_obs = 15.0   # Repulsão de obstáculos (paredes)
k_rep_agent = 5.0  # Repulsão entre drones (evitar colisão)

# Distâncias de Influência (d0)
d0_obs = 2.0       # Distância para começar a desviar de obstáculos
d0_agent = 1.5     # Distância para começar a desviar de outros drones

max_speed = 3.0

# --------------------------
# 2. Configuração do Cenário
# --------------------------
# Objetivo (Goal)
goal_pos = np.array([12.0, 12.0])

# Obstáculos (Círculos: [x, y, raio])
obstacles = [
    {'pos': np.array([6.0, 6.0]), 'radius': 1.5},
    {'pos': np.array([4.0, 10.0]), 'radius': 1.0},
    {'pos': np.array([10.0, 4.0]), 'radius': 1.2},
    {'pos': np.array([8.0, 8.0]), 'radius': 0.8}
]

# --------------------------
# 3. Inicialização
# --------------------------
np.random.seed(42)
# Drones iniciam no canto inferior esquerdo (0 a 3)
pos = np.random.uniform(0, 3, (N_drones, 2))
vel = np.zeros((N_drones, 2))

traj_hist = np.zeros((steps, N_drones, 2))

# --------------------------
# 4. Funções de Força (Eq. 26 e 254)
# --------------------------

def get_att_force(p_curr, p_goal, k):
    """
    Força atrativa (Eq. 26, parte atrativa simplificada como proporcional ao erro).
    F_att = -grad(U_att) = -k * (p - p_goal)
    """
    return -k * (p_curr - p_goal)

def get_rep_force(p_curr, p_obstacle, radius_obs, d0, k):
    """
    Força repulsiva baseada no gradiente do potencial inverso (Eq. 254).
    """
    # Vetor do obstáculo para o drone
    vec_diff = p_curr - p_obstacle
    dist_center = np.linalg.norm(vec_diff)
    
    # Distância real até a SUPERFÍCIE do obstáculo
    dist_surface = dist_center - radius_obs
    
    # Se estiver fora da distância de influência (d0), força é zero
    if dist_surface > d0 or dist_surface <= 0:
        return np.zeros(2)
    
    # Cálculo do Gradiente (Repulsão)
    # A fórmula clássica derivada de U_rep = 0.5*k*(1/d - 1/d0)^2 é:
    # F_rep = k * (1/d - 1/d0) * (1/d^2) * (gradiente_distancia)
    
    term1 = (1.0 / dist_surface) - (1.0 / d0)
    term2 = 1.0 / (dist_surface**2)
    
    # Vetor unitário apontando para fora do obstáculo
    unit_vec = vec_diff / dist_center
    
    force_mag = k * term1 * term2
    return force_mag * unit_vec

# --------------------------
# 5. Loop de Simulação
# --------------------------
print("Iniciando simulação APF...")

for k in range(steps):
    
    # Calcular forças para cada drone
    forces = np.zeros((N_drones, 2))
    
    for i in range(N_drones):
        p_i = pos[i]
        
        # 1. Atração ao Objetivo
        f_att = get_att_force(p_i, goal_pos, k_att)
        
        # 2. Repulsão de Obstáculos Estáticos
        f_rep_obs_total = np.zeros(2)
        for obs in obstacles:
            f_rep = get_rep_force(p_i, obs['pos'], obs['radius'], d0_obs, k_rep_obs)
            f_rep_obs_total += f_rep
            
        # 3. Repulsão de Outros Agentes (Evitar colisão mútua)
        f_rep_agent_total = np.zeros(2)
        for j in range(N_drones):
            if i != j:
                # Tratamos o outro drone como um obstáculo pequeno (raio 0.3)
                f_rep = get_rep_force(p_i, pos[j], 0.3, d0_agent, k_rep_agent)
                f_rep_agent_total += f_rep
        
        # Soma Total (Eq. 26)
        forces[i] = f_att + f_rep_obs_total + f_rep_agent_total
    
    # Atualizar Física
    # v_new = v + F*dt (Segunda Lei de Newton simples / Dinâmica de ponto)
    vel += forces * dt
    
    # Limitar velocidade
    speeds = np.linalg.norm(vel, axis=1)
    for i in range(N_drones):
        if speeds[i] > max_speed:
            vel[i] = vel[i] / speeds[i] * max_speed
            
    pos += vel * dt
    traj_hist[k] = pos

# --------------------------
# 6. Visualização
# --------------------------
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-1, 15)
ax.set_ylim(-1, 15)
ax.set_title("Artificial Potential Fields (APF) - Obstacle Avoidance")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.grid(True, linestyle='--', alpha=0.5)

# Desenhar Obstáculos
for obs in obstacles:
    circle = plt.Circle(obs['pos'], obs['radius'], color='k', alpha=0.4)
    ax.add_patch(circle)
    # Círculo de influência (d0) tracejado
    circle_d0 = plt.Circle(obs['pos'], obs['radius'] + d0_obs, color='k', fill=False, linestyle=':', alpha=0.2)
    ax.add_patch(circle_d0)

# Desenhar Objetivo
goal_marker, = ax.plot([goal_pos[0]], [goal_pos[1]], 'r*', markersize=15, label='Objetivo')

# Drones e Rastros
drone_dots = [ax.plot([], [], 'bo', markersize=6)[0] for _ in range(N_drones)]
drone_trails = [ax.plot([], [], 'b-', lw=1, alpha=0.3)[0] for _ in range(N_drones)]

ax.legend(loc='upper left')

def init():
    for dot, trail in zip(drone_dots, drone_trails):
        dot.set_data([], [])
        trail.set_data([], [])
    return drone_dots + drone_trails + [goal_marker]

def update(frame):
    curr_pos = traj_hist[frame]
    
    trail_len = 50
    start = max(0, frame - trail_len)
    
    for i in range(N_drones):
        # Ponto
        drone_dots[i].set_data([curr_pos[i, 0]], [curr_pos[i, 1]])
        # Rastro
        drone_trails[i].set_data(traj_hist[start:frame, i, 0], traj_hist[start:frame, i, 1])
        
    return drone_dots + drone_trails + [goal_marker]

# Criar Animação
ani = animation.FuncAnimation(
    fig, update, frames=range(0, steps, 2),
    init_func=init, interval=30, blit=True
)

print("Salvando animação 'apf_simulation.mp4'...")
ani.save("apf_simulation.mp4", writer='ffmpeg', fps=30)
print("Vídeo salvo com sucesso.")

# Análise de Distância ao Alvo
final_pos = traj_hist[-1]
dist_to_goal = np.linalg.norm(final_pos - goal_pos, axis=1)
df = pd.DataFrame({
    'drone_id': range(N_drones),
    'final_dist_goal': dist_to_goal
})
print("\n--- Resultados Finais ---")
print(df)
df.to_csv("apf_results.csv", index=False)