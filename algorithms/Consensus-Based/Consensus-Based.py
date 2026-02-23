import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd

# --------------------------
# 1. Parâmetros da Simulação
# --------------------------
dt = 0.05
steps = 600
N_drones = 5  # 5 Drones em pentágono

# Ganhos de Consenso (Eq. 21)
Kp = 1.0
Kv = 1.0

# --------------------------
# 2. Definição da Topologia de Rede (Grafo)
# --------------------------
# Grafo em Anel: 0<-1, 1<-2, 2<-3, 3<-4, 4<-0
A = np.zeros((N_drones, N_drones))
for i in range(N_drones):
    neighbor = (i + 1) % N_drones
    A[i, neighbor] = 1.0

# --------------------------
# 3. Definição da Formação Desejada
# --------------------------
radius = 4.0
des_angles = np.linspace(0, 2*np.pi, N_drones, endpoint=False)
offsets = np.column_stack([radius * np.cos(des_angles), radius * np.sin(des_angles)])

# --------------------------
# 4. Inicialização
# --------------------------
np.random.seed(10)

# Drones iniciam espalhados
pos = np.random.uniform(-8, 8, (N_drones, 2))
vel = np.random.uniform(-2, 2, (N_drones, 2))

# Histórico
traj_hist = np.zeros((steps, N_drones, 2))

# --------------------------
# 5. Loop de Controle
# --------------------------
print("Iniciando simulação Consensus-Based...")

for k in range(steps):
    u_cmds = np.zeros((N_drones, 2))
    
    for i in range(N_drones):
        sum_pos = np.zeros(2)
        sum_vel = np.zeros(2)
        
        for j in range(N_drones):
            if A[i, j] > 0:
                pos_error = (pos[i] - offsets[i]) - (pos[j] - offsets[j])
                vel_error = vel[i] - vel[j]
                
                sum_pos += A[i, j] * pos_error
                sum_vel += A[i, j] * vel_error
        
        u_cmds[i] = -Kp * sum_pos - Kv * sum_vel
        
    vel += u_cmds * dt
    pos += vel * dt
    traj_hist[k] = pos

# --------------------------
# 6. Visualização
# --------------------------
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-15, 15)
ax.set_ylim(-15, 15)
ax.set_title("Consensus-Based Control (Ring Topology)")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.grid(True, linestyle='--', alpha=0.5)

# Atores
lines_adj = [] 
for i in range(N_drones):
    line, = ax.plot([], [], 'k--', lw=0.5, alpha=0.4)
    lines_adj.append(line)

drones_dot, = ax.plot([], [], 'bo', markersize=8, label='Drones')
target_shapes, = ax.plot([], [], 'rx', alpha=0.3, label='Alvos Relativos')

# --- CORREÇÃO PRINCIPAL AQUI ---
# Inicializamos o Quiver JÁ com os dados do frame 0.
# Assim o Matplotlib sabe que existem 5 setas desde o início.
initial_x = traj_hist[0, :, 0]
initial_y = traj_hist[0, :, 1]
quiver = ax.quiver(initial_x, initial_y, np.zeros(N_drones), np.zeros(N_drones), color='b', scale=20)

def init():
    # Limpa apenas os dados de linhas e pontos
    drones_dot.set_data([], [])
    target_shapes.set_data([], [])
    for line in lines_adj:
        line.set_data([], [])
    
    # Para o Quiver, apenas garantimos que a velocidade (U, V) é zero,
    # mantendo as posições originais definidas na criação.
    quiver.set_UVC(np.zeros(N_drones), np.zeros(N_drones))
    
    return [drones_dot, target_shapes, quiver] + lines_adj

def update(frame):
    curr_pos = traj_hist[frame]
    
    # Atualiza pontos
    drones_dot.set_data(curr_pos[:, 0], curr_pos[:, 1])
    
    # Atualiza setas (Quiver)
    if frame < steps - 1:
        curr_vel = (traj_hist[frame+1] - curr_pos)/dt
        # Atualiza a posição das setas
        quiver.set_offsets(curr_pos)
        # Atualiza a direção/tamanho das setas
        quiver.set_UVC(curr_vel[:, 0], curr_vel[:, 1])
    
    # Atualiza linhas da rede
    line_idx = 0
    for i in range(N_drones):
        for j in range(N_drones):
            if A[i, j] > 0:
                x_line = [curr_pos[i, 0], curr_pos[j, 0]]
                y_line = [curr_pos[i, 1], curr_pos[j, 1]]
                lines_adj[line_idx].set_data(x_line, y_line)
                line_idx += 1
                
    # Desenha forma ideal
    centroid = np.mean(curr_pos, axis=0)
    ideal_targets = centroid + offsets
    target_shapes.set_data(ideal_targets[:, 0], ideal_targets[:, 1])
    
    return [drones_dot, target_shapes, quiver] + lines_adj

ani = animation.FuncAnimation(
    fig, update, frames=range(0, steps, 4),
    init_func=init, interval=50, blit=False 
)

print("Salvando animação 'consensus_formation.mp4'...")
ani.save("consensus_formation.mp4", writer='ffmpeg', fps=30)
print("Vídeo salvo com sucesso.")

# Análise Final
final_pos = traj_hist[-1]
df = pd.DataFrame(final_pos, columns=['x', 'y'])
df.to_csv("consensus_results.csv", index=False)