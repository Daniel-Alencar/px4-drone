import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd

# --------------------------
# 1. Parâmetros da Simulação
# --------------------------
dt = 0.05  # Passo de tempo (s)
steps = 800
N_drones = 4  # Número de drones na estrutura

# Ganhos do controlador PD (Eq. 7 do texto)
Kp = 2.0  # Ganho proporcional (focando em manter a posição rígida)
Kd = 1.5  # Ganho derivativo (amortecimento para evitar oscilação)

# Limites físicos
max_speed = 4.0  # Velocidade máxima dos drones

# --------------------------
# 2. Definição da Estrutura Virtual (Geometria)
# --------------------------
# A "Estrutura Virtual" será um quadrado girando e se movendo.
# Posições relativas fixas (r_i^{VS}) em relação ao centro virtual (0,0)
# Formato: [x, y]
structure_offsets = np.array([
    [ 2.0,  2.0],  # Drone 1: Frente-Esquerda
    [ 2.0, -2.0],  # Drone 2: Frente-Direita
    [-2.0, -2.0],  # Drone 3: Trás-Direita
    [-2.0,  2.0]   # Drone 4: Trás-Esquerda
])

# --------------------------
# 3. Inicialização de Estados
# --------------------------
# Estado da Estrutura Virtual (p_v, v_v, theta_v, omega_v)
# Inicialmente no centro
pv = np.array([0.0, 0.0])
vv = np.array([0.0, 0.0])
theta_v = 0.0      # Orientação inicial
omega_v = 0.0      # Velocidade angular inicial

# Estados Físicos dos Drones (p_i, v_i)
# Iniciam "bagunçados" para mostrar a convergência para a estrutura
np.random.seed(42)
pos_drones = np.random.uniform(-5, 5, (N_drones, 2)) 
vel_drones = np.zeros((N_drones, 2))

# Histórico para animação e análise
# [step, id_drone (0=Virtual, 1..N=Drones), x/y]
traj_hist = np.zeros((steps, N_drones + 1, 2))

# --------------------------
# 4. Trajetória da Estrutura Virtual
# --------------------------
def update_virtual_structure_state(t):
    """
    Define o movimento do 'corpo virtual'.
    Trajetória: Uma figura '8' (Lemniscata de Bernoulli) com rotação.
    """
    a = 6.0  # Largura da figura
    scale_speed = 0.4
    
    # Posição (p_v)
    x = a * np.sin(scale_speed * t)
    y = a * np.sin(scale_speed * t) * np.cos(scale_speed * t)
    pv_new = np.array([x, y])
    
    # Velocidade Linear (v_v) - Derivada temporal
    vx = a * scale_speed * np.cos(scale_speed * t)
    vy = a * scale_speed * (np.cos(scale_speed * t)**2 - np.sin(scale_speed * t)**2)
    vv_new = np.array([vx, vy])
    
    # Orientação (theta_v) - Vamos fazer a estrutura girar lentamente
    theta_new = 0.2 * t 
    
    # Velocidade Angular (omega_v)
    omega_new = 0.2  # rad/s constante
    
    return pv_new, vv_new, theta_new, omega_new

def rotation_matrix_2d(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])

# --------------------------
# 5. Loop de Simulação
# --------------------------
print("Iniciando simulação Virtual Structure...")

for k in range(steps):
    t = k * dt
    
    # A. Atualizar Estado da Estrutura Virtual (Referência Global)
    pv, vv, theta_v, omega_v = update_virtual_structure_state(t)
    Rv = rotation_matrix_2d(theta_v)
    
    # Salvar histórico da estrutura virtual (índice 0)
    traj_hist[k, 0, :] = pv
    
    # B. Controle dos Drones (Seguir a estrutura)
    for i in range(N_drones):
        # Posição relativa fixa deste drone
        ri_vs = structure_offsets[i]
        
        # 1. Calcular Posição Desejada (Eq. 5)
        # p_i^d = p_v + R_v * r_i^{VS}
        pos_relative_global = Rv @ ri_vs
        p_desired = pv + pos_relative_global
        
        # 2. Calcular Velocidade Desejada (Eq. 6)
        # v_i^d = v_v + omega_v x (R_v * r_i^{VS})
        # Nota: Produto vetorial 2D (omega k x r) = [-omega*ry, omega*rx]
        # omega_v é escalar em 2D
        cross_prod = np.array([-omega_v * pos_relative_global[1], 
                                omega_v * pos_relative_global[0]])
        v_desired = vv + cross_prod
        
        # 3. Lei de Controle (Eq. 7 e 8)
        # u_i = Kp(p_d - p_i) + Kd(v_d - v_i)
        
        # Erros
        e_p = p_desired - pos_drones[i]
        e_v = v_desired - vel_drones[i]
        
        # Comando de aceleração (u_i)
        u_i = Kp * e_p + Kd * e_v
        
        # 4. Atualizar Dinâmica do Drone (Euler Integration)
        # v_new = v_old + u * dt
        vel_drones[i] += u_i * dt
        
        # Saturação de velocidade
        speed = np.linalg.norm(vel_drones[i])
        if speed > max_speed:
            vel_drones[i] = vel_drones[i] * (max_speed / speed)
            
        # p_new = p_old + v * dt
        pos_drones[i] += vel_drones[i] * dt
        
        # Salvar histórico
        traj_hist[k, i+1, :] = pos_drones[i]

# --------------------------
# 6. Configuração da Animação
# --------------------------
fig, ax = plt.subplots(figsize=(8, 8))
limit = 10
ax.set_xlim(-limit, limit)
ax.set_ylim(-limit, limit)
ax.set_title("Simulação Virtual Structure (Square Formation)")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.grid(True, linestyle='--', alpha=0.5)

# Elementos gráficos
# Centro Virtual (apenas referência visual, não é um drone real)
virtual_center, = ax.plot([], [], 'kx', markersize=8, label='Centro Virtual', alpha=0.5)
virtual_trail, = ax.plot([], [], 'k:', lw=1, alpha=0.3)

# Drones Reais
drone_dots = [ax.plot([], [], 'o', markersize=8)[0] for _ in range(N_drones)]
drone_trails = [ax.plot([], [], '-', lw=1, alpha=0.5)[0] for _ in range(N_drones)]

# Linhas conectando os drones (para visualizar a rigidez da forma)
# Vamos desenhar um polígono fechado conectando os drones
formation_lines, = ax.plot([], [], 'r-', lw=1.5, alpha=0.6, label='Estrutura Rígida')

ax.legend(loc='upper right')

def init():
    virtual_center.set_data([], [])
    virtual_trail.set_data([], [])
    formation_lines.set_data([], [])
    for dot, trail in zip(drone_dots, drone_trails):
        dot.set_data([], [])
        trail.set_data([], [])
    return [virtual_center, virtual_trail, formation_lines] + drone_dots + drone_trails

def update(frame):
    # Recupera estado atual
    curr_virtual = traj_hist[frame, 0]
    curr_drones = traj_hist[frame, 1:]
    
    # 1. Atualiza Centro Virtual
    virtual_center.set_data([curr_virtual[0]], [curr_virtual[1]])
    
    # Rastro do centro
    start = max(0, frame - 100)
    virtual_trail.set_data(traj_hist[start:frame, 0, 0], traj_hist[start:frame, 0, 1])
    
    # 2. Atualiza Drones
    x_drones = []
    y_drones = []
    
    for i in range(N_drones):
        # Ponto
        drone_dots[i].set_data([curr_drones[i, 0]], [curr_drones[i, 1]])
        
        # Rastro individual
        drone_trails[i].set_data(traj_hist[start:frame, i+1, 0], traj_hist[start:frame, i+1, 1])
        
        # Coleta para desenhar o polígono
        x_drones.append(curr_drones[i, 0])
        y_drones.append(curr_drones[i, 1])
    
    # 3. Desenha as linhas da estrutura (quadrado fechado)
    # Adiciona o primeiro ponto ao final para fechar o ciclo
    x_poly = x_drones + [x_drones[0]]
    y_poly = y_drones + [y_drones[0]]
    formation_lines.set_data(x_poly, y_poly)
    
    return [virtual_center, virtual_trail, formation_lines] + drone_dots + drone_trails

# Criar animação
ani = animation.FuncAnimation(
    fig, update, frames=range(0, steps, 2), # Pula frames para suavidade/performance
    init_func=init, interval=30, blit=True
)

# Salvar vídeo (Requer FFmpeg instalado)
print("Salvando animação 'virtual_structure.mp4'...")
ani.save("virtual_structure.mp4", writer='ffmpeg', fps=30)
print("Vídeo salvo com sucesso.")

# --------------------------
# 7. Análise de Erro de Formação
# --------------------------
# O erro aqui é: quão longe o drone real está da sua posição virtual ideal?
final_step = steps - 1
pv_f, vv_f, theta_f, omega_f = update_virtual_structure_state(final_step*dt)
Rv_f = rotation_matrix_2d(theta_f)

errors = []
print("\n--- Resultados Finais ---")
for i in range(N_drones):
    # Posição ideal onde o drone DEVERIA estar
    p_ideal = pv_f + Rv_f @ structure_offsets[i]
    # Posição real
    p_real = traj_hist[final_step, i+1, :]
    
    error = np.linalg.norm(p_ideal - p_real)
    errors.append(error)
    print(f"Drone {i+1}: Erro de posição = {error:.4f} m")

# Salvar dados em CSV
df = pd.DataFrame({
    'drone_id': range(1, N_drones+1),
    'final_error': errors,
    'offset_x': structure_offsets[:,0],
    'offset_y': structure_offsets[:,1]
})
df.to_csv("virtual_structure_results.csv", index=False)

plt.show()