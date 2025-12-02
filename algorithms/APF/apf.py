import numpy as np

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd




# --------------------------
# Parâmetros da simulação
# --------------------------
np.random.seed(1)
N = 6
dt = 0.1
steps = 800
k_attr = 1.2
k_rep_obs = 8.0
k_rep_agent = 5.0
d0_obs = 1.2
d0_agent = 0.6
max_speed = 1.5

# --------------------------
# Ambiente
# --------------------------
goal = np.array([6.0, 6.0])
obstacles = [
    {"pos": np.array([3.0, 3.0]), "r": 0.8},
    {"pos": np.array([4.8, 2.2]), "r": 0.6},
    {"pos": np.array([2.0, 5.0]), "r": 0.7},
]

# Posições iniciais
positions = np.column_stack((
    np.random.uniform(0.0, 1.5, size=N),
    np.random.uniform(0.0, 1.5, size=N)
))
velocities = np.zeros_like(positions)

# Histórico de trajetórias
traj = np.zeros((steps+1, N, 2))
traj[0] = positions.copy()

# --------------------------
# Forças
# --------------------------
def attractive_force(p, goal, k=k_attr):
    return -k*(p - goal)

def repulsive_obstacle(p, obs, k=k_rep_obs, d0=d0_obs):
    pos = obs["pos"]
    r = obs["r"]
    vec = p - pos
    dist = np.linalg.norm(vec)
    if dist <= r:
        dist = r + 1e-3
    if dist > d0 + r:
        return np.zeros(2)
    eff = dist - r
    if eff <= 0:
        eff = 1e-3
    mag = k*(1.0/eff - 1.0/d0) * (1.0/(eff**2))
    return mag * (vec / (dist + 1e-12))

def repulsive_agent(p, others, k=k_rep_agent, d0=d0_agent):
    total = np.zeros(2)
    for q in others:
        vec = p - q
        dist = np.linalg.norm(vec)
        if dist == 0 or dist > d0:
            continue
        mag = k*(1.0/dist - 1.0/d0) * (1.0/(dist**2))
        total += mag*(vec/dist)
    return total

# --------------------------
# Plot setup
# --------------------------
fig, ax = plt.subplots(figsize=(7,7))
ax.set_xlim(-1,8)
ax.set_ylim(-1,8)
ax.set_title("UAV Swarm Control com APF - Animação")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.grid(True)

# Obstáculos
for obs in obstacles:
    circle = plt.Circle(tuple(obs["pos"]), obs["r"], fill=False, linewidth=1.2)
    ax.add_patch(circle)

# Objetivo
ax.scatter(goal[0], goal[1], marker='*', s=200, label='Objetivo')

# Criar artistas (pontos e trilhas)
drone_dots = [ax.plot([], [], 'o', markersize=8)[0] for _ in range(N)]
trails = [ax.plot([], [], '-', lw=1.2)[0] for _ in range(N)]

# --------------------------
# init function (necessária para blit=True)
# --------------------------
def init():
    for i in range(N):
        drone_dots[i].set_data([], [])
        trails[i].set_data([], [])
    return drone_dots + trails

# --------------------------
# update function
# --------------------------
def update(frame):
    global positions, velocities
    # Atualiza física
    for i in range(N):
        p = positions[i].copy()
        F = attractive_force(p, goal)
        for obs in obstacles:
            F += repulsive_obstacle(p, obs)
        others = np.delete(positions, i, axis=0)
        F += repulsive_agent(p, others)
        v = velocities[i] + dt*F
        speed = np.linalg.norm(v)
        if speed > max_speed:
            v = v * (max_speed/speed)
        velocities[i] = v
    positions = positions + velocities*dt
    traj[frame] = positions.copy()

    # Atualiza artistas — **usar sequências**, mesmo para um ponto
    artists = []
    for i in range(N):
        # ponto atual como sequências de um elemento
        drone_dots[i].set_data([positions[i,0]], [positions[i,1]])
        # trilha: proteja caso frame == 0
        if frame > 0:
            trails[i].set_data(traj[:frame+1,i,0], traj[:frame+1,i,1])
        else:
            trails[i].set_data([], [])
        artists.append(drone_dots[i])
        artists.append(trails[i])

    return artists

# --------------------------
# Cria animação
# --------------------------
ani = animation.FuncAnimation(
    fig, update, frames=range(1, steps+1),
    init_func=init, interval=40, blit=True, repeat=False
)
ani.save("swarm_apf.mp4", writer='ffmpeg', fps=25)
print("Vídeo salvo em swarm_apf.mp4")


plt.legend()
plt.show()

# --------------------------
# Estatísticas finais
# --------------------------
final_positions = positions
dists = np.linalg.norm(final_positions - goal, axis=1)
df = pd.DataFrame({
    "agent": np.arange(N),
    "final_x": final_positions[:,0],
    "final_y": final_positions[:,1],
    "dist_to_goal": dists
})
df.to_csv("apf_sim_results.csv", index=False)
print("Resultados salvos em apf_sim_results.csv")
print(df)
