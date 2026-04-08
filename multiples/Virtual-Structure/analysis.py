import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import glob
import os

# ---------------------------------------------------------
# 1. ENCONTRAR O ARQUIVO CSV MAIS RECENTE
# ---------------------------------------------------------
arquivos_csv = glob.glob('resultados*.csv')

if not arquivos_csv:
    print("❌ Nenhum arquivo CSV encontrado na pasta atual.")
    exit()

arquivo_mais_recente = max(arquivos_csv, key=os.path.getctime)
# Extrair o nome base (sem .csv) para usar nos nomes das imagens
nome_base = os.path.splitext(os.path.basename(arquivo_mais_recente))[0]
print(f"📊 Analisando os dados de: {arquivo_mais_recente}\n")

df = pd.read_csv(arquivo_mais_recente)

# ---------------------------------------------------------
# 2. RECONSTRUIR O MAPA DE OBSTÁCULOS
# ---------------------------------------------------------
RAIO_CILINDRO = 1.5
OBSTACULOS_ESTATICOS = [
    np.array([20.0, 0.0]), np.array([20.0, 7.0]), np.array([20.0, -7.0]),
    np.array([24.0, 3.5]), np.array([24.0, -3.5]), np.array([24.0, 10.5]),
    np.array([24.0, -10.5]), np.array([28.0, 0.0]), np.array([28.0, 7.0]),
    np.array([28.0, -7.0])
]

# ---------------------------------------------------------
# 3. GERAR GRÁFICO 1: TRAJETÓRIA 2D
# ---------------------------------------------------------
plt.figure(figsize=(10, 8))
ax = plt.gca()
drones = df['Drone_ID'].unique()

for drone_id in drones:
    dados_drone = df[df['Drone_ID'] == drone_id]
    plt.plot(dados_drone['Pos_E_Metros'], dados_drone['Pos_N_Metros'], label=f'Drone {drone_id}', linewidth=2)
    plt.plot(dados_drone['Pos_E_Metros'].iloc[0], dados_drone['Pos_N_Metros'].iloc[0], 'ko', markersize=5) 
    plt.plot(dados_drone['Pos_E_Metros'].iloc[-1], dados_drone['Pos_N_Metros'].iloc[-1], 'k*', markersize=8)

for obs in OBSTACULOS_ESTATICOS:
    circulo = patches.Circle((obs[1], obs[0]), RAIO_CILINDRO, color='red', alpha=0.3)
    ax.add_patch(circulo)

plt.title('Trajetória 2D do Enxame (Virtual-Structure)', fontsize=14, fontweight='bold')
plt.xlabel('Posição Leste (metros)')
plt.ylabel('Posição Norte (metros)')
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(loc='upper right', bbox_to_anchor=(1.15, 1))
plt.axis('equal')
plt.tight_layout()

# SALVAR GRÁFICO 1
nome_img_trajetoria = f"grafico_trajetoria.png"
plt.savefig(nome_img_trajetoria, dpi=300, bbox_inches='tight')
print(f"💾 Trajetória salva como: {nome_img_trajetoria}")
plt.close()

# ---------------------------------------------------------
# 4. GERAR GRÁFICO 2: MARGEM DE SEGURANÇA
# ---------------------------------------------------------
plt.figure(figsize=(10, 5))

for drone_id in drones:
    dados_drone = df[df['Drone_ID'] == drone_id]
    plt.plot(dados_drone['Tempo_s'], dados_drone['Distancia_Minima_Obstaculo_Instante_m'], label=f'Drone {drone_id}', linewidth=1.5)

plt.axhline(y=RAIO_CILINDRO, color='red', linestyle='--', linewidth=2, label='Colisão (1.5m)')
plt.title('Distância ao Obstáculo Mais Próximo', fontsize=14, fontweight='bold')
plt.xlabel('Tempo de Missão (segundos)')
plt.ylabel('Distância (metros)')
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(loc='upper right')
plt.tight_layout()

# SALVAR GRÁFICO 2
nome_img_seguranca = f"grafico_seguranca.png"
plt.savefig(nome_img_seguranca, dpi=300, bbox_inches='tight')
print(f"💾 Margem de segurança salva como: {nome_img_seguranca}")
plt.close()

print("\n✅ Processamento concluído. Imagens geradas com sucesso!")