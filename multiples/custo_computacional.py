import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ---------------------------------------------------------
# CONFIGURAÇÕES DO PROJETO
# ---------------------------------------------------------
# Lista exata dos nomes das pastas dos algoritmos
ALGORITMOS = [
    "APF", 
    "Behavior-Based", 
    "Consensus-Based", 
    "Hybrid-VS-APF", 
    "Leader-Follower", 
    "Virtual-Structure"
]

# Dicionário mapeando o número da pasta para o nome do mundo (Para os títulos dos gráficos)
MUNDOS = {
    "1": "Mundo 1 (Com Obstáculos, Sem Vento)",
    "2": "Mundo 2 (Com Obstáculos e Com Vento)",
    "3": "Mundo 3 (Sem Obstáculos, Sem Vento)",
    "4": "Mundo 4 (Sem Obstáculos, Com Vento)"
}

# Pasta atual (multiples/)
BASE_DIR = os.getcwd()

# ---------------------------------------------------------
# FUNÇÃO PARA PLOTAR O GRÁFICO DE UM MUNDO
# ---------------------------------------------------------
def gerar_graficos_mundo(df_consolidado, id_mundo, nome_mundo):
    # Tira a média caso você tenha rodado a simulação mais de uma vez no mesmo mundo
    df_agrupado = df_consolidado.groupby('Algoritmo').mean().reset_index()
    
    # Ordena pelo tempo de cálculo para o gráfico ficar com uma "escadinha" bonita
    df_agrupado = df_agrupado.sort_values(by='Media_Calc_ms')

    fig, axs = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(f'Custo Computacional - {nome_mundo}', fontsize=16, fontweight='bold')

    algoritmos_presentes = df_agrupado['Algoritmo']
    cores = plt.cm.viridis(np.linspace(0.1, 0.9, len(algoritmos_presentes)))

    # --- Gráfico 1: Tempo Médio de Cálculo ---
    axs[0, 0].bar(algoritmos_presentes, df_agrupado['Media_Calc_ms'], color=cores)
    axs[0, 0].set_title('Tempo Médio de Processamento', fontweight='bold')
    axs[0, 0].set_ylabel('Milissegundos (ms)')
    axs[0, 0].tick_params(axis='x', rotation=30)
    axs[0, 0].grid(axis='y', linestyle='--', alpha=0.7)

    # --- Gráfico 2: Pico Máximo (Gargalo) ---
    axs[0, 1].bar(algoritmos_presentes, df_agrupado['Pico_Calc_ms'], color=cores)
    axs[0, 1].set_title('Pico Máximo de Processamento', fontweight='bold')
    axs[0, 1].set_ylabel('Milissegundos (ms)')
    axs[0, 1].tick_params(axis='x', rotation=30)
    axs[0, 1].grid(axis='y', linestyle='--', alpha=0.7)

    # --- Gráfico 3: Uso de CPU ---
    axs[1, 0].bar(algoritmos_presentes, df_agrupado['Media_CPU_Perc'], color=cores)
    axs[1, 0].set_title('Consumo Médio de CPU', fontweight='bold')
    axs[1, 0].set_ylabel('Uso de CPU (%)')
    axs[1, 0].tick_params(axis='x', rotation=30)
    axs[1, 0].grid(axis='y', linestyle='--', alpha=0.7)

    # --- Gráfico 4: Consumo de RAM ---
    min_ram = max(0, df_agrupado['Pico_RAM_MB'].min() - 5) # Dá um zoom visual na diferença
    axs[1, 1].bar(algoritmos_presentes, df_agrupado['Pico_RAM_MB'] - min_ram, bottom=min_ram, color=cores)
    axs[1, 1].set_title('Consumo de Memória RAM (Pico)', fontweight='bold')
    axs[1, 1].set_ylabel('Megabytes (MB)')
    axs[1, 1].tick_params(axis='x', rotation=30)
    axs[1, 1].grid(axis='y', linestyle='--', alpha=0.7)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Salva a imagem
    nome_imagem = f'comparativo_{id_mundo}.png'
    plt.savefig(nome_imagem, dpi=300, bbox_inches='tight')
    print(f"📊 Gráfico gerado: {nome_imagem}")
    plt.close()

# ---------------------------------------------------------
# PROCESSO PRINCIPAL DE VARREDURA E CONSOLIDAÇÃO
# ---------------------------------------------------------
print(f"🔍 Iniciando varredura na pasta: {BASE_DIR}\n")

# Para cada um dos 4 mundos...
for id_mundo, desc_mundo in MUNDOS.items():
    lista_dfs_mundo = []
    
    # Entra na pasta de cada algoritmo...
    for alg in ALGORITMOS:
        caminho_arquivo = os.path.join(BASE_DIR, alg, id_mundo, "comparativo_computacional.csv")
        
        # Se a pasta/arquivo existir, nós lemos
        if os.path.isfile(caminho_arquivo):
            try:
                df_temp = pd.read_csv(caminho_arquivo)
                lista_dfs_mundo.append(df_temp)
                print(f"✅ Lido: {alg} -> Mundo {id_mundo}")
            except Exception as e:
                print(f"❌ Erro ao ler {caminho_arquivo}: {e}")
        else:
            # Silencioso, apenas pula se a simulação ainda não foi feita
            pass
            
    # Se encontrou dados de pelo menos um algoritmo neste mundo, consolida e gera gráfico
    if lista_dfs_mundo:
        df_consolidado = pd.concat(lista_dfs_mundo, ignore_index=True)
        
        # Salva o arquivo CSV mestre do Mundo
        nome_csv_mestre = f"custo_computacional_{id_mundo}.csv"
        df_consolidado.to_csv(nome_csv_mestre, index=False)
        print(f"💾 Arquivo salvo: {nome_csv_mestre}")
        
        # Gera o gráfico
        gerar_graficos_mundo(df_consolidado, id_mundo, desc_mundo)
        print("-" * 50)
    else:
        print(f"⚠️ Nenhum dado encontrado para o Mundo {id_mundo}.")
        print("-" * 50)

print("\n🚀 Análise Global Concluída com Sucesso!")