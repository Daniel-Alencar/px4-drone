#!/bin/bash

# Mate simulações antigas para limpar a memória
cd ~/PX4-Autopilot

pkill -x px4
pkill gzclient
pkill gzserver

# Configurações iniciais
num_vehicles=10
# Modelo do drone (x500 é padrão para Gazebo Garden/Harmonic)
model="gz_x500" 

# Mundo do Gazebo desejado
world="wind_obstacles_world"

# --- CONFIGURAÇÕES DO GRID ---
cols=4       # Número de drones por linha
spacing=2    # Distância em metros entre os drones

# Loop para iniciar cada drone
for ((i=0; i<num_vehicles; i++)); do
    
    # Define portas baseadas no ID da instância (0, 1, 2, 3, 4)
    mavsdk_port=$((14540 + i))
    qgc_port=$((14550 + i))
    sim_port=$((4560 + i))

    # --- CÁLCULO DA POSIÇÃO NO GRID ---
    # Linha atual: divisão inteira de i pelo número de colunas
    row=$((i / cols))
    
    # Coluna atual: resto da divisão de i pelo número de colunas
    col=$((i % cols))

    # Posição X (Frente/Trás) baseada na linha
    # Multiplicamos por -1 para que as novas linhas nasçam atrás da primeira,
    # e não na frente (onde já estariam os primeiros drones).
    x_pos=$((row * spacing * -1))
    
    # Posição Y (Esquerda/Direita) baseada na coluna
    y_pos=$((col * spacing)) 
    
    # Monta a pose final (Z e ângulos ficam em 0 por padrão se omitidos, ou pode colocar "X,Y,0,0,0,0")
    pose="$x_pos,$y_pos"

    echo "Iniciando Drone $i | MAVSDK: $mavsdk_port | Pose X,Y: $pose"

    # Comando de inicialização (em background usando &)
    PX4_SYS_AUTOSTART=4001 \
    PX4_GZ_WORLD=$world \
    PX4_SIM_MODEL=$model \
    PX4_GZ_MODEL_POSE="$pose" \
    PX4_SIM_HOST_ADDR=127.0.0.1 \
    PX4_SIM_PORT=$sim_port \
    PX4_SIM_QGC_PORT=$qgc_port \
    PX4_SIM_MAVSDK_PORT=$mavsdk_port \
    ./build/px4_sitl_default/bin/px4 -i $i &

    # Espera um pouco entre lançamentos para não travar o PC
    if [ "$i" -eq 0 ]; then
        sleep 20
    else
        sleep 5
    fi
done

echo "Enxame inicializado! Abra o QGroundControl para visualizar."
wait