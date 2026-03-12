#!/bin/bash

# Mate simulações antigas para limpar a memória
cd ~/PX4-Autopilot

pkill -x px4
pkill gzclient
pkill gzserver
# Dica extra: Se estiver usando o Gazebo moderno (Garden/Harmonic), adicione a linha abaixo:
pkill -f 'gz sim' 

# Configurações iniciais
num_vehicles=1
# Modelo do drone
model="gz_x500_mono_cam_down" 
# Mundo do Gazebo desejado
world="bouncing_world"

# Loop para iniciar cada drone
for ((i=0; i<num_vehicles; i++)); do
    
    # Define portas baseadas no ID da instância (0, 1, 2, 3, 4)
    mavsdk_port=$((14540 + i))
    qgc_port=$((14550 + i))
    sim_port=$((4560 + i))

    # Posição inicial (Y varia para ficarem em linha)
    y_pos=$((i * 2)) 
    pose="0,$y_pos"

    echo "Iniciando Drone $i | Mundo: $world | MAVSDK: $mavsdk_port | Pose: $pose"

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
        sleep 10
    fi
done

echo "Enxame inicializado no mundo '$world'! Abra o QGroundControl para visualizar."
wait