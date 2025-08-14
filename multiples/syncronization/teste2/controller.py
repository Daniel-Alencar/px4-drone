# controller_multiprocess.py
import multiprocessing as mp
import asyncio
import time
from mavsdk import System

# -----------------------
# Utilitários do worker
# -----------------------
async def connect_and_check(system_address: str, label: str, grpc_port: int) -> System:
    # Cada processo usa um gRPC (mavsdk_server) diferente
    drone = System(port=grpc_port)
    await drone.connect(system_address=system_address)

    print(f"[{label}] 🔌 Conectando em {system_address} (gRPC {grpc_port})...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[{label}] ✅ Conectado.")
            break

    print(f"[{label}] ⏳ Verificando sensores...")
    async for health in drone.telemetry.health():
        if (health.is_gyrometer_calibration_ok
            and health.is_accelerometer_calibration_ok
            and health.is_magnetometer_calibration_ok):
            print(f"[{label}] ✅ Sensores prontos.")
            break
    return drone

async def arm_takeoff(drone: System, label: str):
    print(f"[{label}] 🚁 Armando...")
    await drone.action.arm()
    print(f"[{label}] 🚀 Decolando...")
    await drone.action.takeoff()
    await asyncio.sleep(10)

async def land(drone: System, label: str):
    print(f"[{label}] 🛬 Pousando...")
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print(f"[{label}] ✅ Pousou com sucesso.")
            break

# -----------------------
# Worker de um drone
# -----------------------
def drone_worker(system_address: str, label: str, grpc_port: int,
                 cmd_queue: mp.Queue, ready_evt: mp.Event, start_evt: mp.Event):
    async def runner():
        drone = await connect_and_check(system_address, label, grpc_port)
        # Sinaliza ao controlador que está pronto
        ready_evt.set()

        while True:
            cmd = cmd_queue.get()  # bloqueia até vir um comando
            if cmd == "takeoff":
                # Espera o controlador liberar a decolagem para ser simultânea
                start_evt.wait()
                await arm_takeoff(drone, label)
            elif cmd == "land":
                await land(drone, label)
            elif cmd == "exit":
                print(f"[{label}] Encerrando processo.")
                break

    asyncio.run(runner())

# -----------------------
# Controlador
# -----------------------
if __name__ == "__main__":
    # No Linux geralmente funciona com 'fork', mas 'spawn' evita herança estranha de estado
    mp.set_start_method("spawn", force=True)

    # Filas de comando e eventos de sincronização
    q1, q2 = mp.Queue(), mp.Queue()
    ready1, ready2 = mp.Event(), mp.Event()
    start_both = mp.Event()  # barreira de decolagem

    # Mapas de conexão (ajuste portas conforme seu setup)
    DRONE1 = dict(system_address="udp://:14541", label="Drone 1", grpc_port=50051)
    DRONE2 = dict(system_address="udp://:14542", label="Drone 2", grpc_port=50052)

    p1 = mp.Process(target=drone_worker, args=(
        DRONE1["system_address"], DRONE1["label"], DRONE1["grpc_port"], q1, ready1, start_both
    ))
    p2 = mp.Process(target=drone_worker, args=(
        DRONE2["system_address"], DRONE2["label"], DRONE2["grpc_port"], q2, ready2, start_both
    ))

    p1.start()
    p2.start()

    # Aguarda os dois processos conectarem e ficarem prontos
    print("⌛ Aguardando drones ficarem prontos...")
    ready1.wait()
    ready2.wait()

    # Envia comando de decolagem para os dois e libera simultaneamente pela barreira
    print("\n🚀 Iniciando decolagem conjunta!\n")
    q1.put("takeoff")
    q2.put("takeoff")
    time.sleep(0.5)  # garante que os workers já estão esperando a barreira
    start_both.set()  # ambos decolam juntos

    # Tempo no ar
    time.sleep(15)

    # Pouso simultâneo (não precisa de barreira, mas você pode usar outra se quiser)
    print("\n🛬 Pousando ambos!\n")
    q1.put("land")
    q2.put("land")

    # Finaliza
    time.sleep(5)
    q1.put("exit")
    q2.put("exit")

    p1.join()
    p2.join()
    print("✅ Missão finalizada.")
