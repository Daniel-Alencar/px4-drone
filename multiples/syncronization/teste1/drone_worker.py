import asyncio
import sys
from mavsdk import System

async def connect_and_check(system_address, label):
    drone = System()
    await drone.connect(system_address=system_address)
    print(f"[{label}] Conectando a {system_address}...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[{label}] Conectado.")
            break

    async for health in drone.telemetry.health():
        if (
            health.is_gyrometer_calibration_ok and
            health.is_accelerometer_calibration_ok and
            health.is_magnetometer_calibration_ok
        ):
            print(f"[{label}] Sensores prontos.")
            break
    return drone

async def arm_takeoff(drone, label):
    print(f"[{label}] Armando...")
    await drone.action.arm()
    print(f"[{label}] Decolando...")
    await drone.action.takeoff()
    await asyncio.sleep(10)

async def land(drone, label):
    print(f"[{label}] Pousando...")
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print(f"[{label}] Pousou com sucesso.")
            break

async def main(system_address, label, pipe):
    drone = await connect_and_check(system_address, label)

    while True:
        cmd = pipe.recv()  # Espera comando do controller
        if cmd == "takeoff":
            await arm_takeoff(drone, label)
        elif cmd == "land":
            await land(drone, label)
        elif cmd == "exit":
            print(f"[{label}] Encerrando processo.")
            break

if __name__ == "__main__":
    # sys.argv[1] = system_address
    # sys.argv[2] = label
    # pipe é passado pelo controller
    import pickle
    import os

    pipe = pickle.loads(os.read(3, 4096))  # lê pipe serializado do controller
    asyncio.run(main(sys.argv[1], sys.argv[2], pipe))
