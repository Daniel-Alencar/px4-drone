import asyncio
from mavsdk import System
import math

MATRIX = [
    [0, 0, 2],
    [0, 0, 0],
    [0, 0, 1]
]

REFERENCE_LAT = None
REFERENCE_LON = None


def find_drone_position(drone_id):
    for y, row in enumerate(MATRIX):
        for x, value in enumerate(row):
            if value == drone_id:
                return (x, y)
    return None


async def connect_and_check(system_address):
    drone = System()
    await drone.connect(system_address=system_address)

    print(f"Conectando ao drone em {system_address}...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Conectado ao drone em {system_address}")
            break

    print(f"Verificando sensores em {system_address}...")
    async for health in drone.telemetry.health():
        if (
            health.is_gyrometer_calibration_ok and
            health.is_accelerometer_calibration_ok and
            health.is_magnetometer_calibration_ok
        ):
            print(f"Sensores prontos em {system_address}")
            break
    return drone


async def arm_takeoff(drone, label, target_alt):
    print(f"[{label}] Armando...")
    await drone.action.arm()
    print(f"[{label}] Decolando para {target_alt} m...")
    await drone.action.set_takeoff_altitude(target_alt)
    await drone.action.takeoff()
    await asyncio.sleep(10)


async def move_relative_to_reference(drone, label, x, y, cruise_alt):
    global REFERENCE_LAT, REFERENCE_LON

    # Definir referência se não existir
    if REFERENCE_LAT is None or REFERENCE_LON is None:
        async for pos in drone.telemetry.position():
            REFERENCE_LAT = pos.latitude_deg
            REFERENCE_LON = pos.longitude_deg
            print(f"[{label}] Definindo referência global: ({REFERENCE_LAT}, {REFERENCE_LON})")
            break

    LAT_STEP = 0.00001
    LON_STEP = 0.00001

    lat_target = REFERENCE_LAT + (y * LAT_STEP)
    lon_target = REFERENCE_LON + (x * LON_STEP)

    print(f"[{label}] Indo para posição ({x},{y}) a {cruise_alt} m...")
    await drone.action.goto_location(lat_target, lon_target, cruise_alt, 0)
    await asyncio.sleep(5)


async def descend_to_formation_alt(drone, label, alt, min_distance=2.0):
    """Desce apenas se não houver outro drone muito próximo"""
    while True:
        safe_to_descend = True
        async for pos in drone.telemetry.position():
            my_lat, my_lon = pos.latitude_deg, pos.longitude_deg
            break

        # Aqui poderíamos ler a posição de outros drones via telemetria compartilhada
        # Para teste, vamos apenas assumir que não há ninguém no mesmo ponto
        # Em sistema real, isso seria substituído por troca de dados entre drones

        if safe_to_descend:
            print(f"[{label}] Descendo para {alt} m...")
            await drone.action.goto_location(my_lat, my_lon, alt, 0)
            break

        print(f"[{label}] Esperando espaço para descer...")
        await asyncio.sleep(1)


async def land(drone, label):
    print(f"[{label}] Pousando...")
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print(f"[{label}] Drone pousou com sucesso.")
            break
    await asyncio.sleep(5)


async def main_drone1():
    pos = find_drone_position(1)
    if not pos:
        print("Drone 1 não encontrado na matriz.")
        return
    drone1 = await connect_and_check("udp://:14541")
    await arm_takeoff(drone1, "Drone 1", target_alt=5)  # Altura diferente
    await move_relative_to_reference(drone1, "Drone 1", *pos, cruise_alt=15)
    await descend_to_formation_alt(drone1, "Drone 1", alt=10)


async def main_drone2():
    pos = find_drone_position(2)
    if not pos:
        print("Drone 2 não encontrado na matriz.")
        return
    drone2 = await connect_and_check("udp://:14542")
    await arm_takeoff(drone2, "Drone 2", target_alt=10)  # Altura diferente
    await move_relative_to_reference(drone2, "Drone 2", *pos, cruise_alt=20)
    await descend_to_formation_alt(drone2, "Drone 2", alt=10)


async def land_drone1():
    drone1 = await connect_and_check("udp://:14541")
    await land(drone1, "Drone 1")


async def land_drone2():
    drone2 = await connect_and_check("udp://:14542")
    await land(drone2, "Drone 2")


if __name__ == "__main__":
    asyncio.run(main_drone1())
    asyncio.run(main_drone2())
    asyncio.run(asyncio.sleep(10))
    asyncio.run(land_drone1())
    asyncio.run(land_drone2())
