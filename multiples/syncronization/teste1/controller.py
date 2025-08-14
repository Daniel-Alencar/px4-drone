import multiprocessing
import pickle
import os
import sys
import time

def start_drone_process(system_address, label):
    parent_conn, child_conn = multiprocessing.Pipe()

    # Criar descritor de arquivo para passar o Pipe via FD
    r_fd, w_fd = os.pipe()
    os.write(w_fd, pickle.dumps(child_conn))
    os.close(w_fd)

    # Inicia processo separado com script worker
    proc = multiprocessing.Process(
        target=os.execvp,
        args=(
            sys.executable,
            [sys.executable, "drone_worker.py", system_address, label],
        ),
        pass_fds=(r_fd,),
    )
    proc.start()
    os.close(r_fd)
    return proc, parent_conn

if __name__ == "__main__":
    # Inicia processos dos dois drones
    p1, conn1 = start_drone_process("udp://:14541", "Drone 1")
    p2, conn2 = start_drone_process("udp://:14542", "Drone 2")

    time.sleep(5)  # Espera ambos conectarem

    print("\n🚀 Decolando ambos!\n")
    conn1.send("takeoff")
    conn2.send("takeoff")

    time.sleep(15)  # Tempo no ar

    print("\n🛬 Pousando ambos!\n")
    conn1.send("land")
    conn2.send("land")

    time.sleep(5)

    conn1.send("exit")
    conn2.send("exit")

    p1.join()
    p2.join()

    print("✅ Missão finalizada.")
