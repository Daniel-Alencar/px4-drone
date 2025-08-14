import multiprocessing
import subprocess
import time

def run_drone(script_name):
    subprocess.run(["python3", script_name])

if __name__ == "__main__":
    # Cria dois processos independentes
    p1 = multiprocessing.Process(target=run_drone, args=("drone1.py",))
    p2 = multiprocessing.Process(target=run_drone, args=("drone2.py",))

    # Inicia os dois processos
    p1.start()
    p2.start()

    # Espera os dois estarem prontos
    print("⌛ Aguardando inicialização...")
    time.sleep(5)

    print("\n🚀 Iniciando decolagem conjunta!\n")
    # Aqui, nos scripts dos drones, eles esperam ENTER
    # Você aperta ENTER no terminal de cada processo ou pode
    # automatizar enviando sinais

    # Aguarda finalização
    p1.join()
    p2.join()
