# Simulação de VANTs & Experimentos de Controle Autônomo

Este repositório contém um conjunto abrangente de ferramentas em Python, experimentos e pipelines de simulação para o desenvolvimento e teste de capacidades autônomas de VANTs (Veículos Aéreos Não Tripulados / Drones). Ele integra **PX4 SITL**, **MAVLink** e **Gazebo** para criar um ambiente totalmente virtual para prototipagem rápida e validação de comportamentos de múltiplos drones.

O projeto foi construído como um espaço de trabalho flexível para pesquisa em **controle de formação**, **planejamento cooperativo de missões**, **pouso baseado em visão computacional** e **mapeamento de ambiente**.

---

## 🎯 Propósito

Este repositório serve como um ambiente unificado onde algoritmos podem ser projetados, testados e iterados sem depender de hardware real. É destinado a:

* Experimentação com estratégias de controle autônomo.
* Desenvolvimento de navegação e pouso baseados em percepção.
* Simulação de missões cooperativas com múltiplos VANTs.
* Prototipagem rápida de ideias de pesquisa antes da implantação no mundo real.

---

## ✨ Principais Funcionalidades

### 1. Controle de Formação
Implementações e experimentos envolvendo:
* Coordenação líder-seguidor.
* Formação de estrutura virtual.
* Controle de movimento baseado em consenso.
* Rastreamento cooperativo de trajetórias.

> **Nota:** Esses scripts se comunicam com o SITL através do MAVLink, permitindo o movimento sincronizado de múltiplos veículos no Gazebo.

### 2. Detecção de Zona de Pouso Baseada em Visão
Pipelines de visão computacional para identificar áreas seguras de pouso, incluindo:
* Extração e filtragem de características (*features*).
* Heurísticas baseadas em limiarização (*thresholding*).
* Segmentação de regiões.

### 3. Mapeamento de Área e Reconstrução de Ambiente
Ferramentas para gerar mapas 2D ou pseudo-3D através de:
* Síntese de dados simulados de câmeras ou LiDAR.
* Costura de imagens (*stitching*) ou mosaicos.
* Construção de grades de ocupação (*occupancy grids*) ou mapas anotados.
* Geração de padrões de voo no estilo de levantamento topográfico (*survey*).

---

## 🛠️ Dependências e Requisitos

Para executar as simulações, os seguintes softwares e bibliotecas são necessários:

* **Python 3.x**
* **MAVSDK / pymavlink**
* **PX4 SITL**
* **Gazebo / Ignition Gazebo**
* **ROS 2**
* **OpenCV** (para módulos de percepção)
* **NumPy, SciPy, Matplotlib** (para processamento de dados e visualização)

---

## ⚙️ Como Funciona

1. O **PX4 SITL** inicia os VANTs virtuais no Gazebo.
2. Os **scripts em Python** se conectam via MAVLink para controlar os veículos.
3. Os algoritmos de controle, percepção e mapeamento rodam inteiramente em Python.
4. A simulação retorna dados de sensores e estado em tempo real.
5. Os resultados podem ser visualizados, registrados (logs) ou reproduzidos para análise.

---

## 🚀 Como Começar

Para configurar adequadamente o seu ambiente e executar os algoritmos, siga os guias detalhados fornecidos nos nossos arquivos de documentação:

* **Instalação:** Siga o passo a passo no arquivo [INSTALL.md](INSTALL.md) para instalar e configurar todos os softwares e dependências necessárias.
* **Execução:** Consulte o arquivo [RUN.md](RUN.md) para obter instruções precisas sobre como iniciar as simulações e os algoritmos de forma correta.

---

## 📁 Estrutura do Repositório

O espaço de trabalho está organizado nos seguintes diretórios principais:

* `algorithms/`: Contém exemplos conceituais de algoritmos de controle de formação e desvio de obstáculos. Estes scripts aplicam apenas a lógica matemática e *não* utilizam a física de drones do PX4 e Gazebo, servindo como uma base simplificada para entender o funcionamento dos algoritmos.
* `multiples/`: Contém os arquivos centrais necessários para a simulação de controle de formação e desvio de obstáculos utilizando instâncias reais de drones no Gazebo (ou seja, simulações de alta fidelidade, o mais próximo possível do mundo real).
* `only_one/`: Contém arquivos de simulações focadas em operações com apenas um único drone (ainda em construção).
* `configs/`: Abriga arquivos de configuração para o software **RViz2**, que se comunica com o ROS 2 e permite visualizar informações detalhadas dos sensores e atuadores do drone simulado.
* `worlds/`: Contém os arquivos de mundo (*worlds*) do Gazebo que podem ser usados para criar diferentes ambientes e cenários nas simulações.
* `assets/`: Armazena apenas arquivos de mídia, imagens e outros recursos visuais.

---

## 💡 Aplicações

Este repositório é altamente adequado para:
* Missões cooperativas de drones e experimentos de enxame (*swarming*).
* Simulações de pouso de precisão e busca e salvamento (*search-and-rescue*).
* Mapeamento baseado em VANTs.
* Benchmarking de algoritmos e estudos de ablação.