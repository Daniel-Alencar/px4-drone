
## ⚙️ Configuração no PX4-Autopilot

### Configuração dos Mundos (Worlds)

Para que as simulações funcionem corretamente, é necessário mover os arquivos de ambiente do Gazebo para o diretório raiz do PX4.

Dentro deste repositório, localize a pasta `worlds/`. Copie todos os arquivos contidos nela para o diretório de *worlds* do seu framework PX4-Autopilot. Se você seguiu a instalação padrão, o caminho do PX4 deve ser `~/PX4-Autopilot/Tools/simulation/gz/worlds/`.

Você pode realizar a cópia pelo terminal utilizando o comando abaixo (certifique-se de ajustar o caminho de destino exato de acordo com a versão do seu PX4):

```bash
cp worlds/* ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```

Após a cópia, a estrutura da sua pasta no PX4 deve ficar semelhante a esta:

> `[Insira aqui a Imagem da estrutura de pastas]`


## 🗂️ Estrutura do Repositório

Os algoritmos de coordenação estão localizados no diretório `multiples/`. Cada subpasta dentro deste diretório representa um algoritmo específico e contém, em sua base, dois arquivos principais:

* `start.sh`: O script em Bash responsável por inicializar o ambiente no Gazebo e instanciar os drones.
* `controller.py`: O script em Python que define a lógica de controle e execução do algoritmo em questão.
* `analysis.py`: O script em Python que faz a análise de trajeto dos drones durante a execução do algoritmo.

## 🚀 Como Executar as Simulações

### 1. Inicializando o Ambiente e os Drones

Navegue até a pasta do algoritmo que deseja testar e observe o arquivo `start.sh`.

**Personalização da Simulação:**
Dentro do arquivo `start.sh`, existem três variáveis principais que você pode editar para customizar a simulação:

```bash
# Quantidade de drones na simulação
num_vehicles=8

# Modelo do drone (ex: gz_x500, gz_x500_mono_cam)
model="gz_x500" 

# Mundo do Gazebo desejado (ex: default, obstacles_world, wind_default, wind_obstacles_world)
world="default"
```

Após customizar da forma desejada, você pode executar o script:

```bash
./start.sh
```

> ⚠️ **Atenção:** As simulações estão otimizadas para operar com até **8 drones**. Em alguns algoritmos, as configurações individuais de cada VANT são especificadas de forma *hardcoded* no arquivo `controller.py`. Configurar `num_vehicles` para um valor superior a 8 pode resultar em comportamentos inesperados sem as devidas adaptações no código.

Após a execução do script, o Gazebo será aberto automaticamente carregando o mundo escolhido e as instâncias dos drones.

### 2. Executando o Algoritmo de Controle

Com o Gazebo aberto e os drones instanciados, abra um novo terminal (na mesma pasta do algoritmo) e inicie o controlador:

```bash
python3 controller.py
```

Este script estabelecerá a comunicação com todas as instâncias no Gazebo e iniciará as iterações do algoritmo. A maioria dos algoritmos deste repositório tem o objetivo de realizar navegação de pontos (decolar, transladar até um alvo e pousar). Acompanhe o comportamento do enxame diretamente no simulador.

## 📊 Análise de Resultados

Após a conclusão da execução de um algoritmo, o sistema gera relatórios contendo os dados de mapeamento do trajeto e o custo computacional da operação.

### Análise de Trajetória

Para gerar uma análise visual do percurso realizado pelos drones, execute:

```bash
python3 analysis.py
```

Este script gerará dois arquivos de imagem:
1.  O trajeto em si plotado graficamente.
2.  Um gráfico definindo as distâncias dos drones para os obstáculos mais próximos.

> **Nota:** Dentro da pasta de cada algoritmo, você encontrará subpastas chamadas **`1, 2, 3 e 4 (representado, respectivamente, 'Mundo com obstáculos, Mundo com obstáculos e com vento, Mundo sem obstáculos e Mundo sem obstáculos e com vento')`**. Elas contêm logs e resultados de execuções anteriores validadas em todos os mundos disponíveis.

### Comparativo de Custo Computacional

Para realizar uma análise profunda e comparativa dos custos computacionais entre diferentes algoritmos, utilize o script global de análise localizado na raiz da pasta `multiples/`:

```bash
python3 custo_computacional.py
```

Este código compila os dados computacionais de todas as execuções (em diferentes cenários/mundos) a partir dos resultados das pastas `1, 2, 3 e 4`. E assim, gera gráficos comparativos de desempenho para facilitar a análise dos resultados.

## Observação

Nem todos os algoritmos conseguem rodar completamente em todos os mundos/cenários possíveis. Isso se deve ao fato de que alguns algoritmos não estão preparados para lidar com obstáculos, e assim, o objetivo de pouso dos drones nunca é alcançado e a execução nunca para ou simplesmente trava.