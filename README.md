# E3 — Localização visual multicâmera de robô móvel

Implementação e validação experimental de uma funcionalidade de Visão Robótica: estimativa da posição de um robô móvel utilizando duas câmeras no Gazebo, ROS Noetic e triangulação geométrica.

Este repositório é uma entrega parcial independente. O projeto completo permanece no repositório [lar-multicamera-robot-localization](https://github.com/JoaoVitorCLima/lar-multicamera-robot-localization).

## Funcionalidade apresentada

O robô possui uma esfera verde fluorescente como marcador visual. Cada câmera captura a imagem, segmenta o marcador no espaço HSV, calcula seu centro em pixels e converte o pixel em um raio de observação usando os parâmetros da câmera.

Os dois raios são expressos no referencial comum `world` e triangulados. O sistema publica a posição estimada em `/multi_camera/pose`.

```text
/camera_1/image_raw ─┐
                     ├─> triangulate_marker ─> /multi_camera/pose
/camera_2/image_raw ─┘
```

A orientação não é estimada pelo marcador esférico; a avaliação principal é a localização planar (`x,y`).

## Resultados experimentais

O experimento validado no Gazebo registrou 481 amostras durante 33,16 s:

| Métrica | Resultado |
|---|---:|
| RMSE planar | 2,21 cm |
| RMSE tridimensional | 2,26 cm |
| Maior erro 3D | 3,90 cm |
| Percentil 95 do erro 3D | 3,24 cm |

Evidências disponíveis:

- [relatório e análise](results/final_corrected/analysis.md);
- [CSV da avaliação](results/final_corrected/poses.csv);
- [métricas em JSON](results/final_corrected/metrics.json);
- [gráfico de erros](results/final_corrected/errors.png);
- [vídeo demonstrativo](results/final_corrected/multi_camera_demo.mp4).

## Execução com Docker

Requisitos: Docker e Docker Compose. O ambiente ROS Noetic é executado no contêiner e não altera o ROS, os drivers NVIDIA ou as configurações do Ubuntu host.

```bash
docker compose build
docker compose run --rm multi_camera_robot_localization \
  roslaunch multi_camera_robot_localization top_view.launch \
  gui:=false record:=true drive:=true
```

Para executar com interface gráfica do Gazebo:

```bash
xhost +SI:localuser:$(id -un)
docker compose run --rm multi_camera_robot_localization \
  roslaunch multi_camera_robot_localization top_view.launch gui:=true
xhost -SI:localuser:$(id -un)
```

## Testes automatizados

```bash
docker compose run --rm multi_camera_robot_localization \
  bash -lc 'cd /ws/src/multi_camera_robot_localization && \
  PYTHONPATH=src python3 -m unittest discover -s tests -v'
```

Os testes verificam a geometria de fusão, a triangulação e o tratamento de ângulos. Para reproduzir a validação automatizada pelo script:

```bash
./scripts/test_docker.sh
```

## Limitações e análise crítica

- a detecção depende da visibilidade e do contraste do marcador;
- oclusões podem deixar apenas uma câmera disponível;
- a precisão depende da calibração e da geometria entre as câmeras;
- bordas da imagem e linhas de visão quase paralelas aumentam a incerteza;
- a altura conhecida no simulador não deve ser apresentada como medição vertical independente;
- a validação no Gazebo deve ser complementada por testes com câmeras e robô reais.

## Estrutura principal

- `scripts/triangulate_marker.py`: detecção e triangulação;
- `launch/top_view.launch`: experimento integrado;
- `husky_urdf_extras/multi_camera.urdf`: câmeras e marcador do robô;
- `worlds/`: ambientes do Gazebo;
- `tests/`: testes automatizados;
- `results/`: dados, métricas, gráficos e vídeo.
