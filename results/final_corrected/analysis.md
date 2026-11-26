# Análise do experimento final

## Configuração

O experimento foi executado no Gazebo Classic 11, dentro do contêiner ROS
Noetic, usando o ambiente do laboratório do LaR. O Husky foi equipado com uma
esfera verde fluorescente, observada simultaneamente pelas câmeras `camera_1` e
`camera_2`.

O detector segmenta a esfera no espaço HSV, filtra regiões alongadas pela
circularidade do contorno e calcula o centro do marcador em cada imagem. Cada
centro é convertido em um raio no referencial `world`; a interseção aproximada
dos dois raios fornece a posição tridimensional estimada.

## Resultado quantitativo

| Métrica | Resultado |
|---|---:|
| Amostras pareadas | 481 |
| Duração | 33,16 s |
| RMSE planar | 0,0221 m (2,21 cm) |
| MAE planar | 0,0204 m (2,04 cm) |
| RMSE 3D | 0,0226 m (2,26 cm) |
| Erro 3D máximo | 0,0390 m (3,90 cm) |
| Percentil 95 do erro 3D | 0,0324 m (3,24 cm) |

O ground truth vertical foi ajustado pela altura mecânica conhecida de 0,72 m,
pois o Gazebo fornece a pose da base do Husky e o detector estima o centro da
esfera.

## Análise crítica

O resultado mostra erro de localização na ordem de centímetros, compatível com
o objetivo do projeto. A triangulação funcionou com as duas câmeras observando
simultaneamente o marcador, e a pose publicada foi utilizada pelo controlador
para iniciar a patrulha do robô.

A orientação não é avaliada como resultado principal: uma esfera não possui
uma orientação visual observável. O campo quaternion publicado é identidade e
o controlador utiliza a posição planar. Para estimar orientação seria necessário
um marcador assimétrico, múltiplas esferas em uma geometria conhecida ou uma
fonte adicional de orientação.

As principais limitações são a dependência das matrizes extrínsecas, a
sensibilidade à iluminação e a possibilidade de oclusão do marcador. A esfera
verde é uma aproximação para o experimento RGB no Gazebo; o marcador passivo
real do OptiTrack depende de iluminação e câmeras infravermelhas.

## Arquivos produzidos

- `poses.csv`: poses estimadas e ground truth pareado;
- `metrics.json`: métricas calculadas;
- `errors.png`: trajetória e erros ao longo do tempo;
- `multi_camera_demo.mp4`: vídeo das duas câmeras anotadas.
- `preview.png`: quadro de prévia com os rótulos e a pose triangulada.
