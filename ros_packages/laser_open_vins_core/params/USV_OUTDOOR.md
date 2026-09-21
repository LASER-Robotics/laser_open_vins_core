# OpenVINS no rio: comparação mono / estéreo

Estes perfis são um ponto de partida para a **simulação** com `front_rgbd`
(848×480, estéreo com baseline de 5 cm). Não são uma calibração para uma
RealSense física. A geometria de calibração foi preservada; as densidades de
ruído da IMU foram ajustadas provisoriamente ao jitter medido na simulação.
O objetivo é comparar o OpenVINS mono e estéreo usando a imagem inteira,
incluindo água e objetos móveis, como base para a pesquisa de mestrado.
Ainda é necessário validar a precisão com imagens, IMU e referência de pose.

## O que mudou

| Parâmetro | Estéreo anterior | Mono anterior | Ambos agora |
|---|---:|---:|---:|
| `max_clones` | 20 | 50 | 30 |
| `dt_slam_delay` | 2 s | 2 s | 5 s |
| `init_window_time` | 2 s | 4 s | 4 s |
| `init_imu_thresh` | 0.75 | 0.50 | 0.50 |
| `init_max_disparity` | 0 (ignorado) | 2 px | 0 (ignorado) |
| `init_max_features` | 80 | 40 | 200 |
| `init_dyn_use` | true | true | true (compatibilidade do parser) |
| `zupt_max_disparity` | 0.50 px | 0.10 px | 0 (somente IMU/velocidade) |
| `fast_threshold` | 15 | 15 | 20 |
| `min_px_dist` | 6 px | 6 px | 12 px |
| `use_mask` | false | false | false |

`num_pts: 600` é o orçamento total: o código divide pelo número de câmeras.
Esta comparação mantém o mesmo orçamento total, não a mesma quantidade por
imagem. O número efetivo também depende da textura e da sobrevivência dos tracks.
Os demais limites de updates, ruídos visuais e calibração são iguais.

O ajuste de repouso de 18/09/2026 usa, nos dois perfis:

```yaml
# estimator_config.yaml
try_zupt: true
zupt_max_velocity: 0.1
zupt_noise_multiplier: 10
zupt_max_disparity: 0.0
zupt_only_at_beginning: true

# kalibr_imu_chain.yaml, dentro de imu0
accelerometer_noise_density: 0.025
gyroscope_noise_density: 0.0004
```

Antes deste ajuste, as densidades eram `0.0018` e `0.00002`, com multiplicador
ZUPT `50`. Com o robô praticamente imóvel, a amostra coletada apresentou
desvios por eixo de até **0,258 m/s²** e **0,00433 rad/s**, a aproximadamente
125 Hz. As novas densidades usam `max(std por eixo) / sqrt(frequência)`,
arredondado para cima. São um modelo efetivo provisório da perturbação observada,
não uma calibração de ruído branco: a amostra tem apenas 3,1 s e inclui jitter
da dinâmica simulada. Os random walks e a geometria não foram alterados.
Não transfira esses valores para uma IMU física nem use oscilações reais do
barco para estimar ruído. Os parâmetros precisam ser reavaliados com uma
gravação maior em repouso e movimento.

FAST mais alto e pontos mais espaçados reduzem cantos fracos e concentrações
locais. Eles **não distinguem água de margem**. Água/reflexos coerentes podem
passar pelo RANSAC e pelos testes do filtro; aumentar features ou landmarks
SLAM não resolve essa hipótese de cena estática violada.

## Imagem inteira e features móveis

`use_mask: false` está aplicado nos dois perfis, sem chaves `mask0`/`mask1`.
Nenhum recorte da imagem ou segmentação de água é aplicado. Os PNGs e o
script de geração criados anteriormente ficaram disponíveis, mas não são
carregados nem executados por estes perfis.

Permanecem o tracking KLT, o RANSAC e os testes de consistência dos updates
nativos do OpenVINS. Features fixas e móveis podem ser detectadas; as móveis
podem ser rejeitadas por inconsistência geométrica. Não foi implementada uma
classificação estático/dinâmico. Se o movimento da água dominar os tracks,
ainda poderá produzir drift: essa limitação faz parte do experimento de base.

Inspecione `/<namespace>/STEREO/ov_msckf/trackhist` e
`/<namespace>/MONO/ov_msckf/trackhist` no RViz/rqt para acompanhar os tracks.

## Inicialização e repouso

Comece fisicamente imóvel por pelo menos 4 segundos **após as imagens e a IMU
começarem a chegar**, e aguarde `[init]: successful initialization`. Só então
inicie o movimento. Motor desligado não significa repouso se houver deriva
ou oscilação do barco.

`init_max_disparity: 0.0` impede que a água em movimento bloqueie a decisão
inicial de repouso. A IMU ainda precisa passar no teste estático com
`init_imu_thresh: 0.50`. As imagens continuam sendo rastreadas.

O parser desta versão rejeita `init_max_disparity: 0` combinado com
`init_dyn_use: false`. Por isso `init_dyn_use` fica `true` por compatibilidade.
No fluxo de `InertialInitializer.cpp`, desativar a disparidade deixa
`is_still=true`; com `try_zupt: true` e `init_imu_thresh > 0`, o caminho escolhido
continua sendo **estático**. Isso não autoriza começar com o robô em movimento.

Se não inicializar, execute com `verbosity:=DEBUG` e confira chegada de imagens,
IMU, timestamps, unidades, frame óptico e `[init-s]: IMU excitation stats`.
Se faltarem cantos na margem, experimente FAST 15 nos dois perfis. Não aumente
o limiar da IMU para tratar oscilações físicas como ruído.

O ZUPT continua habilitado somente no início, com
`zupt_max_disparity: 0.0`: a aceitação depende da IMU e da velocidade estimada,
sem a alternativa visual que pode confundir pouca disparidade com repouso.
Isso não elimina toda ambiguidade entre repouso e movimento a velocidade
constante. Após sair da fase inicial, a odometria usa os updates visuais e
inerciais normais; não há imposição de repouso para esconder drift posterior.
Enquanto o ZUPT é aceito, os updates visuais normais são pulados pelo código
nativo, embora o tracking continue. Para a comparação durante navegação,
analise também o trecho após a saída do repouso inicial.

Nesta implementação, o ZUPT padrão usa restrições de aceleração e velocidade
angular nulas; ele não atribui diretamente zero à velocidade estimada.
Portanto, ainda pode haver pequenas correções de posição e velocidade durante
o repouso. O ajuste reduz drift inicial; não garante uma pose perfeitamente
congelada. Após a primeira saída dessa fase, novas paradas **não** reativam
o ZUPT com este perfil.

No teste com movimento sintético, a saída do ZUPT demorou aproximadamente
1,2 s para uma aceleração de 0,3 m/s² e 0,13 s para uma rotação de 0,15 rad/s.
Isso verifica que o filtro pode sair do repouso, mas também mostra que uma
partida lenta pode ser parcialmente suprimida. Não use esses resultados como
validação de precisão em navegação. Habilitar ZUPT durante todo o percurso
suprimiu ainda mais a translação lenta nesse teste, por isso
`zupt_only_at_beginning` permanece `true`.

## Execução e comparação

Os YAMLs instalados neste workspace são links para os arquivos fonte; basta
reiniciar os estimadores para carregar a alteração, sem novo build. Se precisar
reinstalar o pacote em outro ambiente:

```bash
cd ~/laser_usv_system_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select laser_open_vins_core --symlink-install
source install/setup.bash
```

Use as duas chamadas existentes no `tmux_chico/session.yml`, com
`open_vins_params_folder:=realsense_usv` e `realsense_usv_mono` respectivamente.
O `tmux_chico/session.yml` foi ajustado com `use_sim_time:=true` em ambas,
pois o default do launch depende da variável `REAL_UAV`. As entradas devem ser a
mesma `front_rgbd/infra1/image_raw` e `front_rgbd/imu`, mais `infra2/image_raw`
no estéreo; mantenha namespaces separados como no tmux.

1. Grave um rosbag com as duas imagens, IMU, `/clock` e a referência de pose
   da simulação. Inclua 30–60 s imóvel, saída lenta, curva e nova parada.
2. Reproduza a mesma gravação para cada perfil, preferencialmente um processo
   por vez para evitar competição por CPU. Para replay com `/clock` já gravado,
   reproduza esse tópico; se a gravação não o contiver, use `ros2 bag play
   <bag> --clock`. Não publique dois clocks simultaneamente.
3. Grave os tópicos `/<namespace>/<MONO|STEREO>/ov_msckf/odomimu` e `poseimu`.
   No `ROS2Visualizer` local, `save_total_state` e os caminhos de estado são
   lidos de parâmetros ROS; mudar apenas esses campos no YAML não os ativa.
4. Meça deslocamento acumulado, velocidade RMS no trecho realmente parado,
   erro de trajetória (ATE/RPE) durante movimento e falhas de inicialização.
   Transforme a referência do `base_link` para a IMU antes de comparar poses;
   alinhe os referenciais e o instante inicial. Para comparar escala métrica,
   não use alinhamento com escala livre.
5. Confira `/tmp/ov_usv_mono_timing.txt` e
   `/tmp/ov_usv_stereo_timing.txt`, atrasos e perda de frames. Esses arquivos
   são sobrescritos em cada execução: copie-os entre ensaios. O timing interno
   não inclui toda a latência do ROS e pode não registrar updates ZUPT.

O estéreo fornece baseline instantâneo; o mono precisa de translação temporal
para triangulação. Com baseline estéreo de 5 cm, a vantagem de profundidade
diminui para margens distantes. `fi_max_baseline: 200` limita a razão
distância/baseline: somente 5 cm correspondem a cerca de 10 m nesse teste
geométrico, podendo aumentar quando houver baseline temporal. O limite
`fi_max_dist: 100` por si só não garante triangulação até 100 m. Não relaxe
essas verificações para aceitar profundidades mal condicionadas.

Nenhum dos dois modos garante drift zero. Os landmarks SLAM deste filtro
não equivalem a fechamento de loop global. Escolha o melhor modo pelos erros
no mesmo ensaio e pelo custo de processamento, não pela quantidade de pontos.

## Validação

Os dois YAMLs sem máscara foram verificados quanto à sintaxe e à consistência
entre mono e estéreo. Os links da instalação apontam para as versões atuais.
O build do pacote foi concluído no ajuste anterior. O ajuste atual altera
apenas configuração e documentação; não requer recompilação.

A medição ao vivo **anterior ao ajuste de ruído** confirmou drift real, e não
apenas movimento da câmera do RViz. Em 3,096 s de simulação:

| Saída | Deslocamento entre início e fim |
|---|---:|
| Ground truth | 0,000213 m |
| MONO | 0,41587 m |
| STEREO | 0,32283 m |

Para avaliar o ajuste sem acessar a simulação, um driver nativo do estimador
repetiu ciclicamente as 388 amostras de IMU coletadas. As imagens eram
**sintéticas**, com textura fixa nos dois terços superiores e textura móvel
no terço inferior, disponível ao tracker sem máscara. O teste não é um
replay de imagens reais do rio nem uma gravação de 60 s independentes de IMU.

Com os novos parâmetros, ambos os modos tiveram máximo de aproximadamente
**0,041 m** de norma de posição e **0,040 m/s** de velocidade no ensaio de 60 s.
Ambos terminaram com norma de posição de aproximadamente **0,010 m** e
velocidade de **0,0072 m/s**, aceitando os 1.678 updates ZUPT após inicializar.
O tempo é o dos dados sintéticos. Durante esse repouso,
o fluxo nativo pula os updates visuais; resultados iguais entre mono e estéreo
não demonstram desempenho equivalente durante navegação.

A transição para movimento foi verificada separadamente conforme descrito
acima; a qualidade da trajetória em movimento não foi validada.
O teste ideal anterior, com IMU perfeitamente constante, não reproduzia o
jitter observado e não bastava para diagnosticar este problema.

### Validação ao vivo após o ajuste — 18/09/2026

Os dois comandos solicitados foram executados simultaneamente nos painéis
MONO e STEREO do tmux, com os tópicos reais da simulação aberta. Ambos
inicializaram, carregaram os novos ruídos e `use_sim_time: true`, e tiveram
um único publicador de odometria cada. Nenhum parâmetro foi alterado durante
este teste.

O objetivo era observar 30 s após inicializar. Ambos receberam Ctrl+C
externamente durante a coleta; o intervalo comum disponível foi de
**21,976 s**, entre os tempos simulados 101,448 e 123,424 s.

| Saída | Deslocamento líquido | Máximo afastamento da primeira posição | Velocidade média publicada |
|---|---:|---:|---:|
| Ground truth | 0,000493 m | 0,000944 m | 0,00279 m/s |
| MONO | 0,01497 m | 0,04336 m | 0,03651 m/s |
| STEREO | 233,064 m | 233,064 m | 12,067 m/s |

São normas de deslocamentos dentro de cada referencial, no mesmo intervalo;
não são ATE com alinhamento entre os frames. O ground truth confirma repouso
prático. O MONO limitou a variação de posição a centímetros, mas ainda
publicou velocidade residual; o STEREO **divergiu fortemente**. Portanto,
o bom resultado sintético anterior não validou o estéreo na cena real da
simulação, e o ajuste atual ainda não resolve o problema dos dois modos.

O MONO aceitou 531 updates ZUPT, sem rejeições registradas. O STEREO aceitou
somente um; em seguida a velocidade estimada passou de 0,024 para 0,131 m/s,
acima de `zupt_max_velocity: 0.1`. As quatro rejeições iniciais ocorreram
mesmo com chi² abaixo do limiar. O texto do log imprime `chi2 > limiar`
também quando a rejeição vem da velocidade, por isso essa desigualdade
impressa é enganosa. Depois que o filtro formou cinco clones,
`zupt_only_at_beginning: true` impediu novas tentativas. Isso identifica o
caminho da saída indevida do repouso; não prova uma causa única para toda
a divergência posterior.

A IMU chegou a 125 Hz sem regressões de timestamp. Nas câmeras, cerca de
72% das mensagens consecutivas repetiram o timestamp anterior, sem voltar
no tempo; cada tópico tinha apenas um publicador. Esse comportamento da
fonte das imagens também merece verificação, mas não foi isolado como causa
do erro. Não foi aplicado mascaramento nem alterada a sincronização.

Ao receberem Ctrl+C, ambos terminaram com código -11, consistente com a
falha de encerramento já observada. A divergência do estéreo ocorreu
**antes** da interrupção. A coleta foi salva e o observador foi encerrado.
Artefatos deste ensaio:
`/tmp/openvins_live_validation_20260918/run1/results.json`,
`/tmp/openvins_live_validation_20260918/run1/samples.npz`,
`/tmp/openvins_live_validation_20260918/run1/mono.log` e
`/tmp/openvins_live_validation_20260918/run1/stereo.log`.

A comparação de qualidade em navegação continua pendente. O MONO permaneceu
em ZUPT, enquanto o STEREO saiu dessa fase: este ensaio não permite concluir
que mono é superior a estéreo em movimento. Para isso, é necessário repetir
partida lenta, curva e nova parada com referência de pose.

Arquivos locais da análise (temporários):
`/tmp/openvins_stationary_diagnostics_20260918_145443.npz`,
`/tmp/ov_final_stationary_results.json`,
`/tmp/ov_offline_noise10_begin_results.json` e
`/tmp/ov_stationary_motion.cpp`.
Backup antes deste ajuste:
`/tmp/openvins_before_stationary_noise_20260918_154405/`.

O executável existente apresentou `SIGSEGV` ao encerrar com SIGINT, depois do
teste. A mesma falha foi reproduzida com o YAML estéreo original, portanto
também existe sem estes ajustes. O encerramento do executável precisa de uma
investigação separada; este ajuste de repouso não altera código C++. Logs locais:
`/tmp/ov_usv_unmasked_stereo_stationary_test.log`,
`/tmp/ov_usv_unmasked_mono_stationary_test.log` e
`/tmp/ov_usv_stereo_original_stationary_test.log`.

Referências: [ZUPT no OpenVINS](https://docs.openvins.com/update-zerovelocity.html)
e [calibração de sensores](https://docs.openvins.com/gs-calibration.html).
