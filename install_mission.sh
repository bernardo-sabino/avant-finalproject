#!/bin/bash

# Este script configura o ambiente 'ardu_ws' padrão do container
# para ser usado com os pacotes de missão personalizados.

# Pára o script imediatamente se qualquer comando falhar
set -e

# === DEFINIÇÃO DE CAMINHOS ===

ARDU_WS_PATH="/home/rosuser/ardu_ws"
MISSION_REPO_PATH="/home/rosuser/avant-finalproject" 


echo "--- 1. Instalando Dependências Python (Pip) ---"
pip3 install "numpy<2.0" 
echo "Dependências Pip instaladas."


echo "--- 2. Procurando os Arquivos de Configuração do MAVROS ---"
# Encontra o launch file do mavros_test para fazer o patch
MAVROS_LAUNCH_FILE=$(find $ARDU_WS_PATH/src/mavros_test/launch -name "mavros_udp_launch.py" -print -quit)
DISTANCE_SENSOR_FILE=$(find $ARDU_WS_PATH/src/mavros_test/launch -name "distance_sensor.yaml" -print -quit)

if [ -z "$MAVROS_LAUNCH_FILE" ] || [ -z "$DISTANCE_SENSOR_FILE" ]; then
    echo "ERRO: Não foi possível encontrar 'mavros_udp_launch.py' ou 'distance_sensor.yaml' em $ARDU_WS_PATH/src/mavros_test/launch/"
    exit 1
fi
echo "Launch file encontrado: $MAVROS_LAUNCH_FILE"
echo "Arquivo YAML encontrado: $DISTANCE_SENSOR_FILE"


echo "--- 3. Corrigindo Configurações UDP do MAVROS (no launch file) ---"
# Corrige o fcu_url para a sintaxe de Servidor
sed -i "s|'fcu_url': 'udp://@127.0.0.1:14550'|'fcu_url': 'udp://127.0.0.1:14550@'|g" $MAVROS_LAUNCH_FILE
echo "Corrigido fcu_url para 'udp://127.0.0.1:14550@'"

# Remove totalmente a linha do gcs_url
sed -i "/'gcs_url'/d" $MAVROS_LAUNCH_FILE
echo "Removida a linha 'gcs_url' do launch file."

# Remove a referência ao distance_sensor_yaml do launch file
sed -i "/distance_sensor_yaml/d" $MAVROS_LAUNCH_FILE
echo "Removida a referência ao 'distance_sensor_yaml' do launch file."


echo "--- 4. Esvaziando o distance_sensor.yaml ---"
# Apaga o conteúdo do arquivo e insere o placeholder
echo "#placeholder" > $DISTANCE_SENSOR_FILE
echo "Conteúdo de 'distance_sensor.yaml' substituído por '#placeholder'."


echo "--- 5. Copiando Pacotes da Missão para o ardu_ws ---"
# Copia os 3 pacotes para o 'src' do workspace de simulação
cp -r $MISSION_REPO_PATH/src/* $ARDU_WS_PATH/src/
echo "Pacotes vant_vision_pkg, vant_navigation_pkg, e vant_actuator_pkg copiados."


echo "--- 6. Compilando o Workspace Inteiro ---"
cd $ARDU_WS_PATH
rm -rf build/ install/ log/
colcon build
echo "Workspace compilado com sucesso."


echo "---"
echo "✅ Ambiente pronto!"
echo "Faça o 'source' do workspace e execute a simulação"