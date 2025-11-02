#!/bin/bash

# Este script configura o ambiente 'ardu_ws' padrão do container
# para ser usado com os pacotes de missão personalizados.

set -e

# === 1. DEFINIÇÃO DE CAMINHOS ===

ARDU_WS_PATH="/home/rosuser/ardu_ws"
MISSION_REPO_PATH=$(pwd) # (Assume que o script é rodado de /root/avant-finalproject)


echo "--- 1. Instalando Dependências Python (Pip) ---"
pip3 install "numpy<2.0" 
echo "Dependências Pip instaladas."


echo "--- 2. Procurando o Launch File do MAVROS ---"
# Encontra o launch file do mavros_test para fazer o patch
MAVROS_LAUNCH_FILE=$(find $ARDU_WS_PATH/src/mavros_test/launch -name "*.py" -print -quit)

if [ -z "$MAVROS_LAUNCH_FILE" ]; then
    echo "ERRO: Não foi possível encontrar o launch file do MAVROS em $ARDU_WS_PATH/src/mavros_test/launch/"
    exit 1
fi
echo "Launch file encontrado: $MAVROS_LAUNCH_FILE"


echo "--- 3. Corrigindo Configurações UDP do MAVROS ---"
# Corrige o fcu_url (de "Servidor" para "Cliente")
sed -i "s|'fcu_url': 'udp://@127.0.0.1:14550'|'fcu_url': 'udp://127.0.0.1:14550'|g" $MAVROS_LAUNCH_FILE
# Desativa o gcs_url (já que não estamos usando o QGroundControl)
sed -i "s|'gcs_url': 'udp://127.0.0.1:14551'|'gcs_url': ''|g" $MAVROS_LAUNCH_FILE
echo "Configurações UDP do MAVROS corrigidas."


echo "--- 4. Copiando Pacotes da Missão para o ardu_ws ---"
# Copia seus 3 pacotes para o 'src' do workspace de simulação
cp -r $MISSION_REPO_PATH/src/* $ARDU_WS_PATH/src/
echo "Pacotes vant_vision_pkg, vant_navigation_pkg, e vant_actuator_pkg copiados."


echo "--- 5. Compilando o Workspace Inteiro ---"
cd $ARDU_WS_PATH
colcon build
echo "Workspace compilado com sucesso."


echo "---"
echo "✅ Ambiente pronto!"
