#!/bin/bash

# Carrega o ambiente ROS 2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 
source ~/ardu_ws/install/setup.bash # se estiver usando workspace

# Entra na pasta do script
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
cd "$SCRIPTPATH" || exit 1

# Atualiza o link simbólico
rm -f .tmuxinator.yml
ln -s session.yml .tmuxinator.yml

# Inicia o tmuxinator com a configuração da Fase 1
tmuxinator start -p Projeto.yml
