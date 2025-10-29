# TFE Eletrônica AVANT 2025.2: Navegação Autônoma e Visão Computacional

Este repositório contém o código-fonte do Projeto Final de Eletrônica (TFE) para a equipe AVANT 2025.2.

O objetivo deste projeto é desenvolver um algoritmo de **navegação autônoma e visão computacional** para um VANT (Veículo Aéreo Não Tripulado). A missão consiste em fazer o drone seguir uma linha detectada no solo e, simultaneamente, manter-se centralizado em relação a uma estrutura de travessão, utilizando um ambiente de simulação pré-configurado no **Gazebo SIM**.

---

## Visão Geral do Projeto

O sistema é dividido em alguns componentes principais:

1.  **Simulação:** Um ambiente no Gazebo SIM que replica o cenário da competição, incluindo o drone, a linha e a travessão.
2.  **Visão Computacional:** Um nó ROS que processa o feed da câmera do drone, identifica a linha (usando OpenCV para detecção de bordas ou cor) e localiza o centro da travessão.
3.  **Controle e Navegação:** Um nó ROS que recebe as informações da visão e gera comandos de velocidade (linear) para o drone, implementando a lógica de seguimento de linha e centralização.
