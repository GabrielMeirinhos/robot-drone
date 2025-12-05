# Drone Autônomo – Simulação com PyBullet e MQTT

A atividade foi desenvolvida utilizando a ideia de navegação autônoma baseada na detecção e visita de pontos de interesse no ambiente 3D. Nosso drone utiliza sensores simples para identificar cubos próximos e, a partir deles, calcular a melhor rota possível para sobrevoar cada ponto antes de retornar à sua posição inicial.

Implementamos uma lógica para converter a posição real do drone em coordenadas utilizáveis internamente, permitindo detectar os cubos presentes no cenário e selecionar apenas aqueles dentro do raio de atuação do sensor. A partir disso, aplicamos um processo de heurística (via permutação de rotas) para determinar a ordem ideal de visitação dos alvos, garantindo que o drone faça um percurso eficiente.

Além do movimento, o projeto também implementa um controlador de voo próprio, responsável por ajustar a velocidade horizontal e vertical do drone, garantindo estabilidade e suavidade na navegação. Cada etapa do percurso é simulada em tempo real utilizando o PyBullet.

Nosso sistema também realiza o envio de mensagens MQTT para um broker HiveMQ Cloud, reportando eventos importantes — como a remoção de um cubo detectado — e transmitindo informações sobre a posição atual do drone e do objeto encontrado. Essa interação permite integrar a simulação com dashboards externos, possibilitando monitoramento em tempo real e análises de comportamento do robô ao longo do experimento.

## Instale antes:

- pip install pybullet numpy paho-mqtt

## Como executar apenas rode:

- python main.py

