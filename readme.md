🛸 Drone Simulation with PyBullet + MQTT (HiveMQ Cloud)

Simulação de navegação autônoma de um drone usando PyBullet, com detecção de pontos, cálculo de rota ótima e envio de eventos via MQTT seguro (TLS) para um broker da HiveMQ Cloud.

Este projeto simula um drone simples em 3D, capaz de detectar cubos próximos, calcular uma rota para visitá-los e publicar eventos quando um cubo é removido.

Todo o código da simulação está centralizado em um único arquivo. -main.py

#Funcionalidades
- Simulação 3D com PyBullet

Ambiente físico completo (gravidade, timestep, colisão).

Drone representado por uma caixa com massa.

Cubos gerados aleatoriamente no mapa.

✅ Detecção de pontos

Sensor circular com raio configurável.

Detecta até 3 cubos mais próximos.

✅ Cálculo da melhor rota

Percorre todas as permutações possíveis (TSP brute-force).

Gera sequência ótima para visitar os cubos detectados e retornar ao início.

✅ Controle do drone

Controlador PID simplificado sem inércia:

Controla horizontal (XY) com velocidade limitada.

Controla vertical (Z) com ganho próprio.

Mantém voo suave.

✅ Envio via MQTT (TLS - HiveMQ Cloud)

Conexão segura usando TLS/SSL.

Envia JSON com dados assim que o cubo é removido:

{
  "cubo_removido": {
    "id": 5,
    "posicao_cubo": [...],
    "posicao_drone": [...]
  }
}


Usuário e senha configurados.

📦 Dependências

Instale antes:

pip install pybullet numpy paho-mqtt


Como executar apenas rode:

python main.py

Se quiser rodar sem interface gráfica:

python main.py --headless

🔍 Resumo das Classes
🧩 MQTT

Gerencia conexão TLS com HiveMQ Cloud e publica eventos de forma segura.

🌍 AmbienteSimulacao

Configura PyBullet, chão, timestep e atualizações do mundo.

🛸 DroneSimples

Representa o drone:

Caixa 3D azul

Posição

Corpo rígido

👀 SensorPontos

Detecta cubos próximos ao drone.

🎮 ControladorDrone

Controlador de movimento:

Move o drone em direção ao alvo

Controla velocidades XY e Z separadamente

🧱 Utilitários

Criar cubos

Gerar pontos espalhados

Calcular a menor rota (TSP brute force)

🔄 Fluxo Principal

Criar ambiente, drone, sensor, controlador e MQTT.

Gerar 6 cubos espalhados pelo mapa.

Drone detecta os 3 mais próximos.

Calcula rota ótima para visitá-los.

Ao passar sobre um cubo:

O cubo é removido da simulação.

Evento MQTT é enviado.

Drone volta ao ponto inicial.

📡 Exemplo de JSON enviado
{
  "cubo_removido": {
    "id": 37,
    "posicao_cubo": [1.2, -3.4, 0.2],
    "posicao_drone": [1.25, -3.38, 1.50]
  }
}

📝 Licença

Este projeto pode ser usado para fins acadêmicos, estudos de navegação autônoma e integração com MQTT.