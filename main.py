import pybullet as p
import pybullet_data
import numpy as np
import time
import itertools
import random
import json
import ssl
import paho.mqtt.client as mqtt


# ==========================================================
# FUNÇÃO JSON-SAFE
# ==========================================================

def limpar_json(valor):
    if isinstance(valor, np.generic):
        return valor.item()
    if isinstance(valor, np.ndarray):
        return [limpar_json(v) for v in valor]
    if isinstance(valor, list):
        return [limpar_json(v) for v in valor]
    if isinstance(valor, tuple):
        return tuple(limpar_json(v) for v in valor)
    if isinstance(valor, dict):
        return {k: limpar_json(v) for k, v in valor.items()}
    return valor


# ==========================================================
# MQTT COM TLS — HIVE MQ CLOUD
# TÓPICO CUSTOMIZADO: TOPICO_MEIRINHOS
# ==========================================================

class MQTT:
    def __init__(self):
        self.client = mqtt.Client()

        self.client.tls_set(
            cert_reqs=ssl.CERT_REQUIRED,
            tls_version=ssl.PROTOCOL_TLS
        )

        self.client.username_pw_set("sentinela", "Sentinela123")

        self.client.connect(
            "bdffc9a5bf6e4bf28591393206fc27e0.s1.eu.hivemq.cloud",
            port=8883,
            keepalive=60
        )

        self.topico = "robot/drone"

    def publicar_evento(self, nome, valor):
        """Envia evento apenas quando solicitado."""
        payload = json.dumps({nome: limpar_json(valor)})
        self.client.publish(self.topico, payload)


# ==========================================================
# AMBIENTE
# ==========================================================

class AmbienteSimulacao:
    def __init__(self, gui=True, passo=1/240):
        self.passo = passo
        p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(passo)
        self.chao = p.loadURDF("plane.urdf")

    def atualizar(self):
        p.stepSimulation()
        time.sleep(self.passo)


# ==========================================================
# DRONE
# ==========================================================

class DroneSimples:
    def __init__(self, posicao_inicial=[0, 0, 1]):
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.05])
        vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.05], rgbaColor=[0, 0, 1, 1]
        )

        self.id = p.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=posicao_inicial
        )

        self.posicao_inicial = np.array(posicao_inicial)

    def obter_posicao(self):
        pos, _ = p.getBasePositionAndOrientation(self.id)
        return np.array(pos)


# ==========================================================
# SENSOR
# ==========================================================

class SensorPontos:
    def __init__(self, drone, raio=12):
        self.drone = drone
        self.raio = raio

    def detectar(self, pontos):
        pos_drone = self.drone.obter_posicao()
        det = []
        for pos, cid in pontos:
            if np.linalg.norm(pos - pos_drone) <= self.raio:
                det.append((pos, cid))
            if len(det) == 3:
                break
        return det


# ==========================================================
# CONTROLADOR SEM INÉRCIA
# ==========================================================

class ControladorDrone:
    def __init__(self, drone):
        self.drone = drone

        self.vel_xy = 1.2
        self.vel_z  = 0.8

        self.gxy = 1.8
        self.gz  = 1.4

    def seguir(self, alvo, dt):
        pos = self.drone.obter_posicao()

        # horizontal
        diff_xy = alvo[:2] - pos[:2]
        vel_xy = self.gxy * diff_xy

        norma = np.linalg.norm(vel_xy)
        if norma > self.vel_xy:
            vel_xy = vel_xy / norma * self.vel_xy

        # vertical
        erro_z = alvo[2] - pos[2]
        vel_z = np.clip(self.gz * erro_z, -self.vel_z, self.vel_z)

        vel = np.array([vel_xy[0], vel_xy[1], vel_z])

        p.resetBaseVelocity(self.drone.id, linearVelocity=vel.tolist())


# ==========================================================
# UTILIDADES
# ==========================================================

def criar_cubo(pos):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2])
    vis = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2],
        rgbaColor=[1, 0.3, 0.1, 1]
    )
    return p.createMultiBody(0, col, vis, pos)


def gerar_pontos(n=6, dist_min=3):
    pts = []
    xmin, xmax = -8, 8
    ymin, ymax = -8, 8

    while len(pts) < n:
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        z = 0.2

        novo = np.array([x, y, z])

        ok = True
        for p_old, _ in pts:
            if np.linalg.norm(novo - p_old) < dist_min:
                ok = False
                break

        if ok:
            cid = criar_cubo(novo)
            pts.append((novo, cid))

    return pts


def melhor_rota(inicio, pontos):
    melhor = None
    melhor_dist = 999999

    for perm in itertools.permutations(pontos):
        total = 0
        ant = inicio

        for p in perm:
            total += np.linalg.norm(ant - p)
            ant = p

        total += np.linalg.norm(ant - inicio)

        if total < melhor_dist:
            melhor_dist = total
            melhor = perm

    return list(melhor)


# ==========================================================
# LOOP PRINCIPAL
# ==========================================================

def main():

    ambiente = AmbienteSimulacao()
    drone     = DroneSimples()
    sensor    = SensorPontos(drone)
    control   = ControladorDrone(drone)
    mqtt      = MQTT()

    pontos = gerar_pontos(6, dist_min=3)

    rota = []
    rota_i = 0
    dt = ambiente.passo
    cubo_remover = None

    while True:

        pos = drone.obter_posicao()

        # Detectar cubos
        if not rota:
            detectados = sensor.detectar(pontos)
            if len(detectados) == 3:

                coords  = [d[0] for d in detectados]
                coords3 = [np.array([p[0], p[1], 1.5]) for p in coords]

                seq = melhor_rota(drone.posicao_inicial, coords3)
                seq.append(drone.posicao_inicial.copy())

                rota = seq
                rota_i = 0

        # Seguir rota
        if rota:
            alvo = rota[rota_i]
            control.seguir(alvo, dt)

            if np.linalg.norm(pos[:2] - alvo[:2]) < 0.25:

                cubo_mais_prox = None
                dist = 999

                for ppos, cid in pontos:
                    d = np.linalg.norm(ppos[:2] - alvo[:2])
                    if d < dist:
                        dist = d
                        cubo_mais_prox = (ppos, cid)

                # Encontrou cubo embaixo
                if cubo_mais_prox and dist < 1.0:
                    cubo_remover = cubo_mais_prox

                rota_i += 1
                if rota_i >= len(rota):
                    rota = []

        # Remover cubo + ENVIAR MQTT SOMENTE AQUI
        if cubo_remover:
            pos_cubo, cid = cubo_remover
            pontos = [pp for pp in pontos if pp[1] != cid]
            p.removeBody(cid)

            # ENVIO MQTT SOMENTE AO SUBIR EM CIMA DO CUBO
            mqtt.publicar_evento(
                "cubo_removido",
                {
                    "id": int(cid),
                    "posicao_cubo": limpar_json(pos_cubo),
                    "posicao_drone": limpar_json(drone.obter_posicao())
                }
            )

            cubo_remover = None

        ambiente.atualizar()


if __name__ == "__main__":
    main()
