import matplotlib.pyplot as plt
import numpy as np


def bezier_cubica(P0, P1, P2, P3, n=100):
    t = np.linspace(0, 1, n)[:, None]
    B = (
        (1 - t) ** 3 * P0
        + 3 * (1 - t) ** 2 * t * P1
        + 3 * (1 - t) * t**2 * P2
        + t**3 * P3
    )
    return B[:, 0], B[:, 1]


def gerar_pista2(
    comprimento=60,
    amplitude=5,
    num_ciclos=3,
    num_pontos=500,
    comprimento_reta=15,
    num_pontos_reta=100,
    raio_curva=40,
    num_pontos_curva=150,
    angulo_curva=np.deg2rad(90),
):

    x = np.linspace(0, comprimento, num_pontos)
    y = amplitude * np.sin(2 * np.pi * num_ciclos * x / comprimento)

    x_fim = x[-1]
    y_fim = y[-1]
    derivada = (
        (2 * np.pi * num_ciclos / comprimento)
        * amplitude
        * np.cos(2 * np.pi * num_ciclos * x_fim / comprimento)
    )
    m = derivada

    dx = 1 / np.sqrt(1 + m**2)
    dy = m / np.sqrt(1 + m**2)
    t = np.linspace(0, comprimento_reta, num_pontos_reta)
    x_reta = x_fim + dx * t
    y_reta = y_fim + dy * t

    P0 = np.array([x_reta[-1], y_reta[-1]])
    v = np.array([dx, dy])

    R = np.array(
        [
            [np.cos(angulo_curva), -np.sin(angulo_curva)],
            [np.sin(angulo_curva), np.cos(angulo_curva)],
        ]
    )
    v_rot = R @ v

    P1 = P0 + v * raio_curva * 0.5
    P3 = P0 + v_rot * raio_curva
    P2 = P3 - v_rot * raio_curva * 0.5

    x_curva, y_curva = bezier_cubica(P0, P1, P2, P3, n=num_pontos_curva)

    x_total = np.concatenate([x, x_reta, x_curva])
    y_total = np.concatenate([y, y_reta, y_curva])
    pista = [{"x": float(xi), "y": float(yi)} for xi, yi in zip(x_total, y_total)]
    return pista


def visualizar_pista(pista):
    x = [p["x"] for p in pista]
    y = [p["y"] for p in pista]
    plt.figure(figsize=(10, 6))
    plt.plot(x, y, color="black", linewidth=2)
    plt.axis("equal")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    pista = gerar_pista2()
    visualizar_pista(pista)
