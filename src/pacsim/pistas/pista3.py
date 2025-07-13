import numpy as np
import matplotlib.pyplot as plt


def gerar_pista3(
    num_pontos=1000,
    comprimento_reta_inicial=40,
    raio_curva1=15,
    angulo_curva1=np.pi / 2,
    comprimento_reta_entre_curvas=30,
    raio_curva2=20,
    comprimento_reta_final=20,
):
    n_reta_inicial = int(num_pontos * 0.1)
    n_curva1 = int(num_pontos * 0.25)
    n_reta_entre = int(num_pontos * 0.2)
    n_curva2 = int(num_pontos * 0.3)
    n_reta_final = num_pontos - n_reta_inicial - n_curva1 - n_reta_entre - n_curva2

    x_reta_inicial = np.linspace(0, comprimento_reta_inicial, n_reta_inicial)
    y_reta_inicial = np.zeros(n_reta_inicial)

    theta1 = np.linspace(-np.pi / 2, -np.pi / 2 + angulo_curva1, n_curva1)
    cx1 = comprimento_reta_inicial
    cy1 = raio_curva1
    x_curva1 = cx1 + raio_curva1 * np.cos(theta1)
    y_curva1 = cy1 + raio_curva1 * np.sin(theta1)

    tangente1 = theta1[-1]
    dx1 = -np.sin(tangente1)
    dy1 = np.cos(tangente1)
    x1_final = x_curva1[-1]
    y1_final = y_curva1[-1]

    t_entre = np.linspace(0, comprimento_reta_entre_curvas, n_reta_entre)
    x_reta_entre = x1_final + dx1 * t_entre
    y_reta_entre = y1_final + dy1 * t_entre

    theta2 = np.linspace(tangente1, tangente1 + np.pi, n_curva2)
    cx2 = x_reta_entre[-1] + raio_curva2 * -dy1
    cy2 = y_reta_entre[-1] + raio_curva2 * dx1
    x_curva2 = cx2 + raio_curva2 * np.cos(theta2)
    y_curva2 = cy2 + raio_curva2 * np.sin(theta2)

    tangente2 = theta2[-1]
    dx2 = -np.sin(tangente2)
    dy2 = np.cos(tangente2)
    x2_final = x_curva2[-1]
    y2_final = y_curva2[-1]

    t_final = np.linspace(0, comprimento_reta_final, n_reta_final)
    x_reta_final = x2_final + dx2 * t_final
    y_reta_final = y2_final + dy2 * t_final

    x = np.concatenate([x_reta_inicial, x_curva1, x_reta_entre, x_curva2, x_reta_final])
    y = np.concatenate([y_reta_inicial, y_curva1, y_reta_entre, y_curva2, y_reta_final])

    pista = [{"x": float(xi), "y": float(yi)} for xi, yi in zip(x, y)]
    return pista


def visualizar_pista(pista):
    x = [p["x"] for p in pista]
    y = [p["y"] for p in pista]
    plt.figure(figsize=(10, 7))
    plt.plot(x, y, color="black", linewidth=2)
    plt.axis("equal")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    pista = gerar_pista3()
    visualizar_pista(pista)
