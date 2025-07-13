import matplotlib.pyplot as plt
import numpy as np


def gerar_pista1(
    comprimento_curva=100, amplitude=20, num_pontos_curva=200, comprimento_retas=120
):
    x_curva = np.linspace(0, comprimento_curva, num_pontos_curva)
    y_curva = amplitude * np.sin(2 * np.pi * x_curva / comprimento_curva)

    dx = np.gradient(x_curva)
    dy = np.gradient(y_curva)
    tx_ini = dx[0]
    ty_ini = dy[0]
    norma_ini = np.sqrt(tx_ini**2 + ty_ini**2)
    tx_ini /= norma_ini
    ty_ini /= norma_ini

    x_ini = x_curva[0]
    y_ini = y_curva[0]

    t_entrada = np.linspace(
        -comprimento_retas,
        0,
        int(num_pontos_curva * comprimento_retas / comprimento_curva),
    )
    x_entrada = x_ini + tx_ini * t_entrada
    y_entrada = y_ini + ty_ini * t_entrada

    tx_fim = dx[-1]
    ty_fim = dy[-1]
    norma_fim = np.sqrt(tx_fim**2 + ty_fim**2)
    tx_fim /= norma_fim
    ty_fim /= norma_fim

    x_fim = x_curva[-1]
    y_fim = y_curva[-1]

    t_saida = np.linspace(
        0,
        comprimento_retas,
        int(num_pontos_curva * comprimento_retas / comprimento_curva),
    )
    x_saida = x_fim + tx_fim * t_saida
    y_saida = y_fim + ty_fim * t_saida

    x_total = np.concatenate([x_entrada, x_curva, x_saida])
    y_total = np.concatenate([y_entrada, y_curva, y_saida])

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
    pista = gerar_pista1()
    visualizar_pista(pista)
