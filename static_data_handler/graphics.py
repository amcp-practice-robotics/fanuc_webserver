import numpy as np
import matplotlib.pyplot as plt
import matplotlib

def grafics(outs):
    outs_list = outs.fetchall()
    outs_array = np.array(outs_list)
    outs_array = outs_array[:, 1:]
    n = 10  # количество используемых данных
    fanuc_xyzwpr = outs_array[:n, 6:12]
    our_xyzwpr = outs_array[:n, 12:]
    error = fanuc_xyzwpr - our_xyzwpr

    coord = np.array(['x', 'y', 'z', 'w', 'p', 'r'])

    for i in range(n):
        plt.title("___")
        plt.xlabel("Координаты")
        plt.ylabel("Ошибка")
        plt.plot(coord, error[i], color="green")
        plt.show()

    x = np.arange(1, n + 1)

    figure, axis = plt.subplots(3,2)
    plt.subplots_adjust(hspace=0.7)# right=0.95, bottom=0.3, top=0.95)
    col = 0
    line = 0
    for i in range(6):
        axis[line, col].set_xlabel("___")
        axis[line, col].set_ylabel("Ошибка")
        axis[line, col].set_title(f"Ошибка {coord[i]} для {n} вычислений")
        axis[line, col].plot(x, error[:, i], color="orange")
        line += 1
        if i == 2:
            line = 0
            col += 1
    plt.show()

    error_norm = []

    for i in range(6):
        error_norm.append(np.linalg.norm(error[:, i]))
    plt.title("___")
    plt.bar(coord, error_norm, color="green")
    plt.show()