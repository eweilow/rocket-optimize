from matplotlib import pyplot as plt
from matplotlib import rcParams
import numpy as np

rcParams.update({'font.size': 16})

arrowprops = dict(arrowstyle="->")

t = []
angle = []
thrust = []

t.extend(np.linspace(0, 5, 10))
angle.extend(np.linspace(0, 0, 10))
thrust.extend(np.linspace(1, 1, 10))

t.extend(np.linspace(5, 55, 10))
angle.extend(np.linspace(0, 4, 10))
thrust.extend(np.linspace(1, 1, 10))

t.extend(np.linspace(55, 165, 10))
angle.extend(np.linspace(4, 75, 10))
thrust.extend(np.linspace(1, 1, 10))

t.extend(np.linspace(165, 440, 10))
angle.extend(np.linspace(75, 90, 10))
thrust.extend(np.linspace(1, 1, 10))

plt.plot(t, angle)
plt.annotate(
    "($t_{vertical}$, 0°)",
    (5, 0),
    xytext=(-15, 100),
    textcoords='offset points',
    arrowprops=arrowprops
)
plt.annotate(
    "($t_{kickpitch}$, $\\theta_{kickpitch}$)",
    (55, 4),
    xytext=(25, 0),
    textcoords='offset points',
    arrowprops=arrowprops
)
plt.annotate(
    "($t_{staging}$, $\\theta_{staging}$)",
    (165, 75),
    xytext=(-125, 25),
    textcoords='offset points',
    arrowprops=arrowprops
)
plt.xticks([0])
plt.yticks([0, 90])
plt.xlabel("Time")
plt.ylabel("Thrust angle from vertical")
plt.tight_layout()
plt.savefig("./angleFromVertical.png", dpi=160)
