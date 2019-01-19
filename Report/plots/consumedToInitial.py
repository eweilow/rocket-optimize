from matplotlib import pyplot as plt
from matplotlib import rcParams
import numpy as np

rcParams.update({'font.size': 16})

arrowprops = dict(arrowstyle="->")


deltaVOverExhaust = np.linspace(0, 3, 500)

massRatio = np.exp(deltaVOverExhaust)

mFinal = 4000 + 10000 + 2000
mInitial1 = massRatio * mFinal
vExhaust = 3481
initial = np.exp(-deltaVOverExhaust)
plt.plot(deltaVOverExhaust*vExhaust, mInitial1)

mFinal = 4000 + 10000
mInitial2 = massRatio * mFinal
vExhaust = 3481
initial = np.exp(-deltaVOverExhaust)
plt.plot(deltaVOverExhaust*vExhaust, mInitial2)

plt.figlegend(('Final mass 16 tons', 'Final mass 14 tons'),
              loc='upper left', bbox_to_anchor=(0.25, 0.9))

plt.xlabel("$\\Delta v$ (m/s)")
plt.ylabel("Initial mass (kg)")
plt.yticks(np.linspace(0, 300000, 7))
plt.tight_layout()
plt.savefig("./consumedToInitial.png", dpi=160)

plt.figure()
plt.plot(deltaVOverExhaust*vExhaust, mInitial1 - mInitial2)

plt.xlabel("$\\Delta v$ (m/s)")
plt.ylabel("Mass difference (kg)")
plt.tight_layout()

plt.savefig("./consumedToInitial_diff.png", dpi=160)
