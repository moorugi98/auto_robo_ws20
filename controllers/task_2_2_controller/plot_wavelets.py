import math
import numpy as np
import matplotlib.pyplot as plt

f_obs_i = lambda d, pi, b1, b2, sig: b1 * np.exp(-d/b2) * pi * np.exp(-pi**2 / (2*sig**2))



pis = np.linspace(-math.pi, math.pi, 100)  # pi- psi can go from -180 to 180
beta1s = [0, 1, 10]  # magnitude
beta2s = [1, 10, 100]  # curvature
sigmas = [0.1, 1, 10]  # locality

d = 10  # 10mm close
b1 = 1
b2 = 10
sig = 1
for param in beta1s:
    plt.axhline(y=0.1, xmin=-math.pi, xmax=math.pi, color='black')
    plt.plot(pis, f_obs_i(d, pis, param, b2, sig), label=param)
    plt.legend()
    plt.title('effect of $ \\beta_1$')
    plt.xlabel('$\| \Phi - \Psi \|$ (rad)')
    plt.ylabel('$f_{obs}$')

plt.savefig('beta1.eps')
