import math
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
mpl.rcParams['font.size'] = 20

# 1) Define shape of a single forcelet
f_obs_i = lambda d, diff, b1, b2, sig: b1 * np.exp(-d/b2) * diff * np.exp(-(diff**2) / (2*sig**2))

phis = np.linspace(-math.pi, math.pi, 100)  # pi- psi can go from -180 to 180
beta1s = np.arange(0, 10, 2)  # magnitude of weights
beta2s = [1,2,5,10]  # curvature, the bigger the larger distance also matters
sigmas = [0.1, 0.5, 1, 2, 5]  # locality

d = 16  # d mm close
b1 = 2.5
b2 = 5
sig = 1

# # sigma, b1
# for param in beta2s:
#     # plt.axhline(y=0.1, xmin=-math.pi, xmax=math.pi, color='black')
#     # plt.axhline(y=-0.1, xmin=-math.pi, xmax=math.pi, color='black')
#     plt.plot(phis, f_obs_i(d, phis, b1, param, sig), label=param)
#     plt.legend()
#     plt.title('effect of $\\beta_2$')
#     plt.xticks([-np.pi, 0, np.pi], ['$-\pi$', '0', '$\pi$'])
#     plt.xlabel('$\Phi - \Psi$ (rad)')
#     plt.ylabel('$f_{obs}$')
# plt.savefig('param_beta2_d=16_b1=0.2_sig=1.eps', bbox_inches='tight')
# plt.show()

# # b2
# dists = np.arange(0, 50)
# phi = 1
# for param in beta2s:
#     plt.plot(dists, f_obs_i(dists, phi, b1, param, sig), label=param)
#     plt.legend()
#     plt.title('effect of $\\beta_2$')
#     plt.xlabel('distance (mm)')
#     plt.ylabel('$f_{obs}$')
# plt.savefig('param_beta2_phis=1_b1=0.2_sig=1.eps', bbox_inches='tight')
# plt.show()


# # 2) Add multiple forcelets to see bifurcation effect
# mpl.rcParams['font.size'] = 20
# lamda = 0.05
# f_tar = lambda phis: - lamda * np.sin(phis)
#
# dists = 20
# # psis = [[np.radians(-13), np.radians(13)],  # 4 cm: repellor
# #         [np.radians(-45), np.radians(45)]]  # 8 cm: attractor
# psis = [[np.radians(-10), np.radians(10)],  # repellor
#         [np.radians(-37), np.radians(37)],  # critical point
#         [np.radians(-45), np.radians(45)]]  # attractor
# tar = f_tar(phis)
#
# fig, ax = plt.subplots(nrows=3, sharey=True, sharex=True, figsize=(10,20))
#
# for i, case in enumerate(psis):
#     obss = []
#     for psi in case:
#         obss.append(f_obs_i(dists, phis - psi, b1, b2, sig))
#
#     ax[i].plot(phis, tar, 'gray')
#     [ax[i].plot(phis, obs_i, 'gray') for obs_i in obss]
#     ax[i].plot(phis, tar + np.sum(obss, axis=0), 'red')
#     ax[i].axhline(y=0, xmin=-np.pi, xmax=np.pi, color='black')
#
# ax[0].set(xticks=[-np.pi, -np.pi/2, 0, np.pi/2, np.pi], xticklabels=['$-\pi$', '$-0.5 \pi$', 0, '$0.5 \pi$', '$\pi$'],
#           xlabel='$\Phi$ ')
# ax[0].xaxis.set_label_coords(1.05, 0.55)
# ax[0].yaxis.set_label_coords(0, 1)
# ax[0].set_ylabel('$\\frac {d \Phi} {dt}$', rotation=0)
#
# # plt.savefig('bifurcation.eps')
# plt.show()


# 3) Plot the trace
trace_algo_4 = np.array(np.load('trace/algo/trace_4cm.pickle', allow_pickle=True))
trace_algo_8 = np.array(np.load('trace/algo/trace_8cm.pickle', allow_pickle=True))
trace_dyn_4 = np.array(np.load('trace/trace_4cm.pickle', allow_pickle=True))
trace_dyn_8 = np.array(np.load('trace/trace_8cm.pickle', allow_pickle=True))
print(trace_dyn_4)

fig, ax = plt.subplots(nrows=2, ncols=2, sharex=True, sharey=True, figsize=(14, 20))

# traces
ax[0, 0].plot(trace_algo_4[:,0], trace_algo_4[:,1])
ax[0, 1].plot(trace_algo_8[:,0], trace_algo_8[:,1])
ax[1, 0].plot(trace_dyn_4[:,0], trace_dyn_4[:,1])
ax[1, 1].plot(trace_dyn_8[:,0], trace_dyn_8[:,1])

# labels
ax[0, 0].xaxis.set_label_coords(0.5, 1.1)
ax[0, 0].set(xlabel='4cm', ylabel='algorithmic')
ax[0, 1].xaxis.set_label_coords(0.5, 1.1)
ax[0, 1].set(xlabel='8cm')
ax[1, 0].set(ylabel='dynamic')

for i in range(2):
    # obstacle
    ax[i, 0].add_patch(mpl.patches.Rectangle((20, 170), 100, 50))
    ax[i, 0].add_patch(mpl.patches.Rectangle((160, 170), 100, 50))
    ax[i, 1].add_patch(mpl.patches.Rectangle((0, 170), 100, 50))
    ax[i, 1].add_patch(mpl.patches.Rectangle((180, 170), 100, 50))
    for j in range(2):
        ax[i, j].plot(230, 360, 'o', color='black')  # target

        # ax[i,j].set(xlim=(280, 0), ylim=(0, 410))  # sheet
plt.savefig('trajectory_comparison.eps', bbox_inches='tight')
plt.show()
