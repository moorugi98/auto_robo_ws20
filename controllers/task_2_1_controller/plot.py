import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

mpl.rcParams['font.size'] = 20


# # 1) Time evolution plot
# phis = np.loadtxt('angle_trace.txt')
# plt.plot(phis, 'black')
# plt.axhline(y=1.05969424, xmin=0, xmax=phis.shape[0], color='gray')
# plt.xlabel('time (step)')
# plt.ylabel('$\Phi$ (rad)')
#
# plt.savefig('time evolution plot.eps', bbox_inches='tight')
# plt.show()


# # 2) Trace plot with extreme lambdas
# trace = []
# trace.append(np.array(np.load('trace_lamda=0.0001.pickle', allow_pickle=True)))
# trace.append(np.array(np.load('trace_lamda=0.1.pickle', allow_pickle=True)))
# trace.append(np.array(np.load('trace_lamda=10.pickle', allow_pickle=True)))
#
# fig, ax = plt.subplots(ncols=3, figsize=(21, 10), sharey=True, sharex=True)
# for i, tr in enumerate(trace):
#     ax[i].plot(tr[:, 0], tr[:, 1], 'black', linewidth=3)  # trace
#     ax[i].plot(70, 340, 'o', color='black')  # target
#     ax[i].set(xlim=(280, 0), ylim=(0, 410))
# ax[0].set_title('$\lambda_{tar}=0.0001$')
# ax[1].set_title('$\lambda_{tar}=0.1$')
# ax[2].set_title('$\lambda_{tar}=10$')
# plt.savefig('trace.eps', bbox_inches='tight')
# plt.show()
