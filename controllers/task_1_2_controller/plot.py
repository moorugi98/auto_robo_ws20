import numpy as np
import matplotlib.pyplot as plt
import pickle
from sklearn.linear_model import LinearRegression

# load data
data = np.loadtxt("measurement.txt")
data = np.reshape(data, (4, 10))  # 4 distances x 10 repetition

# normalize to [0,1]
print(np.max(data), np.min(data))
data = (data - np.min(data)) / (np.max(data) - np.min(data))
# # normalize to an and bn
# an = 10
# bn = 20
# data = (bn-an) * (data - np.min(data)) / (np.max(data) - np.min(data))  + an

# params used in measurements
num_dist = 4
max_dist = 19  # in mm
min_dist = 4
dist = np.linspace(min_dist, max_dist, num_dist)  # distances between obstacle and robot

# estimate
mean = np.mean(data, axis=1)
std = np.std(data, axis=1)

dist_func = lambda v, a, b: a * np.log(v) + b  # v to d
val_func = lambda d, a, b: np.exp((d-b) / a)  # d to v

reg = LinearRegression().fit(np.log(mean).reshape(-1, 1), dist)  # d = a lnv + b  so ln(v) as the independent var
a_hat = reg.coef_[0]
b_hat = reg.intercept_
print(a_hat, b_hat)

# Plot
plt.fill_between(dist, mean-std, mean+std, color='gray')
plt.plot(dist, mean, 'black', label='data')
plt.plot(dist, val_func(dist, a_hat, b_hat), 'green', label='fit')
plt.legend()
plt.xlabel('dist (mm)')
plt.ylabel('value')
plt.title('sensor reaction to distance')
# plt.savefig('sensor.eps')
