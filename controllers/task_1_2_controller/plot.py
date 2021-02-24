import numpy as np
import matplotlib.pyplot as plt
import pickle
from scipy.optimize import curve_fit
from sklearn.linear_model import LinearRegression

# load data
data = np.loadtxt("measurement.txt")
data = np.reshape(data, (4,10))  #  4 distance x 10 repeat

# [0,1] normalize
print(np.max(data), np.min(data))
data = (data - np.min(data)) / (np.max(data) - np.min(data))

# params
num_dist = 4
max_dist = 19  # in mm
min_dist = 4
dist = np.linspace(min_dist, max_dist, num_dist)  # distance between obstacle and robot

# estimate
mean = np.mean(data, axis=1)
std = np.std(data, axis=1)

dist_func = lambda v, a, b: a * np.log(v) + b
val_func = lambda d, a, b: np.exp((d-b) / a)

reg = LinearRegression().fit(np.log(mean).reshape(-1,1), dist)
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
plt.savefig('sensor.eps')
