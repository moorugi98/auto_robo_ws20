import numpy as np
import matplotlib.pyplot as plt
import pickle
from scipy.optimize import curve_fit
from sklearn.linear_model import LinearRegression

# load data
with open("test.txt", "rb") as fp:   # Unpickling
    data = pickle.load(fp)

# params
num_dist = 20
obstacle = 190
max_pos = 140
start_pos = 60

# estimate
mean = np.mean(data, axis=0)
std = np.std(data, axis=0)
xvals = np.linspace(obstacle-start_pos, obstacle-max_pos, num_dist)

estim = lambda x, a, b: -a * np.log(x) - b
print(xvals.shape, mean.shape)
optparam, _ = curve_fit(estim, xvals, mean)
a_hat = optparam[0]
b_hat = optparam[1]
print(a_hat, b_hat)

reg = LinearRegression().fit(np.log(xvals).reshape(-1,1), mean)
print(reg.coef_, reg.intercept_)

# Plot
plt.fill_between(xvals, mean-std, mean+std, color='gray')
plt.plot(xvals, mean, 'black')
plt.plot(xvals, estim(mean, -a_hat, -b_hat), 'green')
plt.xlabel('dist (mm)')
plt.ylabel('value')
plt.title('sensor reaction to distance')
plt.savefig('sensor.eps')
