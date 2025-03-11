import numpy as np
import matplotlib.pyplot as plt
from filter import *

if __name__ == '__main__':
    T = 10000                     # number of samples
    Vpp = 5                       # peak-to-peak voltage
    noise_perc = 0.03             # noise percentage
    mu, sigma = 0, noise_perc*Vpp # mean and standard deviation
    noise = np.random.normal(mu, sigma, T)
    
    th = np.linspace(0, 4*np.pi, T)
    signal = (Vpp/2)*np.sin(th)
    real_signal = signal + noise
    
    N = 10 # buffer size
    w = [0.01,0.01,0.02,0.03,0.05,0.08,0.13,0.17,0.21,0.29]

    x = [0.0]*N # measurement buffer
    y_ma = [] # mobile average filtered measure
    y_wma = [] # weighted mobile average filtered measure

    t = 0 # time
    while t < len(real_signal):
        if len(x) >= N:
            x.pop(0)
        x.append(real_signal[t])
        y_ma.append(ma_fir(x))
        y_wma.append(wma_fir(x,w))
        t += 1
    
    plt.figure()
    plt.subplot(211)
    plt.plot(real_signal)
    plt.plot(y_ma)
    plt.subplot(212)
    plt.plot(real_signal)
    plt.plot(y_wma)
    plt.show()