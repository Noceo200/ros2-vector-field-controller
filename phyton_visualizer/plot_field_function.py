from tools import *

def clamp_not_zero(val,min_lim,max_lim,null_radius): #values should be norms > 0.0
    new_val=val
    if val > max_lim:
        new_val = max_lim
    elif val < min_lim and val>null_radius:
        new_val = min_lim
    elif val < null_radius:
        new_val = 0.0
    return new_val

def f(x):
    attr = clamp_not_zero(x**3,0.1,1.0,0.0)
    rep = clamp_not_zero(2*exp(-x),0.0,1.0,0.0)
    return -attr+rep

def y(x):
    res = []
    for x_ in x:
        res.append(abs(f(x_)))
    return res

x = np.arange(0,6,0.1)
fx = y(x)

plt.plot(x,fx)

# Set the maximum values for x and y axes
plt.xlim(0, 6.0)  # replace max_x_value with your desired maximum x-axis value
plt.ylim(0, 2.5)  # replace max_y_value with your desired maximum y-axis value

plt.show()
