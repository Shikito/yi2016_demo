import math
import numpy as np
from matplotlib import pyplot as plt

class TargetObject:
    def __init__(self, surface_shape=None):
        if surface_shape is not None:
            self.surface_shape = surface_shape
        else:
            self.surface_shape = np.vectorize(
                self._yi2016_toy_func,
                otypes=[float]
            )
    
    def _yi2016_toy_func(self, x):
        if x < -2 or x > 2:
            raise ValueError("x must be -2 <= x <= 2")
        if x >= -1 and x <= 1:
            omega = 4 * math.pi
            return math.sin(omega*x) + 1
        else:
            return 1
    
def _main():
    target_object = TargetObject()
    print(target_object.surface_shape(2))
    print(target_object.surface_shape(-2))
    print(target_object.surface_shape(0))

    x = np.linspace(-2, 2, num=1000)
    print(x)

    y = target_object.surface_shape(x)
    plt.figure()
    plt.plot(x, y)
    plt.show()

if __name__=="__main__":
    _main()