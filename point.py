import math
import numpy as np

if __name__ == "__main__":
    start = [0.0, 0.0, 0.5]
    end = [1.5, 0.0, .5]
    time = 4
    time_delta = 0.05
    num = math.floor(time/time_delta)

    print(np.linspace(start, end, num=num ,endpoint= True))
