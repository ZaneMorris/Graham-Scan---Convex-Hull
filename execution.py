#%% Imports
import numpy as np
import Graham
#%% Running Graham
points = np.random.uniform(low=-10, high=10, size=(25,2))
perim = Graham.Graham(points)
