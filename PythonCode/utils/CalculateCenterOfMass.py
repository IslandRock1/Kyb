
import numpy as np

def getCenterOfMass() -> tuple[float, np.matrix]:

    mass = 0.148 + 0.040

    # x/y = 0.029
    # z = 0.053
    delta = 0.010
    COG = np.matrix([
        [0],
        [0],
        [0.046]
    ])

    return (mass, COG)

