
import numpy as np

def getCenterOfMass() -> tuple[float, np.matrix]:

    mass = 0.262

    delta = 0.018
    COG = np.matrix([
        [-delta],
        [-delta],
        [0.05]
    ])

    return (mass, COG)

