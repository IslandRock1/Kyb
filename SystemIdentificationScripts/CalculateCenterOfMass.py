

"""

This file calculates the distance from the four-stroke sensor to the center of mass
Using the centre jog, and the cylinder.

"""

import numpy as np





def getCenterOfMass() -> tuple[float, np.matrix]:

    mass = 0.602

    # x/y = 0.029
    # z = 0.053
    delta = 0.019
    COG = np.matrix([
        [-delta],
        [-delta],
        [0.053]
    ])

    return (mass, COG)

