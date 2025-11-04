

"""

This file calculates the distance from the four-stroke sensor to the center of mass
Using the centre jog, and the cylinder.

"""

import numpy as np

mass_disk = 0.000332726
height_disk = 0.01

mass_cylinder = 0.00101901
height_cylinder = 0.09

mass_weight = 0.271
height_weight = 0.04475 * 2



def getCenterOfMass() -> tuple[float, np.matrix]:

    mass = 0.602

    # x/y = 0.029
    # z = 0.053
    delta = 0.029
    COG = np.matrix([
        [-delta],
        [-delta],
        [0.053]
    ])

    return (mass, COG)

centerOfMass = calculateCenterOfMass()
print(f"Centre of mass is {centerOfMass:.5f} meters from four-stroke sensor, or {centerOfMass * 100:.2f} cm.")
print(f"With modified weight (battery), the center of mass is 6cm from the sensor, with a weight of 0.527 kg.")