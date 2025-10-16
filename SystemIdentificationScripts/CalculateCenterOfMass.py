

"""

This file calculates the distance from the four-stroke sensor to the center of mass
Using the centre jog, and the cylinder.

"""

mass_disk = 0.000332726
height_disk = 0.01

mass_cylinder = 0.00101901
height_cylinder = 0.09

mass_weight = 0.675
height_weight = 0.04475 * 2

def calculateCenterOfMass():
    t0 = (mass_disk * height_disk / 2)
    t1 = (mass_cylinder * (height_cylinder / 2 + height_disk))
    t2 = (mass_weight * (height_weight / 2 + height_disk))

    return (t0 + t1 + t2) / (mass_disk + mass_cylinder + mass_weight)

centerOfMass = calculateCenterOfMass()
print(f"Centre of mass is {centerOfMass:.5f} meters from four-stroke sensor, or {centerOfMass * 100:.2f} cm.")