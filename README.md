# Triangulation
Triangulates source given three receivers and plots in 3D

The incident and azimuth angles of a incident signal onto three receivers is used to estimate the distance of the source.

The algorithm involves calculating the nearest point as the intersection of line_x with a plane that includes line_y and a vector normal to lines x and y. 

The incident and azimuth angles are randomly changed here, but sould be readings from a sensor.
