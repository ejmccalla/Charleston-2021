import numpy as np

'''
The field elements are defined by their vertices and are in units of inches.
Furthermore, the vertices are relative to the field coordinate system where
the x-coordinate points away from the drivers station wall (increasing as
you move away from the wall) and the y-coordinate is coincident with the
wall (increasing from left-to-right).

To build the vertices of the field elements, use the following steps:
  
    1. Define the starting vertice (relative to the field coordinate system)
    2. Define a vertex connected to the starting vertex (relative to the
       starting vertex)
    3. Define a vertex connected to the vertex defined in step 2 (relative
       to the vertex defined in step 2)
    4. Repeat step 3 (defining a new connected vertex relative to the prior
       vertex) until all of the vertices have been defined.
'''

obstacle_1 = [
    # x-coordinate in inches                                                        y-coordinate in inches
    ( 8*12,                                                                         (17.5 * 12) / 2             ),
    ( 0,                                                                            -1*12                       ),
    ( 8*12,                                                                         -1*12                       ),
    ( 8*12,                                                                         1*12                        ),
    ( 0,                                                                            1*12                        ), ]

field_elements_list_in = [ obstacle_1 ]
