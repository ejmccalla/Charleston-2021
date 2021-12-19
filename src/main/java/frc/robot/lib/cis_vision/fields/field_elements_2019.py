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

rocket_1 = [
    # x-coordinate in inches                                                        y-coordinate in inches
    ( 220.25 - 19.625 / np.tan(np.radians(61.25)),                                  0                           ),
    ( 0,                                                                            27.5 - 19.625               ),
    ( 19.625 / np.tan(np.radians(61.25)),                                           27.5                        ),
    ( 19.625 / np.tan(np.radians(61.25)) + 2 * (229.125 - 220.25),                  27.5                        ),
    ( 2 * 19.625 / np.tan(np.radians(61.25)) + 2 * (229.125 - 220.25),              27.5 - 19.625               ),
    ( 2 * 19.625 / np.tan(np.radians(61.25)) + 2 * (229.125 - 220.25),              0                           ),
    ( 0,                                                                            0                           )]

rocket_2 = [
    # x-coordinate in inches                                                        y-coordinate in inches
    ( 54 * 12 - 229.13 - (229.13 - 220.25) - 19.625 / np.tan(np.radians(61.25)),    0                           ),
    ( 0,                                                                            27.5 - 19.625               ),
    ( 19.625 / np.tan(np.radians(61.25)),                                           27.5                        ),
    ( 19.625 / np.tan(np.radians(61.25)) + 2 * (229.125 - 220.25),                  27.5                        ),
    ( 2 * 19.625 / np.tan(np.radians(61.25)) + 2 * (229.125 - 220.25),              27.5 - 19.625               ),
    ( 2 * 19.625 / np.tan(np.radians(61.25)) + 2 * (229.125 - 220.25),              0                           ),
    ( 0,                                                                            0                           )]

rocket_3 = [
    # x-coordinate in inches                                                        y-coordinate in inches
    ( 220.25 - 19.625 / np.tan(np.radians(61.25)),                                  27*12                       ),
    ( 0,                                                                            -(27.5 - 19.625)            ),
    ( 19.625 / np.tan(np.radians(61.25)),                                           -27.5                       ),
    ( 19.625 / np.tan(np.radians(61.25)) + 2 * (229.125 - 220.25),                  -27.5                       ),
    ( 2 * 19.625 / np.tan(np.radians(61.25)) + 2 * (229.125 - 220.25),              -(27.5 - 19.625)            ),
    ( 2 * 19.625 / np.tan(np.radians(61.25)) + 2 * (229.125 - 220.25),              0                           ),
    ( 0,                                                                            0                           )]

rocket_4 = [
    # x-coordinate in inches                                                        y-coordinate in inches
    ( 54 * 12 - 229.13 - (229.13 - 220.25) - 19.625 / np.tan(np.radians(61.25)),    27*12                       ),
    ( 0,                                                                            -(27.5 - 19.625)            ),
    ( 19.625 / np.tan(np.radians(61.25)),                                           -27.5                       ),
    ( 19.625 / np.tan(np.radians(61.25)) + 2 * (229.125 - 220.25),                  -27.5                       ),
    ( 2 * 19.625 / np.tan(np.radians(61.25)) + 2 * (229.125 - 220.25),              -(27.5 - 19.625)            ),
    ( 2 * 19.625 / np.tan(np.radians(61.25)) + 2 * (229.125 - 220.25),              0                           ),
    ( 0,                                                                            0                           )]

cargo_ship = [
    # x-coordinate in inches                                                        y-coordinate in inches
    ( 27 * 12,                                                                      (27 * 12) / 2 - 55.75 / 2   ),
    ( -(95.75 + 9),                                                                 0                           ),
    ( -(95.75 + 9),                                                                 55.75                       ),
    ( 0,                                                                            55.75                       ),
    ( (95.75 + 9),                                                                  55.75                       ),
    ( (95.75 + 9),                                                                  0                           ),
    ( 0,                                                                            0                           ),]

hab_1 = [
    # x-coordinate in inches                                                        y-coordinate in inches
    ( 0,                                                                            (27 * 12) / 2               ),
    ( 0,                                                                            -64                         ),
    ( 95.25,                                                                        -64                         ),
    ( 95.25,                                                                        64                          ),
    ( 0,                                                                            64                          ),
    ( 0,                                                                            0                           ),]

hab_2 = [
    # x-coordinate in inches                                                        y-coordinate in inches
    ( 54*12,                                                                        (27 * 12) / 2               ),
    ( 0,                                                                            -64                         ),
    ( -95.25,                                                                       -64                         ),
    ( -95.25,                                                                       64                          ),
    ( 0,                                                                            64                          ),
    ( 0,                                                                            0                           ),]

field_elements_list_in = [ rocket_1, rocket_2, rocket_3, rocket_4, cargo_ship, hab_1, hab_2 ]
