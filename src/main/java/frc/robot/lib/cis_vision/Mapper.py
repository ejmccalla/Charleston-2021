from fields.field import field
from AStarNode import AStarNode
from AStarGraph import AStarGraph
import cv2
import numpy as np

class Mapper ():
    '''A class used to map the field using a grid representation.

    The field map is constructed by first creating a 2D matrix representing the
    field with a resolution of 1 inch. Next, the field elements are added to
    the map. These represent the area of the map that the robot cannot go. The 
    field is then divided into grid sections by recursively dividing the field
    into quadrants until the minimum grid size is reached. Each grid element
    will be classified as either occupied (a field element is within the grid)
    or empty (robot is free to travel in this space). Finally, the grids are
    represented using a graph. Where each cell in the grid is a node and the
    inter-connects are the neighbors.

    Attributes:
        min_cell_size_in (`int`): The minimum size cell of the grid
            representation of the field map.
        field_map_in (:obj:`numpy.ndarray of numpy.uint8`): The map of the
            field.
        empty_grid (:obj:`numpy.ndarray of uint8`): The empty grid is an array
            of empty cells where each cell is the upper-left and lower-right
            points.
        occupied_grid (:obj:`numpy.ndarray of uint8`): The occupied grid is an
            array of empty cells where each cell is the upper-left and 
            lower-right points.
        adjacency_list (:obj:`list of lists of ints`): A list of empty cells
            and their neighbors.
        graph (:obj:`AStarGraph`): A graph of the connected nodes.
    '''
    def __init__ ( self, min_cell_size_in:int ):
        self.min_cell_size_in = min_cell_size_in
        self.field_map_in = np.zeros( ( int( field.width_ft * 12.0 ),
                                        int( field.length_ft * 12.0 ) ),
                                        dtype=np.uint8)
        self.field_map_in = self.add_field_elements_to_map( self.field_map_in )
        self.empty_grid, self.occupied_grid = self.get_field_grid()
        self.adjacency_list = self.get_adjacency_list()
        self.graph = self.build_graph()


    def add_field_elements_to_map ( self, field_map_in:np.ndarray ):
        '''The function will add the field elements to the map of the field.

        Args:
            field_map_in (:obj:`numpy.ndarray of numpy.uint8`): The map of the
                field.

        Returns:
            updated_map (:obj:`numpy.ndarray of numpy.uint8`) A map of the
                field with the field elements added.
        
        '''
        updated_map = field_map_in.copy()
        for element_in in field.elements_in:
            x_field_in = element_in[0][0]
            y_field_in = element_in[0][1]
            pts = [ ( int( x_field_in ), int( y_field_in ) ) ]
            for vertex_in in element_in[1:]:
                pts.append( ( int( ( x_field_in + vertex_in[0] ) ), 
                              int( ( y_field_in + vertex_in[1] ) ) ) )
            updated_map = cv2.fillPoly( updated_map, [ np.array( ( pts ) ) ], ( 255, 0, 0 ) )

        return updated_map


    def add_grid_to_map ( self, bgr_field_map_in:np.ndarray, grid:np.ndarray, 
                          color:tuple ):
        '''The function will add the grid representation to a map of the field.

        Args:
            bgr_field_map_in (:obj:`numpy.ndarray of numpy.uint8`): The map of
                the field in BGR format.
            grid (:obj:`numpy.ndarray of uint8`): The grid is an array of cells
                where each cell is the upper-left and lower-right points.
            color (:obj:`3-tuple of ints`): The RGB color of the grid.

        Returns:
            updated_map (:obj:`numpy.ndarray of numpy.uint8`) A map of the
                field with the grid representation added.
        
        '''
        updated_map = bgr_field_map_in.copy()
        for cell in grid:
            updated_map = cv2.rectangle( updated_map, cell[0], cell[1], color, 1 )
        
        return updated_map


    def add_adjacency_list_to_map ( self, field_map_in:np.ndarray,
                                    grid:np.ndarray, adjacency_list:list ):
        '''The function will add the adjacency list to a map of the field.

        Args:
            field_map_in (:obj:`numpy.ndarray of numpy.uint8`): The map of the
                field.
            grid (:obj:`numpy.ndarray of uint8`): The grid is an array of cells
                where each cell is the upper-left and lower-right points.
            adjacency_list (:obj:`list of list of ints`): A list of cells and
                the cells which are adjacent to them.

        Returns:
            updated_map (:obj:`numpy.ndarray of numpy.uint8`) A map of the
                field with the grid representation added.
        
        '''
        updated_map = field_map_in.copy()
        for cell_id, node in enumerate(adjacency_list):
            for adj_cell_id in node:
                pt1 = ( int( ( grid[cell_id][0][0] + grid[cell_id][1][0] ) / 2 ),
                        int( ( grid[cell_id][0][1] + grid[cell_id][1][1] ) / 2 ) )
                pt2 = ( int( ( grid[adj_cell_id][0][0] + grid[adj_cell_id][1][0] ) / 2 ),
                        int( ( grid[adj_cell_id][0][1] + grid[adj_cell_id][1][1] ) / 2 ) )
                updated_map = cv2.line( updated_map, pt1, pt2, ( 127, 127, 127 ), 1 )
        
        return updated_map


    def find_empty_cell ( self, pt1, pt2 ):
        '''The function will recursively build the grid representation of the
        field map.

        Args:
            pt1 (:obj:`tuple(float,float)): Upper-left vertex of the grid cell.
            pt2 (:obj:`tuple(float,float)): Lower-right vertex of the grid
                cell.

        Returns:
            grid (:obj:`list of :obj:`tuple(float,float,boolean)`): A list of
                the grid cells with the center coordinates (x,y) and a boolean
                flag to track if the cell is occupied (true is occupied).
        '''
        grid = []
        if (pt2[0] - pt1[0] ) / 2 >= self.min_cell_size_in:
            x_mid = int( ( pt2[0] + pt1[0] ) / 2 )
            y_mid = int( ( pt2[1] + pt1[1] ) / 2 )
            grid += self.find_empty_cell( pt1,             (x_mid, y_mid)  )
            grid += self.find_empty_cell( (x_mid, y_mid),  pt2             )
            grid += self.find_empty_cell( (pt1[0], y_mid), (x_mid, pt2[1]) )
            grid += self.find_empty_cell( (x_mid, pt1[1]), (pt2[0], y_mid) )
        elif np.sum( self.field_map_in[ pt1[1]:pt2[1], pt1[0]:pt2[0] ] ) == 0:
            grid.append( ( pt1, pt2, False ) )
        else:
            grid.append( ( pt1, pt2, True ) )

        return grid


    def get_field_grid ( self ):
        '''The function will build the grid representation of the field map.

        The purpose of this function, and not just using find_empty_cell, is
        this function will split the space into halves whereas find_empty_cell
        will split the space into quarters. This is to accomodate the situation
        where only half of the FRC is used for mapping. For example, some years
        the robots have been physically limited to only half of the field.

        Args:

        Returns:
            empty, occupied (:obj:`numpy.ndarray', :obj:`numpy.ndarray') A list
                of cells which are empty and a list of cells which are occupied.
        '''
        grid = []
        pt1 = ( 0, 0 )
        pt2 = ( int( field.length_ft * 12 / 2 ), int( field.width_ft * 12 ) )
        grid += self.find_empty_cell( pt1, pt2 )
        pt1 = ( int( field.length_ft * 12 / 2 ), 0)
        pt2 = ( int( field.length_ft * 12 ), int( field.width_ft * 12 ) )
        grid += self.find_empty_cell( pt1, pt2 )
        grid = np.array( grid, dtype=object )
        empty =  grid[ grid[:,2] == False ]
        occupied = grid[ grid[:,2] == True ]
        return ( empty, occupied )


    def get_adjacency_list ( self ):
        '''The function will find the neighbors of each of the nodes in the
        graph.

        Args:

        Returns:
            adjacency_list (:obj:`list of int'): A list of the neighboring Node
            ID's.
        '''
        adjacency_list = [ [] for _ in range( self.empty_grid.shape[0] ) ]
        for cell_id, act_cell in enumerate( self.empty_grid ):
            for adj_cell_id, adj_cell in enumerate( self.empty_grid ):
                if cell_id != adj_cell_id:
                    if ( ( act_cell[1][0] >= adj_cell[0][0] and act_cell[0][0] <= adj_cell[1][0] ) and 
                         ( act_cell[0][1] <= adj_cell[1][1] and act_cell[1][1] >= adj_cell[0][1] ) ):
                        adjacency_list[ cell_id ].append( adj_cell_id )

        return adjacency_list


    def build_graph (self):
        '''This function will build an A* graph represeentation of the playing field.

        Args:

        Returns:
            graph (:obj:`AStarGraph') An A* graph.        
        '''
        node_list = []
        for cell_id, neighbors in enumerate( self.adjacency_list ):
            mid_x = ( self.empty_grid[ cell_id ][0][0] + self.empty_grid[ cell_id ][1][0] ) / 2
            mid_y = ( self.empty_grid[ cell_id ][0][1] + self.empty_grid[ cell_id ][1][1] ) / 2
            pt1 = np.array( ( mid_x, mid_y ) )
            distances = []
            for neighbor_id in neighbors:
                pt2 = np.array( ( ( self.empty_grid[ neighbor_id ][0][0] + self.empty_grid[ neighbor_id ][1][0] ) / 2, 
                                  ( self.empty_grid[ neighbor_id ][0][1] + self.empty_grid[ neighbor_id ][1][1] ) / 2 ) )
                distances.append( np.linalg.norm( pt2 - pt1 ) )
            node_list.append( AStarNode( cell_id, mid_x, mid_y, neighbors, distances, 0.0, 0.0, 0.0, None ) )
        graph = AStarGraph( node_list )
        
        return graph

