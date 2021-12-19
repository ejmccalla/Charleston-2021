from AStarNode import AStarNode
from typing import List
import numpy as np

class AStarGraph():
    '''A class used to define a collection of nodes and their connections.

    The A* graph is used by the A* search algorithm to find optimal paths
    between graph nodes.

    Attributes:
        nodes (:obj:`list` of :obj:`AStarNode`): A list of A* nodes.
    '''
    def __init__ ( self, nodes:List[AStarNode] ):
        self.nodes = nodes


    def reset_all_costs_and_paths ( self ):
        '''Set the A* node costs f, g to infinity, h to 0 and the preceding
        node ID to None.

        Args:
            None

        Returns:
            None
        '''        
        for node in self.nodes:
            node.f = float("inf")
            node.g = float("inf")
            node.h = 0.0
            node.came_from_id = None


    def get_node ( self, node_id ):
        '''Get the node with the given ID.

        Args:
            node_id (:obj:`int`): The node ID.

        Returns:
            node (:obj:`AStarNode`): The node with the given ID.
        '''
        try:
            node = self.nodes[node_id]       
        except:
            node = None
            
        return node


    def get_nearest_node ( self, x_in, y_in ):
        '''Get the node that is nearest (Euclidean distance) to the given (x,y)
        coordinates.

        Args:
            x_in (:obj:`float`): The x-coordinate in the field.
            y_in (:obj:`float`): The y-coordinate in the field.

        Returns:
            node (:obj:`AStarNode`): The closest node.
        '''
        min_d = np.inf
        nearest_node = None
        for node in self.nodes:
            d = np.linalg.norm( np.array( ( node.x_in, node.y_in ) ) -
                                np.array( ( x_in, y_in ) ) )
            if d < min_d:
                min_d = d
                nearest_node = node

        return nearest_node

