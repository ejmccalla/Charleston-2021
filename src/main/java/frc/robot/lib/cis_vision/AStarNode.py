from dataclasses import dataclass
from typing import List
import numpy as np

@dataclass
class AStarNode:
    '''A node dataclass for A* path planning algorithm.

    Attributes:
        x_in (:obj:`float`): The x-coordinate of the node in the field.
        y_in (:obj:`float`): The y-coordinate of the node in the field.
        neighbors (:obj:`list` of :obj:`int`): A list of adjacent nodes.
        distances (:obj:`list` of :obj:`float`): A list of the Euclidean
            distances to the neighboring nodes.
        f (:obj:`float`): The total cost computed as g(n) + h(n). 
        g (:obj:`float`): The cost computed as the path distance from the
            starting node.
        h (:obj:`float`): The heuristic cost computed as the Euclidean distance
            to the goal node.
        came_from_id (:obj:`int`): The node on the path that precedes to this
            node.
    '''
    id: int
    x_in: float
    y_in: float
    neighbors: List[int]
    distances: List[int]
    f: float
    g: float
    h: float
    came_from_id: int


    def update_costs ( self, goal_node, g ):
        '''Update the A* costs given the path distance from the starting node.

        Args:
            goal_node (:obj:`AStarNode`): Another A_Star node.
            g (:obj:`float`): The cost computed as the path distance from the
                starting node.

        Returns:
            None
            
        '''        
        self.h = np.linalg.norm( np.array( ( goal_node.x_in, goal_node.y_in ) ) -
                                 np.array( ( self.x_in, self.y_in ) ) )
        self.g = g
        self.f = self.g + self.h


    def eq ( self, other_node ):
        '''Compare node ID's to see if they're the same.

        Args:
            other_node (:obj:`AStarNode`): Another A* node.

        Returns:
            True if the node ID's are equal, otherwise False.

        '''        
        if not isinstance( other_node, AStarNode ):
            return NotImplemented
        return self.id == other_node.id
