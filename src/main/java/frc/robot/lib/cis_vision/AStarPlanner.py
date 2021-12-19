from AStarNode import AStarNode
from Mapper import Mapper
import numpy as np

# from queue import PriorityQueue TODO: Revisit this if performance becomes an issue

class AStarPlanner ():
    '''Implementation of the A* path planning algorithm.

    The A* node can be used to build a graph and perform a graph search using
    the well-known A* algorithm. The defining feature of the algorithm is the
    addition of the heuristic cost function. The total cost is then defined
    as this cost plus the cost incurred reaching a node. The heuristid is
    meant to guide the search to arrive at the optimal solution quickly.
    The heuristic functinon used in this implementation is the Euclidean
    distance.

    The node also represents a physical space defined by a rectangular
    center and boundary. For this implementation, only the center (x_in, y_in)
    is used. TODO: Future improvement to path planning/following may benefit
    from including information about the boudaries. For example, if 8 points
    were included which represented the boundary vertices and bisections,
    a planner could take advantage of the higher resoltion by planning more
    optimal paths.

    Attributes:

    '''
    def __init__ ( self ):
        self.mapper = Mapper( 24 )  # In FRC, robots are pretty standard sizes
        self.open_set = None
        self.closed_set = None
        self.start_node = None
        self.current_node = None
        self.end_node = None


    def initialize ( self, start_node:AStarNode, end_node:AStarNode ):
        '''The function will initialize the planner.

        All prior cost an paths will be reset to their default values. The
        open set will be initialized with the starting node and the closed
        set will be initialized empty.

        Args:
            start_node (:obj:`AStarNode'): The starting node of the search.
            end_node (:obj:`AStarNode'): The ending node of the search.

        Returns:
            None        
        '''
        self.mapper.graph.reset_all_costs_and_paths()
        self.start_node = start_node
        self.end_node = end_node
        self.start_node.update_costs( self.end_node, 0 )
        self.open_set = np.array( ( self.start_node.f, self.start_node.id ) ).reshape(1, 2)
        self.closed_set = []


    def iteration ( self ):
        '''The function will perform a single iteration of the A* search.

        Args:

        Returns:
            None        
        '''
        self.current_node = self.mapper.graph.get_node( int( self.open_set[0,1] ) )
        self.open_set = np.delete( self.open_set, 0, 0 )
        
        # Found the goal, send a -1 back to indicate to the caller the path has
        # been found.
        if self.current_node.eq( self.end_node ): return -1

        # Process the neighboring nodes
        for neighbor_id, distance in zip(self.current_node.neighbors, self.current_node.distances):
            neighbor_node = self.mapper.graph.get_node( neighbor_id )
            tentative_g = self.current_node.g + distance
            
            # Update the neighbor path with this shorter one.
            if tentative_g < neighbor_node.g:
                neighbor_node.came_from_id = self.current_node.id
                neighbor_node.update_costs( self.end_node, tentative_g )
                if neighbor_id not in self.open_set[:,1]:
                    self.open_set = np.append( self.open_set, [ [ neighbor_node.f, neighbor_id] ], 0 )
        
        # Sort the open set
        self.open_set = self.open_set[ np.argsort( self.open_set[:, 0] ) ] 

        # Send the size of the open list back to indicate the status of the
        # search. If the list is empty, then the size will be zero. This 
        # means that the goal node cannot be found.
        return self.open_set.size


    def get_waypoints ( self ):
        '''The function will return the path as a list of waypoints.

        Args:

        Returns:
            waypoints_x_in, waypoints_y_in (:obj:`list of float',
                :obj:`list of float'): A list of x-coordinates and a list of
                y-coordates.
        '''
        current_node = self.end_node
        waypoints_x_in = [current_node.x_in]
        waypoints_y_in = [current_node.y_in]

        while current_node != self.start_node:
            prev_node_id = current_node.came_from_id
            if prev_node_id is not None:
                current_node =  self.mapper.graph.get_node( prev_node_id )
            else:
                print("ERROR: no prior node ID")
                break
            waypoints_x_in.insert( 0, current_node.x_in )
            waypoints_y_in.insert( 0, current_node.y_in )

        return waypoints_x_in, waypoints_y_in


    def plan_path ( self, start, end ):
        '''The function will return the path as a list of waypoints.

        Args:
            start (:obj:`tuple(float,float)'): The starting xy-coordinates.
            end (:obj:`tuple(float,float)'): The starting xy-coordinates.

        Returns:
            waypoints_x, waypoints_y (:obj:`list of float', :obj:`list of float'):
                A list of x-coordinates and a list of y-coordates.
        '''
        starting_node = self.mapper.graph.get_nearest_node( start[0], start[1])
        ending_node = self.mapper.graph.get_nearest_node( end[0], end[1] )
        self.initialize( starting_node, ending_node )
        itr_resutls = self.iteration()

        # -1 means a path has been found, 0 means no path has been found and
        # the search should quit, > 0 means the search is ongoing.
        while itr_resutls > 0:
            itr_resutls = self.iteration()
        if itr_resutls == -1:
            return [], []
        else:
            return self.get_waypoints()
