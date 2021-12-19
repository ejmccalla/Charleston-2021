# C:\Users\ejmcc\AppData\Local\Programs\Python\Python37\python "C:\Users\ejmcc\GIT Projects\Charleston-2021\src\main\java\frc\robot\lib\planner\Visualizer.py"
from fields.field import field
from AStarPlanner import AStarPlanner
import cv2
import matplotlib.pyplot as plt
import numpy as np

class Visualizer ():
    '''A class used for to visualize the map building and path planning.

    Attributes:

    '''
    def __init__ ( self ):
        self.planner = AStarPlanner()

    def display_field_map (self, rgb_field_map_in):
        '''Display a map of the field.

        Args:

        Returns:
            None

        '''
        _ , ax = plt.subplots( 1, 1 )
        ax.set_xticks( [ 12 * i for i in range( 0, int( field.length_ft + 1 ), 3 ) ] )
        ax.set_xticklabels( [ i for i in range( 0, int( field.length_ft + 1), 3 ) ] )
        ax.set_yticks( [ 12 * i for i in range( 0, int( field.width_ft + 1), 3 ) ] )
        ax.set_yticklabels( [ i for i in range( 0, int( field.width_ft + 1), 3 ) ] )
        ax.xaxis.tick_top()
        plt.imshow( rgb_field_map_in )
        mng = plt.get_current_fig_manager()
        mng.window.state("zoomed")
        plt.show()

    def display_field_image_with_grids (self):
        '''Display a map of the field with the grid representation overlaid.

        Args:

        Returns:
            None

        '''
        field_img_bgr = field.img_bgr
        yt = ( ( field.img_offsets_in[1][1] - field.img_offsets_in[0][1] ) / 
            ( field.width_ft * 12 ) )
        
        xt = ( ( field.img_offsets_in[1][0] - field.img_offsets_in[0][0] ) /
            ( field.length_ft * 12 ) )

        for cell in self.planner.mapper.empty_grid:
            pt1 = ( int( field.img_offsets_in[0][0] + cell[0][0] * xt ), 
                    int( field.img_offsets_in[0][1] + cell[0][1] * yt ) )
            pt2 = ( int( field.img_offsets_in[0][0] + cell[1][0] * xt ), 
                    int( field.img_offsets_in[0][1] + cell[1][1] * yt ) )
            field_img_bgr = cv2.rectangle( field_img_bgr, pt1, pt2, ( 0, 255, 0 ), 2 )

        for cell in self.planner.mapper.occupied_grid:
            pt1 = ( int( field.img_offsets_in[0][0] + cell[0][0] * xt),
                    int( field.img_offsets_in[0][1] + cell[0][1] * yt) )
            pt2 = ( int( field.img_offsets_in[0][0] + cell[1][0] * xt),
                    int( field.img_offsets_in[0][1] + cell[1][1] * yt) )
            field_img_bgr = cv2.rectangle( field_img_bgr, pt1, pt2, ( 0, 0, 0 ), 2 )

        _, ax = plt.subplots( 1, 1 )
        xt = ( field.img_offsets_in[1][0] - field.img_offsets_in[0][0] ) / ( field.length_ft * 12 )
        ax.set_xticks( [ xt * i * 12 + field.img_offsets_in[0][0] for i in range( 0, int( field.length_ft + 1 ), 3 ) ] )
        ax.set_xticklabels( [ i for i in range( 0, int( field.length_ft+1 ), 3 ) ] )
        
        yt = ( field.img_offsets_in[1][1] - field.img_offsets_in[0][1] ) / ( field.width_ft * 12 )
        ax.set_yticks( [ yt * i * 12 + field.img_offsets_in[0][1] for i in range( 0, int( field.length_ft + 1 ), 3 ) ] )
        ax.set_yticklabels( [ i for i in range( 0, int( field.width_ft+1 ), 3 ) ] )
        ax.xaxis.tick_top()
        plt.imshow( cv2.cvtColor( field_img_bgr, cv2.COLOR_BGR2RGB ) )
        plt.show()

    def display_planned_path (self):
        '''Display a map of the field with the starting node, goal node and the
        planned path.

        Args:

        Returns:
            None

        '''
        f = cv2.cvtColor( self.planner.mapper.field_map_in, cv2.COLOR_GRAY2RGB )
        f = self.planner.mapper.add_grid_to_map( f,
                                                 self.planner.mapper.empty_grid, 
                                                 ( 127, 127, 127 ) )
        start_center = ( int( visualizer.planner.start_node.x ),
                         int( visualizer.planner.start_node.y ) )
        end_center = ( int( visualizer.planner.end_node.x ),
                       int( visualizer.planner.end_node.y ) )
        f = cv2.circle( f, start_center, 5,  ( 255, 0, 0 ), -1 )
        f = cv2.circle( f, end_center, 5,  ( 0, 255, 0 ), -1 )
        current_node = self.planner.end_node
        while(current_node.came_from_id != None):
            current_center = ( int( current_node.x ),
                               int( current_node.y ) )
            came_from_node = visualizer.planner.mapper.graph.get_node( current_node.came_from_id )
            came_from_center = ( int( came_from_node.x ),
                                 int( came_from_node.y ) )
            f = cv2.line( f, current_center, came_from_center, (127, 127, 0), 1 )
            current_node = came_from_node
        return f


if __name__ == '__main__':
    visualizer = Visualizer()

    for i in range(1):
        start_node_id = np.random.choice(visualizer.planner.mapper.graph.nodes).id
        end_node_id = np.random.choice(visualizer.planner.mapper.graph.nodes).id
        start_center = ( int( visualizer.planner.mapper.graph.get_node( start_node_id ).x_in ),
                         int( visualizer.planner.mapper.graph.get_node( start_node_id ).y_in ) )
        end_center = ( int( visualizer.planner.mapper.graph.get_node( end_node_id ).x_in ),
                       int( visualizer.planner.mapper.graph.get_node( end_node_id ).y_in ) )


        visualizer.planner.initialize( visualizer.planner.mapper.graph.get_node(start_node_id), 
                                       visualizer.planner.mapper.graph.get_node(end_node_id) )
        keep_searching = visualizer.planner.iteration()
        # images = []
        while keep_searching > 0:
            f = cv2.cvtColor( visualizer.planner.mapper.field_map_in, cv2.COLOR_GRAY2RGB )
            f = visualizer.planner.mapper.add_grid_to_map( f,visualizer.planner.mapper.empty_grid, ( 127, 127, 127 ) )
            f = cv2.circle( f, start_center, 5,  (255, 0, 0), -1 )
            f = cv2.circle( f, end_center, 5,  (0, 255, 0), -1 )
            current_center = ( int( visualizer.planner.current_node.x_in ), int( visualizer.planner.current_node.y_in ) )
            f = cv2.circle( f, current_center, 5,  (128, 128, 128), -1 )

            for node in visualizer.planner.mapper.graph.nodes:
                if node.f != float("inf"):
                    f = cv2.putText(f, str(int(node.g)), (int(node.x_in - 8), int(node.y_in - 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0,255,0), 1)
                    f = cv2.putText(f, str(int(node.h)), (int(node.x_in - 8), int(node.y_in + 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0,255,0), 1)
            

            #f = cv2.putText(f, str(int(visualizer.planner.get_current_node().f)), next_center, cv2.FONT_HERSHEY_SIMPLEX, 0.25, (127,127,0), 1)
            # images.append(f.copy())
            visualizer.display_field_map( f )
            keep_searching = visualizer.planner.iteration()

        # f = cv2.cvtColor( visualizer.planner.mapper.field_map_in, cv2.COLOR_GRAY2RGB )
        # f = visualizer.planner.mapper.add_grid_to_map( f,
        #                                     visualizer.planner.mapper.empty_grid, 
        #                                     ( 127, 127, 127 ) )
        # f = cv2.circle( f, start_center, 5,  (255, 0, 0), -1 )
        # f = cv2.circle( f, end_center, 5,  (0, 255, 0), -1 )
        # wp_x, wp_y = visualizer.planner.get_waypoints()
        # pt1 = None
        # pt2 = None
        # for x, y in zip( wp_x, wp_y ):
        #     pt2 = pt1
        #     pt1 = ( int( x ), int( y ) )
        #     if pt2 is not None and pt1 is not None:
        #         f = cv2.line( f, pt1, pt2, (127, 127, 0), 3 )

        # visualizer.display_field_map( f )

        

        # images.append(visualizer.display_planned_path().copy())
        # height, width, channels = f.shape
        # out = cv2.VideoWriter('planner.avi', cv2.VideoWriter_fourcc(*'DIVX'), 1, (width, height))
        # for i in range(len(images)):
        #     out.write(images[i])
        # out.release()


    # f = cv2.cvtColor( visualizer.planner.mapper.field_map_in, cv2.COLOR_GRAY2RGB )
    # # f = visualizer.mapper.add_adjacency_list_to_map( f,
    # #                                                  visualizer.mapper.empty_grid, 
    # #                                                  visualizer.mapper.adjacency_list )    
    # # visualizer.display_field_map( f )
    # # Add the empty cells of the grid and color them green
    # f = visualizer.planner.mapper.add_grid_to_map( f,
    #                                                visualizer.planner.mapper.empty_grid, 
    #                                                ( 0, 255, 0 ) )
    # # Add the occupied cells of the grid and color them red
    # f = visualizer.planner.mapper.add_grid_to_map( f,
    #                                                visualizer.planner.mapper.occupied_grid, 
    #                                                ( 255, 0, 0 ) )
    # # visualizer.display_field_map( f )                                  

    # visualizer.display_field_image_with_grids()
