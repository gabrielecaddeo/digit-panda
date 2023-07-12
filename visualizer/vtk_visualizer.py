# 
#  Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT)
#  
#  This software may be modified and distributed under the terms of the
#  GPL-2+ license. See the accompanying LICENSE file for details.
#  
import numpy as np
import time
import threading
import vtk
import vtk.util.numpy_support as vtk_np
import argparse

# import rospy

class VTKPointCloudOnMesh():
    """
    A class to represent a dynamic point cloud on a mesh.
    
    '''

    Attributes
    ----------
    vtk_points : vtk.vtkPoints
        vtk representation of the point cloud
    vtk_cells : vtkCellArray
        list of the points ids of the point cloud
    poly_data : vtkPolyData
        a geometric structure of vertices
    actor_point_cloud : vtkActor
        the vtk object to display the point cloud in the scene
    actor_mesh : vtkActor
        the vtk object to display the mesh in the scene

    Methods
    -------
    update(thread_lock, update_on, function, args):
        Start a new thread to continuously update the point cloud with a function.
    update_actor(thread_lock, update_on, function, args)
        Update the point cloud depending on the function passed.
    """

    def __init__(self, mesh_path_object, mesh_path_digit):
        """
        Constructor.

        Parameters
        ----------
        mesh_path : str
            path to the .obj file of the mesh
        point_cloud_path : str
            path the .txr file of the point cloud
        rgb_color : tuple
            rgb color code of the point cloud
        points_size: float
            size of the points
        """


        # Initialize the mesh
        reader = vtk.vtkOBJReader()
        reader.SetFileName(mesh_path_object)
        mapper_mesh = vtk.vtkPolyDataMapper()
        output_port = reader.GetOutputPort()
        mapper_mesh.SetInputConnection(output_port)

        self.actor_mesh_object = vtk.vtkActor()
        self.actor_mesh_object.SetMapper(mapper_mesh)

        reader = vtk.vtkOBJReader()
        reader.SetFileName(mesh_path_digit)
        mapper_mesh = vtk.vtkPolyDataMapper()
        output_port = reader.GetOutputPort()
        mapper_mesh.SetInputConnection(output_port)

        self.actor_mesh_digit = vtk.vtkActor()
        self.actor_mesh_digit.SetMapper(mapper_mesh)

        ## ROS
        rospy.init_node('digit_node')

        self.digit_state = rospy.Subscriber('/franka_state_controller/franka_states', PoseStamped, self.callback_pose_digit)
        self.object_state = rospy.Subscriber('/aruco_board_detector/board_pose', PoseStamped, self.callback_pose_aruco)
        self.bool_digit = False
        self.bool_object = False

        # Mutex
        self.mutex = Lock()
        self.mutex2 = Lock()

        self.pose_digit = self.pose_aruco = None


    def update(self, thread_lock, update_on, function, args):
        """
        Start a new thread to continuously update the point cloud with a function.

        Parameters
        ----------
        thread_lock : threading.Lock
            semaphore
        update_on : threading.Event
            signal for the semaphore
        function : python function
            function to update the point cloud
        args : tuple
            arguments of the function passed as argument

        Returns
        -------
        None
        """
        
        thread = threading.Thread(target=self.update_actor, args=(thread_lock, update_on, function, args))
        thread.start()


    def update_actor(self, thread_lock, update_on, function, args):
        """
        Function to update the point cloud

        Parameters
        ----------
        thread_lock : threading.Lock
            semaphore
        update_on : threading.Event
            signal for the semaphore
        function : python function
            function to update the point cloud
        args : tuple
            arguments of the function passed as argument

        Returns
        -------
        None
        """

        while (update_on.is_set()) and self.mutex and self.mutex2:
            time.sleep(0.01)
            thread_lock.acquire()
            
            # # Update the 
            # nparray = function(*args)

            # Update the vtk class of the point cloud

            ### Modify trial
            if self.bool_digit:

                self.actor_mesh_digit.RotateWXYZ(self.pose_digit.orientation.w, self.pose_digit.orientation.x, self.pose_digit.orientation.y, self.pose_digit.orientation.z)
                self.actor_mesh_digit.SetPosition(self.pose_digit.position.x, self.pose_digit.position.y, self.pose_digit.position.z)

            if self.bool_object:

                self.actor_mesh_object.RotateWXYZ(self.pose_aruco.orientation.w, self.pose_aruco.orientation.x, self.pose_aruco.orientation.y, self.pose_aruco.orientation.z)
                self.actor_mesh_object.SetPosition(self.pose_aruco.position.x, self.pose_aruco.position.y, self.pose_aruco.position.z)

            # pos1 = self.actor_mesh_digit.GetPosition()
            # self.actor_mesh_digit.SetPosition(pos1[0] - 0.001, pos1[1], pos1[2])

            thread_lock.release()


    def callback_pose_digit(self, data):

        with self.mutex:
            self.bool_digit = True
            self.pose_digit = data.pose

    def callback_pose_aruco(self, data):

        with self.mutex2:
            self.bool_object = True
            self.pose_aruco = data.pose

class VTKVisualisation():
    """
    A class to start a vtk visualizer
    
    '''

    Attributes
    ----------
    thread_lock : threading.Lock
        semaphore
    ren : vtkRenderer
        renderer
    axes_actor : vtkAxesActor
        the vtk object to display the axis in the scene
    ren_win : vtkRenderWindow
        the vtk object to set a window for the renderer
    int_ren : vtkRenderWindowInteractor
        the vtk object to interacti with the window
    style : vtkInteractorStyleTrackballCamera
        the vtk object to decide how interact with the window

    Methods
    -------
    update(thread_lock, update_on, function, args):
        Method to update the render window.

    """

    def __init__(self, thread_lock, actor_wrapper, background=(0, 0, 0), axis=True):
        """
        Constructor.

        Parameters
        ----------
        thread_lock : threading.Lock
            semaphore
        actor_wrapper : VTKPointCloudOnMesh
            wrapper for the actors
        background : tuple
            rgb color code for the window background
        axis : bool
            boolean to decide wheater render the axis in the origin or not
        """
         
        # Assign the semaphore
        self.thread_lock = thread_lock

        # Initialize the renderer
        self.ren = vtk.vtkRenderer()
        self.ren.SetBackground(*background)

        # Add the point cloud
        self.ren.AddActor(actor_wrapper.actor_mesh_object)
        
        # Add the mesh
        self.ren.AddActor(actor_wrapper.actor_mesh_digit)

        # Add the axes
        if axis:
            self.axes_actor = vtk.vtkAxesActor()
            self.axes_actor.AxisLabelsOff()
            self.axes_actor.SetTotalLength(1, 1, 1)
            self.ren.AddActor(self.axes_actor)

        # Create a window
        self.ren_win = vtk.vtkRenderWindow()
        self.ren_win.AddRenderer(self.ren)

        # Create an interactor
        self.int_ren = vtk.vtkRenderWindowInteractor()
        self.int_ren.SetRenderWindow(self.ren_win)
        self.int_ren.Initialize()

        # Change the style to actively interact with the camera in the scene
        self.style = vtk.vtkInteractorStyleTrackballCamera()
        self.int_ren.SetInteractorStyle(self.style)

        # Add a function to the loop
        self.int_ren.AddObserver("TimerEvent", self.update_visualisation)
        dt = 30 # ms
        timer_id = self.int_ren.CreateRepeatingTimer(dt)


    def update_visualisation(self, obj=None, event=None):
        """
        Method to update the render window.

        Parameters
        ----------
        obj : vtkXRenderWindowInteractor, optional
            convenience object that provides event bindinds to common graphics functions
        event : str, optional
            name of the event to handle
        """

        time.sleep(0.01)
        self.thread_lock.acquire()
        self.ren.GetRenderWindow().Render()
        self.thread_lock.release()



def function_loop(prune_object):
    """
    Function to be passed in the loop to update the point cloud dependending on the image coming from the DIGIT sensor.

    Parameters
    ----------
    prune_object : PrunePointCloud
        wrapper for the point cloud pruner
    digit_object : Digit
        wrapper for the Digit sensor
    """
    
    print('ciao')
    return prune_object

def main():
    """
    Main function of the script.
    """

    # Initialize the parser and parse the inputs
    parser = argparse.ArgumentParser(description= '')
    parser.add_argument('--object', dest='mesh', help='object to import the mesh of', type=str, required=True)
    parser.add_argument('--point_cloud', dest='point_cloud', help='point cloud of the object under test', type=str, required=True)
    args = parser.parse_args()
    
    # Initialize the threading event and the semaphore
    update_on = threading.Event()
    update_on.set()
    thread_lock = threading.Lock()

    # Set the visualizer
    actorWrapper = VTKPointCloudOnMesh(args.mesh, args.point_cloud)
    actorWrapper.update(thread_lock, update_on, function_loop, ('ciao'))  
    viz = VTKVisualisation(thread_lock, actorWrapper)
    viz.int_ren.Start()

    update_on.clear()

main()