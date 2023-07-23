#
#  Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT)
#
#  This software may be modified and distributed under the terms of the
#  GPL-2+ license. See the accompanying LICENSE file for details.
#
import argparse
from geometry_msgs.msg import PoseStamped
import numpy as np
import pyquaternion as pyq
import rospy
import threading
from threading import Lock
import time
import vtk
import vtk.util.numpy_support as vtk_np


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

    def __init__(self, mesh_path_object, mesh_path_digit, point_cloud_path, rgb_color=(1, 0, 0), points_size=3):
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

        # Initialize the point cloud with a 3D point in the origin as default
        self.point_cloud_array = np.loadtxt(point_cloud_path)[:, :3] 
        self.n_elements = self.point_cloud_array.shape[0]

        self.labels = np.loadtxt(point_cloud_path)[:, 6] 
        # Create the vtk class for the point cloud
        self.vtk_points = vtk.vtkPoints()
        self.vtk_cells = vtk.vtkCellArray()
        self.poly_data = vtk.vtkPolyData()
        self.vtk_points.SetData(vtk_np.numpy_to_vtk(self.point_cloud_array))
        cells_npy = np.vstack([np.ones(self.n_elements,dtype=np.int64),
                               np.arange(self.n_elements,dtype=np.int64)]).T.flatten()
        self.vtk_cells.SetCells(self.n_elements,vtk_np.numpy_to_vtkIdTypeArray(cells_npy))
        self.poly_data.SetPoints(self.vtk_points)
        self.poly_data.SetVerts(self.vtk_cells)

        # Assing the point cloud to the actor through a mapper
        mapper_point_cloud = vtk.vtkPolyDataMapper()
        mapper_point_cloud.SetInputDataObject(self.poly_data)

        self.actor_point_cloud = vtk.vtkActor()
        self.actor_point_cloud.SetMapper(mapper_point_cloud)
        self.actor_point_cloud.GetProperty().SetRepresentationToPoints()
        self.actor_point_cloud.GetProperty().SetColor(*rgb_color)
        self.actor_point_cloud.GetProperty().SetPointSize(points_size)

        # Initialize the mesh
        reader = vtk.vtkOBJReader()
        reader.SetFileName(mesh_path_object)
        mapper_mesh = vtk.vtkPolyDataMapper()
        output_port = reader.GetOutputPort()
        mapper_mesh.SetInputConnection(output_port)

        self.actor_mesh_object = vtk.vtkActor()
        self.actor_mesh_object.SetMapper(mapper_mesh)

        reader = vtk.vtkSTLReader()
        reader.SetFileName(mesh_path_digit)
        mapper_mesh = vtk.vtkPolyDataMapper()
        output_port = reader.GetOutputPort()
        mapper_mesh.SetInputConnection(output_port)

        self.actor_mesh_digit = vtk.vtkActor()
        self.actor_mesh_digit.SetMapper(mapper_mesh)

        ## ROS
        rospy.init_node('visualizer_node')

        self.digit_state = rospy.Subscriber('/digit_node/digit_pose', PoseStamped, self.callback_pose_digit)
        self.object_state = rospy.Subscriber('/digit_node/object_pose', PoseStamped, self.callback_pose_aruco)
        self.bool_digit = False
        self.bool_object = False

        # Mutex
        self.mutex_object = Lock()
        self.mutex_digit = Lock()

        self.pose_digit = self.pose_aruco = None

    def update(self, thread_lock, update_on):
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

        thread = threading.Thread(target=self.update_actor, args=(thread_lock, update_on))
        thread.start()

    def update_actor(self, thread_lock, update_on):
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

        while (update_on.is_set()) and self.mutex_object and self.mutex_digit:
            time.sleep(0.01)
            thread_lock.acquire()

            ### Modify trial
            if self.bool_digit:
                
                vtk_matrix_digit, numpy_matrix_digit = self.set_matrix(self.pose_digit)
                self.actor_mesh_digit.SetUserMatrix(vtk_matrix_digit)

            if self.bool_object:
                
                vtk_matrix_object, numpy_matrix_object = self.set_matrix(self.pose_aruco)
                self.actor_mesh_object.SetUserMatrix(vtk_matrix_object)
                
                if self.bool_digit:
                    
                    # Change Refernce Frame for the point_cloud
                    points_robot_frame = numpy_matrix_object @ np.concatenate([self.point_cloud_array.T, np.ones((1,self.n_elements))], axis=0)

                    # Intialize transformation to the surface of the gel
                    transform_gel_digit = np.eye(4)
                    transform_gel_digit[:, 3] = np.array([0.02, 0, 0.015])

                    # Transform the pose of the digit
                    matrix_surface_gel = numpy_matrix_digit @ transform_gel_digit

                    # Compute the distance between the points in the robot reference frame and the DIGIT gel surface
                    distances = np.linalg.norm(points_robot_frame - matrix_surface_gel[:,3].T, axis=0)

                    # Find the min value
                    min_index = np.argmin(distances)
                    min_label = self.labels[min_index]

                    indexes = np.array(self.labels==min_label)
                    new_points = self.point_cloud_array[indexes]

                    # Update the vtk class of the point cloud
                    n_elements = new_points.shape[0]
                    self.vtk_points.SetData(vtk_np.numpy_to_vtk(new_points))
                    cells_npy = np.vstack([np.ones(n_elements,dtype=np.int64),
                                        np.arange(n_elements,dtype=np.int64)]).T.flatten()
                    self.vtk_cells.SetCells(n_elements,vtk_np.numpy_to_vtkIdTypeArray(cells_npy))
                    self.poly_data.SetPoints(self.vtk_points)
                    self.poly_data.SetVerts(self.vtk_cells)

                    self.poly_data.Modified()


            thread_lock.release()

    def set_matrix(self, pose):

        transform_matrix = pyq.Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).transformation_matrix
        transform_matrix[0,3] = pose.position.x
        transform_matrix[1,3] = pose.position.y
        transform_matrix[2,3] = pose.position.z
        matrix4x4 = vtk.vtkMatrix4x4()
        [matrix4x4.SetElement(x, y, transform_matrix[x,y]) for x in range(4) for y in range(4)]

        return matrix4x4, transform_matrix



    def callback_pose_digit(self, data):

        with self.mutex_digit:
            self.bool_digit = True
            self.pose_digit = data.pose

    def callback_pose_aruco(self, data):

        with self.mutex_object:
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


def main():
    """
    Main function of the script.
    """

    # Initialize the parser and parse the inputs
    parser = argparse.ArgumentParser(description= '')
    parser.add_argument('--object', dest='mesh', help='object to import the mesh of', type=str, required=True)
    parser.add_argument('--sensor', dest='sensor', help='sensor to import the mesh of', type=str, required=True)
    args = parser.parse_args()

    # Initialize the threading event and the semaphore
    update_on = threading.Event()
    update_on.set()
    thread_lock = threading.Lock()

    # Set the visualizer
    actorWrapper = VTKPointCloudOnMesh(args.mesh, args.sensor)
    actorWrapper.update(thread_lock, update_on)
    viz = VTKVisualisation(thread_lock, actorWrapper)
    viz.int_ren.Start()

    update_on.clear()

main()
