import logging
import numpy as np
from pydrake.geometry import MeshcatVisualizer, StartMeshcat
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.all import FindResourceOrThrow, Parser, AddMultibodyPlantSceneGraph, SpatialInertia, UnitInertia, Sphere, CoulombFriction, Mesh
# from pydrake.examples import ManipulationStation

# from manipulation import running_as_notebook
# from manipulation.meshcat_utils import MeshcatPoseSliders, WsgButton
# from manipulation.scenarios import (AddIiwaDifferentialIK,
#                                     MakeManipulationStation)


# Start the visualizer.
meshcat = StartMeshcat()
model_directives = """
directives:
- add_directives:
    file: package://manipulation/clutter.dmd.yaml
- add_model:
    name: foam_brick
    file: package://drake/examples/manipulation_station/models/061_foam_brick.sdf
    default_free_body_pose:
        base_link:
            translation: [0, -0.6, 0.2]
"""

def AddGolfBall(plant):
    mu = 10.0
    ball = plant.AddModelInstance("ball")
    ball_body = plant.AddRigidBody("ball_body", ball,
                                   SpatialInertia(
                                       mass=0.2,
                                       p_PScm_E=np.array([0., 0., 0.]),
                                       G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
    # shape = Sphere(0.05)
    shape = Mesh('robot/golf_ball/golf_ball.obj', scale=0.03)
    if plant.geometry_source_is_registered():
        plant.RegisterCollisionGeometry(ball_body, RigidTransform(p=[-0.3, 0, 0]), shape, "book_body", CoulombFriction(mu, mu))
        plant.RegisterVisualGeometry(ball_body, RigidTransform(p=[-0.3, 0, 0]), shape, "book_body", [.9, .2, .2, 1.0])
    return ball

meshcat.Delete()
meshcat.DeleteAddedControls()
builder = DiagramBuilder()

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)

# Load the file into the plant/scene_graph.
parser = Parser(plant)
parser.AddModelFromFile('./robot/iiwa/iiwa_with_club.sdf')

ball = AddGolfBall(plant)

plant.Finalize()

visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
diagram = builder.Build()
context = diagram.CreateDefaultContext()
diagram.Publish(context)

while True:
    a = 0


# def teleop_3d():
#     builder = DiagramBuilder()
#
#     time_step = 0.001
#     station = builder.AddSystem(
#         MakeManipulationStation(model_directives, time_step=time_step))
#     plant = station.GetSubsystemByName("plant")
#     controller_plant = station.GetSubsystemByName(
#         "iiwa_controller").get_multibody_plant_for_control()
#
#     # Add a meshcat visualizer.
#     visualizer = MeshcatVisualizer.AddToBuilder(
#         builder, station.GetOutputPort("query_object"), meshcat)
#     meshcat.ResetRenderMode()
#     meshcat.DeleteAddedControls()
#
#     # Set up differential inverse kinematics.
#     differential_ik = AddIiwaDifferentialIK(
#         builder,
#         controller_plant,
#         frame=controller_plant.GetFrameByName("iiwa_link_7"))
#     builder.Connect(differential_ik.get_output_port(),
#                     station.GetInputPort("iiwa_position"))
#     builder.Connect(station.GetOutputPort("iiwa_state_estimated"),
#                     differential_ik.GetInputPort("robot_state"))
#
#     # Set up teleop widgets.
#     teleop = builder.AddSystem(
#         MeshcatPoseSliders(
#             meshcat,
#             min_range=MeshcatPoseSliders.MinRange(roll=0,
#                                                   pitch=-0.5,
#                                                   yaw=-np.pi,
#                                                   x=-0.6,
#                                                   y=-0.8,
#                                                   z=0.0),
#             max_range=MeshcatPoseSliders.MaxRange(roll=2 * np.pi,
#                                                   pitch=np.pi,
#                                                   yaw=np.pi,
#                                                   x=0.8,
#                                                   y=0.3,
#                                                   z=1.1),
#             body_index=plant.GetBodyByName("iiwa_link_7").index()))
#     builder.Connect(teleop.get_output_port(0),
#                     differential_ik.get_input_port(0))
#     builder.Connect(station.GetOutputPort("body_poses"),
#                     teleop.GetInputPort("body_poses"))
#     wsg_teleop = builder.AddSystem(WsgButton(meshcat))
#     builder.Connect(wsg_teleop.get_output_port(0),
#                     station.GetInputPort("wsg_position"))
#
#     diagram = builder.Build()
#     simulator = Simulator(diagram)
#     context = simulator.get_mutable_context()
#
#     # if running_as_notebook:  # Then we're not just running as a test on CI.
#     simulator.set_target_realtime_rate(1.0)
#
#     meshcat.AddButton("Stop Simulation", "Escape")
#     print("Press Escape to stop the simulation")
#     while meshcat.GetButtonClicks("Stop Simulation") < 1:
#         simulator.AdvanceTo(simulator.get_context().get_time() + 2.0)
#     meshcat.DeleteButton("Stop Simulation")
#
#     # else:
#     #     simulator.AdvanceTo(0.1)
#
#
# teleop_3d()