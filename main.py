import logging
import numpy as np
from pydrake.geometry import MeshcatVisualizer, StartMeshcat
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

from scenario import (AddGolfBall, AddIiwaDifferentialIK, MakeManipulationStation)
from meshcat_utils import MeshcatPoseSliders, WsgButton


def teleop_3d():
    builder = DiagramBuilder()
    time_step = 0.001
    station = builder.AddSystem(
        MakeManipulationStation(time_step=time_step))
    plant = station.GetSubsystemByName("plant")
    controller_plant = station.GetSubsystemByName(
        "iiwa_controller").get_multibody_plant_for_control()

    # Add a meshcat visualizer.
    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), meshcat)
    meshcat.ResetRenderMode()
    meshcat.DeleteAddedControls()

    # Set up differential inverse kinematics.
    differential_ik = AddIiwaDifferentialIK(
        builder, controller_plant,
        frame=controller_plant.GetFrameByName("iiwa_link_7"))
    builder.Connect(differential_ik.get_output_port(),
                    station.GetInputPort("iiwa_position"))
    builder.Connect(station.GetOutputPort("iiwa_state_estimated"),
                    differential_ik.GetInputPort("robot_state"))

    # Set up teleop widgets.
    teleop = builder.AddSystem(
        MeshcatPoseSliders(
            meshcat,
            min_range=MeshcatPoseSliders.MinRange(roll=0, pitch=-0.5, yaw=-np.pi,
                                                  x=-0.6, y=-0.8, z=0.0),
            max_range=MeshcatPoseSliders.MaxRange(roll=2 * np.pi, pitch=np.pi, yaw=np.pi,
                                                  x=0.8, y=0.3, z=1.1),
            body_index=plant.GetBodyByName("iiwa_link_7").index()))
    builder.Connect(teleop.get_output_port(0),
                    differential_ik.get_input_port(0))
    builder.Connect(station.GetOutputPort("body_poses"),
                    teleop.GetInputPort("body_poses"))
    # wsg_teleop = builder.AddSystem(WsgButton(meshcat))
    # builder.Connect(wsg_teleop.get_output_port(0),
    #                 station.GetInputPort("wsg_position"))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # if running_as_notebook:  # Then we're not just running as a test on CI.
    simulator.set_target_realtime_rate(1.0)
    diagram.Publish(context)

    meshcat.AddButton("Stop Simulation", "Escape")
    print("Press Escape to stop the simulation")
    while meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 2.0)
    meshcat.DeleteButton("Stop Simulation")

    # else:
    #     simulator.AdvanceTo(0.1)


# Start the visualizer.
meshcat = StartMeshcat()

# builder = DiagramBuilder()
#
# plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)

# # Load the file into the plant/scene_graph.
# parser = Parser(plant)
# parser.AddModelFromFile('./robot/iiwa/iiwa_with_club.sdf')

# ball = AddGolfBall(plant)

# plant.Finalize()

# visualizer = MeshcatVisualizer(meshcat).AddToBuilder(builder, scene_graph, meshcat)
# diagram = builder.Build()
# context = diagram.CreateDefaultContext()
# diagram.Publish(context)

teleop_3d()

# while True:
#     a = 0
