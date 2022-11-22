import numpy as np
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.all import (
    AbstractValue, Adder, AddMultibodyPlantSceneGraph, BallRpyJoint, BaseField,
    Box, CameraInfo, Capsule, ClippingRange, CoulombFriction, Cylinder,
    Demultiplexer, DepthImageToPointCloud, DepthRange, DepthRenderCamera,
    DiagramBuilder, GeometryInstance,
    InverseDynamicsController, LeafSystem, LoadModelDirectivesFromString,
    MakeMultibodyStateToWsgStateSystem, MakePhongIllustrationProperties,
    MakeRenderEngineVtk, Mesh, ModelInstanceIndex, MultibodyPlant, Parser,
    PassThrough, PrismaticJoint, ProcessModelDirectives, RenderCameraCore,
    RenderEngineVtkParams, RevoluteJoint, Rgba, RgbdSensor, RigidTransform,
    RotationMatrix, SchunkWsgPositionController, SpatialInertia,
    Sphere, StateInterpolatorWithDiscreteDerivative, UnitInertia)
from pydrake.manipulation.planner import (
    DifferentialInverseKinematicsIntegrator,
    DifferentialInverseKinematicsParameters)


def AddGolfBall(plant):
    mu = 10.0
    ball = plant.AddModelInstance("ball")
    ball_body = plant.AddRigidBody("ball_body", ball,
                                   SpatialInertia(
                                       mass=0.2,  # 0.2, TODO: set back to nonzero after adding terrain
                                       p_PScm_E=np.array([0., 0., 0.]),
                                       G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
    shape = Mesh('robot/golf_ball/golf_ball.obj', scale=0.03)
    if plant.geometry_source_is_registered():
        plant.RegisterCollisionGeometry(ball_body, RigidTransform(p=[0.7, 1, 1]), shape, "ball_body",
                                        CoulombFriction(mu, mu))
        plant.RegisterVisualGeometry(ball_body, RigidTransform(p=[0.7, 1, 1]), shape, "ball_body", [.9, .2, .2, 1.0])
    return ball


def AddTerrain(plant):
    mu = 10.0
    terrain = plant.AddModelInstance("terrain")
    terrain_body = plant.AddRigidBody("terrain_body", terrain,
                                      SpatialInertia(
                                          mass=0.2,  # 0.2, TODO: set back to nonzero after adding terrain
                                          p_PScm_E=np.array([0., 0., 0.]),
                                          G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
    shape = Mesh('robot/terrain/terrain.obj', scale=0.3)
    if plant.geometry_source_is_registered():
        plant.RegisterCollisionGeometry(terrain_body, RigidTransform(p=np.array([0, 0, -0.5])), shape, "ball_body",
                                        CoulombFriction(mu, mu))
        plant.RegisterVisualGeometry(terrain_body, RigidTransform(p=np.array([0, 0, -0.5])), shape, "ball_body", [.9, .2, .2, 1.0])
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("terrain_body"))
    return terrain


def AddIiwa(plant):
    sdf_path = 'robot/iiwa/iiwa_with_club.sdf'

    parser = Parser(plant)
    iiwa = parser.AddModelFromFile(sdf_path, model_name='iiwa')
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"))

    # Set default positions:
    q0 = [0.0, 0.1, 0, -1.2, 0, 1.6, 0]
    index = 0
    for joint_index in plant.GetJointIndices(iiwa):
        joint = plant.get_mutable_joint(joint_index)
        if isinstance(joint, RevoluteJoint):
            joint.set_default_angle(q0[index])
            index += 1

    return iiwa


def AddIiwaDifferentialIK(builder, plant, frame=None):
    params = DifferentialInverseKinematicsParameters(plant.num_positions(), plant.num_velocities())
    time_step = plant.time_step()
    q0 = plant.GetPositions(plant.CreateDefaultContext())
    params.set_nominal_joint_position(q0)
    params.set_end_effector_angular_speed_limit(2)
    params.set_end_effector_translational_velocity_limits([-2, -2, -2], [2, 2, 2])
    if plant.num_positions() == 3:  # planar iiwa
        iiwa14_velocity_limits = np.array([1.4, 1.3, 2.3])
        params.set_joint_velocity_limits(
            (-iiwa14_velocity_limits, iiwa14_velocity_limits))
        # These constants are in body frame
        assert (
                frame.name() == "iiwa_link_7"
        ), "Still need to generalize the remaining planar diff IK params for different frames"  # noqa
        params.set_end_effector_velocity_flag(
            [True, False, False, True, False, True])
    else:
        iiwa14_velocity_limits = np.array([1.4, 1.4, 1.7, 1.3, 2.2, 2.3, 2.3])
        params.set_joint_velocity_limits(
            (-iiwa14_velocity_limits, iiwa14_velocity_limits))
        params.set_joint_centering_gain(10 * np.eye(7))
    if frame is None:
        frame = plant.GetFrameByName("body")
    differential_ik = builder.AddSystem(
        DifferentialInverseKinematicsIntegrator(
            plant,
            frame,
            time_step,
            params,
            log_only_when_result_state_changes=True))
    return differential_ik


def MakeManipulationStation(time_step=0.002):
    """
    Creates a manipulation-master station system, which is a sub-diagram containing:
      - A MultibodyPlant with populated via the Parser from the
        `model_directives` argument AND the `filename` argument.
      - A SceneGraph
      - For each model instance starting with `iiwa_prefix`, we add an
        additional iiwa controller system
      - For each body starting with `camera_prefix`, we add a RgbdSensor

    Args:
        time_step: the standard MultibodyPlant time step.
    """
    builder = DiagramBuilder()

    # Add (only) the iiwa, WSG, and cameras to the scene.
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    parser = Parser(plant)
    AddGolfBall(plant)
    AddIiwa(plant)
    AddTerrain(plant)
    plant.Finalize()

    for i in range(plant.num_model_instances()):
        model_instance = ModelInstanceIndex(i)
        model_instance_name = plant.GetModelInstanceName(model_instance)

        if model_instance_name.startswith('iiwa'):
            num_iiwa_positions = plant.num_positions(model_instance)

            # I need a PassThrough system so that I can export the input port.
            iiwa_position = builder.AddSystem(PassThrough(num_iiwa_positions))
            builder.ExportInput(iiwa_position.get_input_port(),
                                model_instance_name + "_position")
            builder.ExportOutput(iiwa_position.get_output_port(),
                                 model_instance_name + "_position_commanded")

            # Export the iiwa "state" outputs.
            demux = builder.AddSystem(
                Demultiplexer(2 * num_iiwa_positions, num_iiwa_positions))
            builder.Connect(plant.get_state_output_port(model_instance),
                            demux.get_input_port())
            builder.ExportOutput(demux.get_output_port(0),
                                 model_instance_name + "_position_measured")
            builder.ExportOutput(demux.get_output_port(1),
                                 model_instance_name + "_velocity_estimated")
            builder.ExportOutput(plant.get_state_output_port(model_instance),
                                 model_instance_name + "_state_estimated")

            # Make the plant for the iiwa controller to use.
            controller_plant = MultibodyPlant(time_step=time_step)
            AddIiwa(controller_plant)
            controller_plant.Finalize()

            # Add the iiwa controller
            iiwa_controller = builder.AddSystem(
                InverseDynamicsController(controller_plant,
                                          kp=[100] * num_iiwa_positions,
                                          ki=[1] * num_iiwa_positions,
                                          kd=[20] * num_iiwa_positions,
                                          has_reference_acceleration=False))
            iiwa_controller.set_name(model_instance_name + "_controller")
            builder.Connect(plant.get_state_output_port(model_instance),
                            iiwa_controller.get_input_port_estimated_state())

            # Add in the feed-forward torque
            adder = builder.AddSystem(Adder(2, num_iiwa_positions))
            builder.Connect(iiwa_controller.get_output_port_control(),
                            adder.get_input_port(0))
            # Use a PassThrough to make the port optional (it will provide zero
            # values if not connected).
            torque_passthrough = builder.AddSystem(
                PassThrough([0] * num_iiwa_positions))
            builder.Connect(torque_passthrough.get_output_port(),
                            adder.get_input_port(1))
            builder.ExportInput(torque_passthrough.get_input_port(),
                                model_instance_name + "_feedforward_torque")
            builder.Connect(adder.get_output_port(),
                            plant.get_actuation_input_port(model_instance))

            # Add discrete derivative to command velocities.
            desired_state_from_position = builder.AddSystem(
                StateInterpolatorWithDiscreteDerivative(
                    num_iiwa_positions,
                    time_step,
                    suppress_initial_transient=True))
            desired_state_from_position.set_name(
                model_instance_name + "_desired_state_from_position")
            builder.Connect(desired_state_from_position.get_output_port(),
                            iiwa_controller.get_input_port_desired_state())
            builder.Connect(iiwa_position.get_output_port(),
                            desired_state_from_position.get_input_port())

            # Export commanded torques.
            builder.ExportOutput(adder.get_output_port(),
                                 model_instance_name + "_torque_commanded")
            builder.ExportOutput(adder.get_output_port(),
                                 model_instance_name + "_torque_measured")

            builder.ExportOutput(
                plant.get_generalized_contact_forces_output_port(
                    model_instance), model_instance_name + "_torque_external")

    # # Cameras.
    # AddRgbdSensors(builder,
    #                plant,
    #                scene_graph,
    #                model_instance_prefix=camera_prefix)

    # Export "cheat" ports.
    builder.ExportOutput(scene_graph.get_query_output_port(), "query_object")
    builder.ExportOutput(plant.get_contact_results_output_port(),
                         "contact_results")
    builder.ExportOutput(plant.get_state_output_port(),
                         "plant_continuous_state")
    builder.ExportOutput(plant.get_body_poses_output_port(), "body_poses")

    diagram = builder.Build()
    diagram.set_name("ManipulationStation")
    return diagram
