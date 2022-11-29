import numpy as np

from pydrake.all import (
    AddMultibodyPlantSceneGraph, Box, DiagramBuilder, ContactVisualizerParams,
    LeafSystem, MeshcatVisualizer, ContactVisualizer, MeshcatVisualizerParams,
    MultibodyPlant, FixedOffsetFrame, Role, AbstractValue, EventStatus,
    PlanarJoint, ContactResults, BodyIndex, RigidTransform, RotationMatrix,
    SceneGraph, JointSliders, Simulator, SpatialInertia, Sphere, UnitInertia,
    StartMeshcat, CoulombFriction, RollPitchYaw, Mesh
)

from scenario import AddTerrain, AddGolfBall

from IPython.display import clear_output

# Start the visualizer.
meshcat = StartMeshcat()


slope  = 0.1
q1 = [0.0, 0.15, 0]
q2 = [0.0, 0.20, 0]

# box sizes
box_size1 = [0.05, 0.05, 0.05]
box_size2 = [0.05, 0.05, 0.05]


nonstack_pose1 = [[ 9.94885056e-01, -1.55900179e-02,  9.98031851e-02,  1.95741356e-02],
 [ 1.56650268e-02,  9.99877295e-01,  3.21017596e-05, -1.15839339e-04],
 [-9.97914393e-02,  1.53148200e-03,  9.95007198e-01,  5.83171660e-02],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]

nonstack_pose2 = [[-1.00114398e-01,  5.89905683e-03,  9.94958446e-01,  2.11092651e-01],
 [ 4.44194866e-04,  9.99982590e-01, -5.88414908e-03,  2.24353720e-03],
 [-9.94975834e-01, -1.47132606e-04, -1.00115275e-01,  3.89206165e-02],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]


stack_pose1 = [[ 9.94996362e-01, -3.77459725e-05,  9.99111553e-02,  2.64605688e-01],
 [ 3.79942503e-05,  9.99999999e-01, -5.82200282e-07, -1.13024604e-06],
 [-9.99111552e-02,  4.37533660e-06,  9.94996362e-01,  3.33522993e-02],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]

stack_pose2 = [[ 9.94996470e-01, -1.41453220e-05,  9.99100858e-02,  2.66676737e-01],
 [ 1.41925281e-05,  1.00000000e+00,  2.38281298e-07, -9.14267754e-07],
 [-9.99100858e-02,  1.18088765e-06,  9.94996470e-01,  5.22415534e-02],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]


class PrintContactResults(LeafSystem):
    """ Helpers for printing contact results
    """

    def __init__(self):
        LeafSystem.__init__(self)
        self.DeclareAbstractInputPort("contact_results",
                                      AbstractValue.Make(ContactResults()))
        self.DeclareForcedPublishEvent(self.Publish)

    def Publish(self, context):
        formatter = {'float': lambda x: '{:5.2f}'.format(x)}
        results = self.get_input_port().Eval(context)

        clear_output(wait=True)
        if results.num_point_pair_contacts() == 0:
            print("no contact")
        for i in range(results.num_point_pair_contacts()):
            info = results.point_pair_contact_info(i)
            pair = info.point_pair()
            force_string = np.array2string(
                info.contact_force(), formatter=formatter)
            print(
                f"slip speed:{info.slip_speed():.4f}, "
                f"depth:{pair.depth:.4f}, "
                f"force:{force_string}\n")
        return EventStatus.Succeeded()


def AddGTBox(plant, visual_shape, name, pose, color=[.5, .5, .9, 1.0]):
    instance = plant.AddModelInstance(name + "gt")
    plant.RegisterVisualGeometry(plant.world_body(), RigidTransform(pose), visual_shape, name + "gt", color)


def AddBoxDifferentGeometry(plant, visual_shape, collision_shape, name, mass=1, mu=1, color=[.5, .5, .9, 1.0]):
    instance = plant.AddModelInstance(name)
    # inertia = UnitInertia.SolidBox(visual_shape.width(), visual_shape.depth(),
    #                                visual_shape.height())

    body = plant.AddRigidBody(
        name, instance,
        SpatialInertia(mass=mass,
                       p_PScm_E=np.array([0., 0., 0.]),
                       G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
    if plant.geometry_source_is_registered():
        """ register collision geometry"""
        plant.RegisterCollisionGeometry(body, RigidTransform(), collision_shape, name,
                                        CoulombFriction(mu, mu))
        """ register visual geometry"""
        plant.RegisterVisualGeometry(body, RigidTransform(), visual_shape, name, color)
    return


q1_teleop = [0.0, 0.15, 0]
q2_teleop = [0.0, 0.20, 0]

# box sizes
box_size1_teleop = [0.05, 0.05, 0.05]
box_size2_teleop = [0.05, 0.05, 0.05]


def make_teleop_simulation(time_step, mass, mu, slope_size=[10, 10, 0.1], mu_g=0.1, interactive=True):
    """
    a nice interface to inspect contact forces
    YOUR CODE HERE
    """
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.01)
    box = Box(*slope_size)
    X_WBox = RigidTransform(RotationMatrix.MakeYRotation(slope), [0, 0, 0])
    plant.RegisterCollisionGeometry(plant.world_body(), X_WBox, box, "ground",
                                    CoulombFriction(mu_g, mu_g))
    plant.RegisterVisualGeometry(plant.world_body(), X_WBox, box, "ground",
                                 [.9, .9, .9, 1.0])
    #
    # terrain = AddTerrain(plant)
    # golf_ball = AddGolfBall(plant)

    shape_ball = Mesh('robot/golf_ball/golf_ball.obj', scale=0.03)
    box_instance = AddBoxDifferentGeometry(plant, shape_ball, shape_ball, "box",
                                           mass[0], mu[0], color=[0.8, 0, 0, 1.0])
    # box_instance = AddGolfBall(plant)

    frame = plant.AddFrame(
        FixedOffsetFrame(
            "planar_joint_frame", plant.world_frame(),
            RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2))))
    plant.AddJoint(
        PlanarJoint("box",
                    frame,
                    plant.GetFrameByName("box"),
                    damping=[0, 0, 0]))

    box_instance_2 = AddBoxDifferentGeometry(plant, Box(*box_size2_teleop), Box(*[x for x in box_size2_teleop]),
                                             "box_2",
                                             mass[1], mu[1], color=[0, 0.8, 0, 1.0])

    frame_2 = plant.AddFrame(
        FixedOffsetFrame(
            "planar_joint_frame_2", plant.world_frame(),
            RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2))))
    plant.AddJoint(
        PlanarJoint("box_2",
                    frame_2,
                    plant.GetFrameByName("box_2"),
                    damping=[0, 0, 0]))
    plant.Finalize()

    """ meshcat visualization """
    meshcat.Delete()
    meshcat.DeleteAddedControls()
    meshcat_param = MeshcatVisualizerParams()

    """ kProximity for collision geometry and kIllustration for visual geometry """
    meshcat_param.role = Role.kIllustration
    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, meshcat, meshcat_param)
    meshcat.Set2dRenderMode(xmin=-.2, xmax=.2, ymin=-.2, ymax=0.3)
    print_contact_results = builder.AddSystem(PrintContactResults())
    builder.Connect(plant.get_contact_results_output_port(),
                    print_contact_results.get_input_port())

    """ visualize contact force """
    cparams = ContactVisualizerParams()
    cparams.force_threshold = 1e-4
    cparams.newtons_per_meter = 2
    cparams.radius = 0.001
    contact_visualizer = ContactVisualizer.AddToBuilder(
        builder, plant, meshcat, cparams)

    """ add joint slider """
    default_interactive_timeout = None # if running_as_notebook else 1.0
    lower_limit = [-0.2, -0.2, -np.pi / 2.0]
    upper_limit = [0.2, 0.2, np.pi / 2.0]
    lower_limit += lower_limit
    upper_limit += upper_limit
    sliders = builder.AddSystem(
        JointSliders(meshcat,
                     plant,
                     initial_value=q1_teleop + q2_teleop,
                     lower_limit=lower_limit,
                     upper_limit=upper_limit,
                     step=0.001))

    diagram = builder.Build()
    # if running_as_notebook and interactive:
    sliders.Run(diagram, default_interactive_timeout)
    meshcat.DeleteAddedControls()
    return plant, diagram

plant_a, diagram_a = make_teleop_simulation(0.01, (0.1, 0.1), (0.1, 0.1))