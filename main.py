import pybullet as p
import pybullet_data
import numpy as np
import torch
import argparse

from sklearn import linear_model

from robot_golf.env.ball import ball_aerodynamics
from robot_golf.sys_id.sys_id import prepare
from robot_golf.sys_id.predictor import QDotPredictor
from robot_golf.sys_id.dyn_trainer import DynTrainer


def main(args):
    # init
    p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    ball_pos = np.array([0.0, -0.5, 0.35])
    robot_id, joint_pos, ball_id, v_ball_init, hole_pos = prepare(ball_pos)

    # fitting
    valid_qdots = np.load('robot_golf/sys_id/data/qdots_6300.npy', allow_pickle=True)
    valid_qdots = valid_qdots.astype(np.float64)
    valid_vs = np.load('robot_golf/sys_id/data/vs_6300.npy', allow_pickle=True)
    valid_vs = valid_vs.astype(np.float64)
    input_all = valid_vs.copy()

    # ransac
    ransac_qdot1 = linear_model.RANSACRegressor(max_trials=1000000, residual_threshold=2)
    ransac_qdot1.fit(input_all, valid_qdots[:, 0])
    ransac_qdot2 = linear_model.RANSACRegressor()
    ransac_qdot2.fit(input_all, valid_qdots[:, 1])
    inlier_mask = ransac_qdot1.inlier_mask_

    if args.no_learning:
        qdot_1 = ransac_qdot1.predict(v_ball_init.reshape(1, -1))[0]
        qdot_2 = ransac_qdot2.predict(v_ball_init.reshape(1, -1))[0]
    else:
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        if args.train:
            # learn dynamics
            qdot_predictor = QDotPredictor(device=device).to(device)
            dyn_trainer = DynTrainer(qdot_predictor, device,
                                     qdots=valid_qdots[inlier_mask, :], vs=valid_vs[inlier_mask, :])
            dyn_trainer.train(n_iter=40960)
        else:
            # use pre-trained model
            qdot_predictor = torch.load('robot_golf/sys_id/data/40960.pkl', map_location=device)

        qdots = qdot_predictor(torch.tensor(v_ball_init, device=device).unsqueeze(0).type(torch.float))
        qdot_1 = qdots[0, 0]
        qdot_2 = qdots[0, 1]

    # hitting
    n_joints = p.getNumJoints(robot_id)
    rest_in_hole = False
    while True:
        # joint control
        for i in range(n_joints - 3):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0)
        p.setJointMotorControl2(robot_id, n_joints - 3, p.POSITION_CONTROL, joint_pos[-1], qdot_1, maxVelocity=10000)
        p.setJointMotorControl2(robot_id, n_joints - 4, p.POSITION_CONTROL, joint_pos[-2], qdot_2, maxVelocity=10000)

        # apply aerodynamics
        ball_aerodynamics(ball_id)

        # do simulation
        p.stepSimulation()
        p.addUserDebugPoints([p.getBasePositionAndOrientation(ball_id)[0]], [[1, 0, 0]], pointSize=3)

        if np.linalg.norm(np.array(p.getBasePositionAndOrientation(ball_id)[0]) - np.array(
                [hole_pos[0], hole_pos[1], -.626])) < 0.05 and \
                np.linalg.norm(np.array(p.getBaseVelocity(ball_id)[0])) < 0.3 and not rest_in_hole:
            rest_in_hole = True
            print('Goal!')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('--no-learning', action='store_true', default=False,
                        help='If true, use linear fitting; else, use supervised learning')
    parser.add_argument('--train', action='store_true', default=False,
                        help='If true, train the learning model from scratch; else, use pretrained model')

    args = parser.parse_args()
    main(args)
