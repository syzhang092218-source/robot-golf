import os
import numpy as np
import pybullet as p
import cv2

from PIL import Image


def make_video_linear(
        name: str,
        duration: float,
        camera_init_pos,
        camera_final_pos,
        target_init_pos,
        target_final_pos,
        fov_init,
        fov_final,
        simulation_step=None,
        movement=None,
        width=1280,
        height=720,
        frame_rate=24,
        release=False,
):
    if release:
        video = []
        n_frame = np.round(frame_rate * duration)
        camera_step = (camera_final_pos - camera_init_pos) / n_frame
        target_step = (target_final_pos - target_init_pos) / n_frame
        fov_step = (fov_final - fov_init) / n_frame
        if simulation_step is None:
            simulation_step = n_frame
        assert simulation_step >= n_frame
        step_per_frame = simulation_step // n_frame

        i_frame = 0
        for i_step in range(1, simulation_step + 1):
            if movement is not None:
                movement()
            if i_step % step_per_frame == 0:
        # for i_frame in range(n_frame):
                projectionMatrix = p.computeProjectionMatrixFOV(
                    fov=fov_init + i_frame * fov_step,
                    aspect=width / height,
                    nearVal=0.1,
                    farVal=50
                )
                viewMatrix = p.computeViewMatrix(
                    cameraEyePosition=list(i_frame * camera_step + camera_init_pos),
                    cameraTargetPosition=list(i_frame * target_step + target_init_pos),
                    cameraUpVector=[0, 0, 1]
                )
                width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                    width=width,
                    height=height,
                    viewMatrix=viewMatrix,
                    projectionMatrix=projectionMatrix,
                    renderer=p.ER_BULLET_HARDWARE_OPENGL
                )
                video.append(rgbImg)
                i_frame += 1
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(f'{name}.mp4', fourcc, frame_rate, (width, height))
        if not os.path.exists(f'video/{name}'):
            os.mkdir(f'video/{name}')
        for i_img, img in enumerate(video):
            img_pil = Image.fromarray(img, 'RGBA')
            img_rgb = img_pil.convert('RGB')
            img_rgb.save(f'video/{name}/{i_img}.png')
            img_new = cv2.imread(f'video/{name}/{i_img}.png')
            out.write(img_new)
        out.release()
    else:
        if movement is not None:
            for i_step in range(1, simulation_step + 1):
                movement()
