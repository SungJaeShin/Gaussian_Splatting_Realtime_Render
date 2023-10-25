import torch
import torchvision
import os
from os import makedirs
from tqdm import tqdm
from gaussian_renderer import render, GaussianModel
from diff_gaussian_rasterization import GaussianRasterizationSettings, GaussianRasterizer
from scene import Scene
from scene.gaussian_model import GaussianModel
from utils.general_utils import safe_state
from utils.graphics_utils import getWorld2View2, getProjectionMatrix
from utils.sh_utils import eval_sh
from argparse import ArgumentParser
from arguments import ModelParams, PipelineParams, get_combined_args
import math

import numpy as np
import rospy
from nav_msgs.msg import Odometry
import pdb
from collections import deque

import threading
import time
import cv2 as cv

class LCD_NeRF:
    def __init__(self, gaussian, scenes, bg_color, background):
        # For basic setting of Gaussian Splatting Model
        self.gaussian_model = gaussian
        self.scene = scenes
        self.bg_color = bg_color
        self.background = background

        # Setting basic rendering related parameters 
        self.FoVx = scene.getTrainCameras()[0].FoVx
        self.FoVy = scene.getTrainCameras()[0].FoVy
        self.zfar = scene.getTrainCameras()[0].zfar
        self.znear = scene.getTrainCameras()[0].znear
        self.trans = scene.getTrainCameras()[0].trans
        self.scale = scene.getTrainCameras()[0].scale

        # For Subscribe Keyframe pose 
        self.keyframe_pose_buf = deque()
        self.sub_keyframe_pose = rospy.Subscriber("/vins_estimator/keyframe_pose", Odometry, self.keyframe_pose_callback)

        # For Mutex and Multi-thread
        self.lock = threading.Lock()
        threading.Thread(target=self.Rendering_2D_img).start()
        # self.thread = threading.Thread(target=self.Rendering_2D_img, args=cur_pose)
        # self.thread.start()
        
    def quaternion_rotation_matrix(self, Q):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
        Ref: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/        
        """
        # Extract the values from Q = w + xi + yj + zk = q0 + q1i + q2j + q3k
        q0 = Q.w
        q1 = Q.x
        q2 = Q.y
        q3 = Q.z
            
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
            
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
            
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
            
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                               [r10, r11, r12],
                               [r20, r21, r22]])
                                
        return rot_matrix

    def keyframe_pose_callback(self, pose):
        self.lock.acquire()
        keyframe_pose = pose.pose.pose
        self.keyframe_pose_buf.append(keyframe_pose)
        # print("Keyframe Pose: ", self.keyframe_pose_buf[-1]) # for debugging ! 
        self.lock.release()

    def make_camera_matrices(self, T, R):
        world_T = torch.tensor(getWorld2View2(R, T, self.trans, self.scale)).transpose(0, 1).cuda()
        proj_T = getProjectionMatrix(znear=self.znear, zfar=self.zfar, fovX=self.FoVx, fovY=self.FoVy).transpose(0,1).cuda()
        full_proj_T = (world_T.unsqueeze(0).bmm(proj_T.unsqueeze(0))).squeeze(0)
        camera_center = world_T.inverse()[3, :3] 

        return world_T, proj_T, full_proj_T, camera_center

    def rasterization_setting(self, world_T, full_proj_T, camera_center):       
        raster_settings = GaussianRasterizationSettings(            
            image_height=480,
            image_width=640,
            tanfovx=math.tan(self.FoVx * 0.5),
            tanfovy=math.tan(self.FoVy * 0.5),
            bg=self.background,
            scale_modifier=1.0,
            viewmatrix=world_T,
            projmatrix=full_proj_T,
            sh_degree=self.gaussian_model.active_sh_degree,
            campos=camera_center,
            prefiltered=False,
            debug=False
        )

        return GaussianRasterizer(raster_settings=raster_settings)

    def Rendering_2D_img(self):
        # self.thread = threading.Thread(target=self.__init__).start()
        
        while True:        
            if len(self.keyframe_pose_buf) == 0:
                time.sleep(0.5)
                continue
            
            # Mutex for keyframe_pose_buf
            self.lock.acquire()

            # Get current pose
            cur_pose = self.keyframe_pose_buf.popleft()
            cur_translation = np.array([cur_pose.position.x, cur_pose.position.y, cur_pose.position.z])
            cur_orientation = self.quaternion_rotation_matrix(cur_pose.orientation)

            # Get rasterization using camera related matrices
            world_T, proj_T, full_proj_T, camera_center = self.make_camera_matrices(cur_translation, cur_orientation)
            rasterizer = self.rasterization_setting(world_T, full_proj_T, camera_center)

            # Setting to get Render 2D image !
            screenspace_points = torch.zeros_like(self.gaussian_model.get_xyz, dtype=self.gaussian_model.get_xyz.dtype, requires_grad=True, device="cuda") + 0
            try:
                screenspace_points.retain_grad()
            except:
                pass

            means3D = self.gaussian_model.get_xyz
            means2D = screenspace_points
            opacity = self.gaussian_model.get_opacity
            scales = self.gaussian_model.get_scaling
            rotations = self.gaussian_model.get_rotation
            cov3D_precomp = None
            shs = self.gaussian_model.get_features
            colors_precomp = None

            # Get 2D Render image !!
            rendered_image, radii = rasterizer(
                means3D = means3D,
                means2D = means2D,
                shs = shs,
                colors_precomp = colors_precomp,
                opacities = opacity,
                scales = scales,
                rotations = rotations,
                cov3D_precomp = cov3D_precomp
            )
    
            self.lock.release()

            # To Show rendered image
            rendered_image_cv = rendered_image.cpu().detach().numpy().transpose(1, 2, 0)
            cv.imshow("Render 2D image", rendered_image_cv)
            cv.waitKey(1)
            time.sleep(0.5)

if __name__ == "__main__":
    parser = ArgumentParser(description="Testing script parameters")
    model = ModelParams(parser, sentinel=True)
    pipeline = PipelineParams(parser)
    parser.add_argument("--iteration", default=-1, type=int)
    args = get_combined_args(parser)
    print("Rendering: " + args.model_path)
    
    # Get Trained Gaussian Model and Scenes    
    gaussians = GaussianModel(model.extract(args).sh_degree)
    scene = Scene(model.extract(args), gaussians, load_iteration=args.iteration, shuffle=False)
    
    # Get Background Color
    bg_color = [1,1,1] if model.extract(args).white_background else [0, 0, 0]
    background = torch.tensor(bg_color, dtype=torch.float32, device="cuda")

    # 2D Rendering Class
    print("===== Rendering Node =====")
    rospy.init_node("Rendering_Node")
    LCD_NeRF(gaussians, scene, bg_color, background)    

    rospy.spin()






