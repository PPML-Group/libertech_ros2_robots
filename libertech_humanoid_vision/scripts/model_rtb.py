#!/usr/bin/env python3
from roboticstoolbox.robot.ERobot import ERobot
from math import pi
import numpy as np
np.set_printoptions(precision=6, suppress=True)

class ModelRtb(ERobot):
    def __init__(self,
                 robot_path,
                 display=True):
        links, name, urdf_string, urdf_filepath = self.URDF_read(robot_path)
        super().__init__(
            links,
            name=name.upper(),
            manufacturer="Libertech",
            # gripper_links=links[7],
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )
        
        if display:
            print(self)
            
        # fk_rtb = self.model_rtb.fkine(q,end=self.model_rtb.link_dict[name]) # SE3
        # self.model_rtb.ikine_LM(pos,end=self.model_rtb.link_dict[name]).q
        # print(f'{j} \n {self.model_rtb.jacob0(q,end=self.model_rtb.link_dict[name])}')
        # print(f'{j} \n {self.model_rtb.jacobe(q,end=self.model_rtb.link_dict[name])}')
