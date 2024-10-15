import numpy as np
from pydrake.systems.framework import LeafSystem, BasicVector, Context
from pydrake.common.value import AbstractValue # type: ignore
from pydrake.math import RigidTransform
from pydrake.perception import PointCloud

class GripperMotionController(LeafSystem):
    def __init__(self):
        super().__init__()
        
        self.DeclareVectorInputPort("svh_state", BasicVector(np.zeros()))
        self.DeclareVectorInputPort("svh_net_actuation", BasicVector(np.zeros()))
        self.DeclareVectorOutputPort("svh_desired_state", BasicVector(np.zeros()))
        self.DeclareVectorOutputPort("svh_feedforward Torque", BasicVector(np.zeros()))
        self.DeclareAbstractInputPort("X_OG", AbstractValue().Make(RigidTransform()))
        self.DeclareAbstractInputPort("object_point_cloud", AbstractValue().Make(PointCloud()))

        def CalcDesiredState(self, context:Context, output:BasicVector):
            output.SetZero()

        def CalcFeedforwardTorque(self, context:Context, output:BasicVector):
            output.SetZero()
