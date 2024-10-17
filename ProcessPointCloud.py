from pydrake.systems.framework import LeafSystem
from pydrake.common.value import AbstractValue # type: ignore
from pydrake.perception import PointCloud, Concatenate, BaseField, Fields
from pydrake.all import Parallelism

class ProcessPointCloud(LeafSystem):
    def __init__(self, fields=BaseField.kXYZs):
        super().__init__()
        self.fields_ = fields
        self.DeclareAbstractInputPort("point_cloud_in_0", AbstractValue.Make(PointCloud(0, Fields(fields))))
        self.DeclareAbstractInputPort("point_cloud_in_1", AbstractValue.Make(PointCloud(0, Fields(fields))))
        self.DeclareAbstractOutputPort("point_cloud", self.alloc_concat, self.calc_concat)

    def calc_concat(self, context, state):
        input_val_0:PointCloud = self.get_input_port(0).EvalAbstract(context).get_value().Crop([0.3, -0.5, 0.005], [1.0, 0.5, 0.25])
        input_val_1:PointCloud = self.get_input_port(1).EvalAbstract(context).get_value().Crop([0.3, -0.5, 0.005], [1.0, 0.5, 0.25])
        raw_pc = Concatenate([input_val_0, input_val_1])
        raw_pc.VoxelizedDownSample(0.005, Parallelism(12))
        state.SetFrom(AbstractValue.Make(raw_pc))

    def alloc_concat(self):
        return AbstractValue.Make(PointCloud(0, Fields(self.fields_)))
