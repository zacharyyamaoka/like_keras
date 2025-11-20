from .ur import UR
# from .one_dof import OneDofDescription
from .dummy_dh import DummyDH
# from .dummy_five_bar import DummyFiveBar
from .bam_fb import BamFb

# Import all arms here so they can be easily imported directly
# Also register them so they can be made via RobotDescription.make()
# RobotDescription.register("ur5e", UR5e)
# RobotDescription.register("one_dof", OneDofDescription)
# RobotDescription.register("arm_400_v1", Arm400v1)
# RobotDescription.register("dummy_dh", DummyDH)
# RobotDescription.register("bam_arm_prototype", BamArmPrototype)