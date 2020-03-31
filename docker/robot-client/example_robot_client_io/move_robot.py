from frankapy import FrankaArm


if __name__ == "__main__":
    fa = FrankaArm()
    
    fa.reset_joints()
    pose = fa.get_pose()
    pose.translation[0] += 0.1
    fa.goto_pose(pose)
    fa.reset_joints()
