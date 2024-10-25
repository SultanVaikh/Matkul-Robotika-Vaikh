import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_1', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    pose_goal = Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)

    move_group.stop()
    move_group.clear_pose_targets()

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass