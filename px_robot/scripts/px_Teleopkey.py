import termios, sys, os
import rospy
from numpy import array

TERMIOS = termios
import time
from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand


def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

def getkey(): #Teclado
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    c = str(c).replace('b', "").replace('\'', "")
    return c

def deg2raw(input_list: list = [0,0,0,0,0], min_deg: int = -150, max_deg: int = 150)->list:
    out_list = [0,0,0,0,0]
    for i in range(len(input_list)):
        out_list[i] = int( ((input_list[i] - min_deg)*1024)/(max_deg-min_deg) )
    return out_list



def main(goal_position: list = [30,45,-30,-60,700], home_position: list = [0,0,0,0,0]):

    motors_ids = [1,2,3,4,5]
    goal_position_raw = deg2raw(goal_position)
    home_position_raw = deg2raw(home_position)
    selected_link = "Waist"
    print("_________________________________")
    print("Link: ",selected_link)
    
    # Goal_Position (0,1023)
    # Torque_Limit (0,1023)
    jointCommand('', motors_ids[0], 'Torque_Limit', 600, 0)
    jointCommand('', motors_ids[1], 'Torque_Limit', 400, 0)
    jointCommand('', motors_ids[2], 'Torque_Limit', 400, 0)
    jointCommand('', motors_ids[3], 'Torque_Limit', 400, 0)
    jointCommand('', motors_ids[4], 'Torque_Limit', 400, 0)

    while(True):
        key = getkey()
        
        if(selected_link == "Waist"):
            if(key == "w"):
                selected_link = "Shoulder"
                print("_________________________________")
                print("Link: ",selected_link)
            elif(key == "s"):
                print("_________________________________")
                selected_link = "Gripper"
                print("Link: ",selected_link)
            elif(key == "d"):
                print("Moviendo a la posición objetivo ...")
                jointCommand('', motors_ids[0], 'Goal_Position', goal_position_raw[0], 0.5)
            elif(key == "a"):
                print("Moviendo a la posición HOME ...")
                jointCommand('', motors_ids[0], 'Goal_Position', home_position_raw[0], 0.5)
                
        elif(selected_link == "Shoulder"):
            if(key == "w"):
                selected_link = "Elbow"
                print("_________________________________")
                print("Link: ",selected_link)
            elif(key == "s"):
                print("_________________________________")
                selected_link = "Waist"
                print("link: ",selected_link)
            elif(key == "d"):
                print("Moviendo a la posición objetivo ...")
                jointCommand('', motors_ids[1], 'Goal_Position', goal_position_raw[1], 0.5)
            elif(key == "a"):
                print("Moviendo a la posición HOME ...")
                jointCommand('', motors_ids[1], 'Goal_Position', home_position_raw[1], 0.5)

        elif(selected_link == "Elbow"):
            if(key == "w"):
                selected_link = "Wrist"
                print("_________________________________")
                print("Link: ",selected_link)
            elif(key == "s"):
                print("_________________________________")
                selected_link = "Shoulder"
                print("Link: ",selected_link)
            elif(key == "d"):
                print("Moviendo a la posición objetivo ...")
                jointCommand('', motors_ids[2], 'Goal_Position', goal_position_raw[2], 0.5)
            elif(key == "a"):
                print("Moviendo a la posición HOME ...")
                jointCommand('', motors_ids[2], 'Goal_Position', home_position_raw[2], 0.5)

        elif(selected_link == "Wrist"):
            if(key == "w"):
                selected_link = "Gripper"
                print("_________________________________")
                print("Link: ",selected_link)
            elif(key == "s"):
                print("_________________________________")
                selected_link = "Elbow"
                print("Link: ",selected_link)
            elif(key == "d"):
                print("Moviendo a la posición objetivo ...")
                jointCommand('', motors_ids[3], 'Goal_Position', goal_position_raw[3], 0.5)
            elif(key == "a"):
                print("Moviendo a la posición HOME ...")
                jointCommand('', motors_ids[3], 'Goal_Position', home_position_raw[3], 0.5)


        elif(selected_link == "Gripper"):
            if(key == "w"):
                selected_link = "Waist"
                print("_________________________________")
                print("Link: ",selected_link)
            elif(key == "s"):
                print("_________________________________")
                selected_link = "Wrist"
                print("Link: ",selected_link)
            elif(key == "d"):
                print("Moviendo a la posición objetivo ...")
                jointCommand('', motors_ids[4], 'Goal_Position', goal_position_raw[4], 0.5)
            elif(key == "a"):
                print("Moviendo a la posición HOME ...")
                jointCommand('', motors_ids[4], 'Goal_Position', home_position_raw[4], 0.5)



if __name__ == '__main__':
    try:
        main()

        
    except rospy.ROSInterruptException:
        pass
