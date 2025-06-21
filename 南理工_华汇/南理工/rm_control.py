from robotic_arm import *
import time
from convert import convert 
import math
class RobotArmController():
    def __init__(self, ip_address="192.168.10.18", model=RM65):
        """初始化并连接机械臂，支持指定IP地址和型号"""
        self.robot = Arm(model, ip_address)  # 直接传递字符串，不需要转换
        if self.robot.API_Version():
            print(f"成功连接到机械臂，IP地址：{ip_address}")
        else:
            print("连接机械臂失败。")
    def init_arm(self, pose,joint, v=35, r=0,  speed_gripper=500, height_lift=0, speed_lift=30, block=True,timeout=30):
        """初始化机械臂"""
        # 机械臂回归初始位置
        self.set_joint_en_state(7,True, block=True)
        print("关节使能完成")
        time.sleep(0.5)
        self.robot.Movej_Cmd(joint=joint,v=v, r=r, trajectory_connect=0, block=True)
        self.robot.Movej_P_Cmd(pose, v, r, trajectory_connect=0, block=True)
        print("机械臂回到初始位置完成")
        # 升降柱到初始位置
        #self.robot.Set_Lift_Height( height_lift, speed_lift, block=True)
        #print("升降机初始化完成。")
        self.DH_init(port=1, baudrate=115200,block=True)

    def DH_init(self,port=1, baudrate=115200,block=True):
        """
        大寰夹爪初始化
        """
        self.set_Tool_Voltage(3, block=True)
        # 初始化夹爪（此时才启动Modbus通讯）
        self.set_Modbus_Mode(port=port, baudrate=baudrate, timeout=3, block=block)
        #关闭io模式
        self.write_Single_Register(1, 0x0402, 0, 1, True)
        print("关闭io")
        # 完全初始化（重新标定最大最小行程）
        self.write_Single_Register(1, 0x0100, 0xA5, 1, True)
        print("完全初始化行程")
        time.sleep(2)
        print("回零完成")

    def get_joint_en_state(self):
        """
        获取关节使能状态
        """
        try:
            result, state = self.robot.Get_Joint_EN_State()
            if result == 0:
                print(f"Joint enable state: {state}")
            else:
                print(f"Failed to get joint enable state. Error code: {result}")
            return result, state
        except Exception as e:
            print(f"An error occurred while getting joint enable state: {e}")
            return -1, None  # 返回失败的错误码和 None
    
    def set_joint_en_state(self, joint_num, state, block=True):
        """
        设置指定关节的使能状态
        
        参数：
        - joint_num: 关节序号，1~7
        - state: True-使能，False-掉使能
        - block: True-阻塞，False-非阻塞
        """
        try:
            result = self.robot.Set_Joint_EN_State(joint_num, state, block)
            if result == 0:
                print(f"Joint {joint_num} enable state set to {'enabled' if state else 'disabled'}")
            else:
                print(f"Failed to set joint {joint_num} enable state. Error code: {result}")
            return result
        except Exception as e:
            print(f"An error occurred while setting joint {joint_num} enable state: {e}")
            return -1  # 返回失败的错误码

    def get_current_tool_frame(self):
        """
        获取当前工具坐标系
        
        返回：
        - 0: 成功，返回工具坐标系
        - 错误码: 失败
        """
        try:
            result, tool = self.robot.Get_Current_Tool_Frame()
            if result == 0:
                print(f"Current tool frame: {tool}")
            else:
                print(f"Failed to get current tool frame. Error code: {result}")
            return result, tool
        except Exception as e:
            print(f"An error occurred while getting current tool frame: {e}")
            return -1, None  # 返回失败的错误码和 None
    
    def get_given_tool_frame(self, name):
        """
        获取指定工具坐标系
        
        参数：
        - name: 目标工具坐标系的名称
        
        返回：
        - 0: 成功，返回指定工具坐标系
        - 错误码: 失败
        """
        try:
            result, tool = self.robot.Get_Given_Tool_Frame(name)
            if result == 0:
                print(f"Tool frame '{name}': {tool}")
            else:
                print(f"Failed to get tool frame '{name}'. Error code: {result}")
            return result, tool
        except Exception as e:
            print(f"An error occurred while getting tool frame '{name}': {e}")
            return -1, None  # 返回失败的错误码和 None
    
    def get_all_tool_frames(self):
        """
        获取所有工具坐标系名称
        
        返回：
        - 0: 成功，返回工具坐标系名称数组和数量
        - 错误码: 失败
        """
        try:
            result, names, length = self.robot.Get_All_Tool_Frame()
            if result == 0:
                print(f"All tool frames: {names}, Number of tool frames: {length}")
            else:
                print(f"Failed to get all tool frames. Error code: {result}")
            return result, names, length
        except Exception as e:
            print(f"An error occurred while getting all tool frames: {e}")
            return -1, None, 0  # 返回失败的错误码和 None


    def set_joint_parameters(self, joint_speeds=[180]*7, joint_accs=[600]*7, joint_limits=[(-180, 180), (-180, 180), (-225, 225), (-225, 225), (-225, 225), (-225, 225), (-225, 225)]):
        """设置关节的速度、加速度和限位参数"""
        # 设置每个关节的最大速度、加速度和位置限位
        for i in range(7):
            self.robot.Set_Joint_Speed(i+1, joint_speeds[i], True)  # 设置关节最大速度
            self.robot.Set_Joint_Acc(i+1, joint_accs[i], True)  # 设置关节最大加速度
            self.robot.Set_Joint_Min_Pos(i+1, joint_limits[i][0], True)  # 设置关节最小位置
            self.robot.Set_Joint_Max_Pos(i+1, joint_limits[i][1], True)  # 设置关节最大位置
        
        print("关节参数设置成功。")

    def change_work_frame(self, name="World"):
            """
            切换到某个工作坐标系，默认World坐标系
            """
            tag = self.robot.Change_Work_Frame(name)  # 修改为 self.robot
            print(f'change_work_frame:{tag}')
            time.sleep(0.5)
            return tag
    def change_tool_frame(self, name="grip"):
        """
        切换到某个工具坐标系，默认grip坐标系
        """
        tag = self.robot.Change_Tool_Frame(name)  # 修改为 self.robot
        print(f'change_tool_frame:{tag}')
        time.sleep(0.5)
        return tag

    def move_stop(self, block=True):
        """
        快速急停
        """
        tag = self.robot.Move_Stop_Cmd(block)  # 修改为 self.robot
        print(f'move_stop:{tag}')
        return tag
    
    def move_pause(self, block=True):
        """
        暂停运动
        """
        tag = self.robot.Move_Pause_Cmd(block)  # 修改为 self.robot
        print(f'move_pause:{tag}')
        return tag
    
    def check_pose_validity(self, pose):
            """
            检查目标位姿是否超出机械臂的有效工作半径（600mm）
            """
            # 计算目标位姿的笛卡尔坐标系下的距离
            x, y, z = pose[:3]  # 假设pose的前三个元素是位置坐标 (x, y, z)
            distance = math.sqrt(x**2 + y**2 + z**2)  # 计算原点到目标点的距离

            # 如果距离大于600mm，返回False
            if distance > 1.025:  # 600+175=775
                print(f"Error: The pose is out of the valid working radius (750mm). Distance: {distance:.2f}m.")
                return False
            return True

    def movej_cmd(self, joint, v, r, trajectory_connect=0, block=True):
        """
        movej_cmd 关节空间运动
        ArmSocket socket句柄
        joint 目标关节1~7角度数组
        v 速度百分比系数，1~100。
        r 交融半径百分比系数，0~100。
        trajectory_connect 代表是否和下一条运动一起规划，0代表立即规划，1代表和下一条轨迹一起规划，当为1时，轨迹不会立即执行
        block True 阻塞 False 非阻塞
        return 0-成功，失败返回:错误码, rm_define.h查询.
        """

        tag = self.robot.Movej_Cmd(joint, v, r, trajectory_connect=0, block=True)  # 修改为 self.robot
        print(f'movej_cmd:{tag}')
        return tag

    def movel_cmd(self, pose, v=10, r=0, trajectory_connect=0, block=True):
        """
        笛卡尔空间直线运动
        
        pose 目标位姿,位置单位：米，姿态单位：弧度
        v 速度百分比系数，1~100。
        r 交融半径百分比系数，0~100。
        trajectory_connect 代表是否和下一条运动一起规划，0代表立即规划，1代表和下一条轨迹一起规划，当为1时，轨迹不会立即执行
        block True 阻塞 False 非阻塞
        
        return:0-成功，失败返回:错误码, rm_define.h查询
        """
        # 校验目标位姿是否在有效范围内
        if not self.check_pose_validity(pose):
            return -1  # 返回错误码
        
        tag = self.robot.Movel_Cmd(pose, v, r, trajectory_connect=0, block=True)  # 修改为 self.robot
        print(f'movecmd:{tag}')
        return tag
    
    def movej_p_cmd(self, pose, v, r, trajectory_connect=0, block=True):
        """
        该函数用于关节空间运动到目标位姿
        param ArmSocket socket句柄
        pose: 目标位姿，位置单位：米，姿态单位：弧度。 注意：目标位姿必须是机械臂当前工具坐标系相对于当前工作坐标系的位姿，
              用户在使用该指令前务必确保，否则目标位姿会出错！！
        v 速度百分比系数，1~100。
        r 交融半径百分比系数，0~100。
        trajectory_connect: 代表是否和下一条运动一起规划，0代表立即规划，1代表和下一条轨迹一起规划，当为1时，轨迹不会立即执行
        block True 阻塞 False 非阻塞
        return 0-成功，失败返回:错误码
        """
        # 校验目标位姿是否在有效范围内
        if not self.check_pose_validity(pose):
            return -1  # 返回错误码
        
        tag = self.robot.Movej_P_Cmd(pose, v, r, trajectory_connect=0, block=True)  # 修改为 self.robot
        print(f'movej_p_cmd:{tag}')
        return tag
    
    def move_to_target_Directly_with_splitting(self,target_pose, v=10, r=0, block=True):
        """
        按照目标点位姿，分阶段拆分路径并执行笛卡尔空间直线运动
        1. 姿态对齐（旋转部分）
        2. Z轴对齐（深度方向）
        3. XY平面对齐
        :param current_pose: 当前坐标 (x, y, z, roll, pitch, yaw)
        :param target_pose: 目标坐标 (x, y, z, roll, pitch, yaw)
        :param num_points: 插值点数目，分割的路径点数量
        :param v: 速度百分比系数（1~100）
        :param r: 交融半径百分比系数（0~100）
        :param block: 是否阻塞，True为阻塞，False为非阻塞
        :return: 0 - 成功，非0 - 错误码
        """
        current_pose=self.get_current_arm_state()[2]
        print(f"当前位姿是{current_pose}")
        if not self.check_pose_validity(target_pose):
            print ("目标位姿超出有效范围")
            return -1  # 返回错误码
        # 步骤1：姿态对齐（旋转部分）
        # 这里只涉及旋转部分，位置不变化
        # 直接对目标点的姿态进行调整

        pose_stage1 = (current_pose[0], current_pose[1], current_pose[2], target_pose[3], target_pose[4], target_pose[5])
        result = self.movel_cmd(pose_stage1, v, r, trajectory_connect=0, block=block)
        if result != 0:
            print(f"姿态对齐失败，错误码：{result}")
            return result
        print("姿态对齐成功。")
        time.sleep(0.5)
        # 步骤2：Z轴对齐（深度方向）
        # 在姿态对齐之后，沿Z轴方向逐步移动，直到到达目标位置的Z轴
        pose_stage2 = (current_pose[0], current_pose[1], target_pose[2], target_pose[3], target_pose[4], target_pose[5])
        result = self.movel_cmd(pose_stage2, v, r, trajectory_connect=0, block=block)
        if result != 0:
            print(f"Z轴对齐失败，错误码：{result}")
            return result
        print("Z轴对齐成功。")
        time.sleep(0.5)
        # 步骤3：XY平面对齐
        # Z轴对齐之后，接下来沿XY平面对齐，直接到达目标位置的X、Y坐标
        pose_stage3 = (current_pose[0], target_pose[1], target_pose[2], target_pose[3], target_pose[4], target_pose[5])
        result = self.movel_cmd(pose_stage3, v, r, trajectory_connect=0, block=block)
        if result != 0:
            print(f"Y对齐失败，错误码：{result}")
            return result
        print("Y轴对齐成功。")
        pose_stage4 = (target_pose[0], target_pose[1], target_pose[2], target_pose[3], target_pose[4], target_pose[5])
        result = self.movel_cmd(pose_stage4, v, r, trajectory_connect=0, block=block)
        if result != 0:
            print(f"X平面对齐失败，错误码：{result}")
            return result
        print("X轴对齐成功。")
        print("已到目标点位")
        return 0  # 所有路径点成功执行，返回0表示成功

    def move_to_target_sideway_with_splitting(self,target_pose, v=10, r=0, block=True):
        """
        按照目标点位姿，分阶段拆分路径并执行笛卡尔空间直线运动
        1. 姿态对齐（旋转部分）
        2. Z轴对齐（深度方向）
        3. XY平面对齐
        :param current_pose: 当前坐标 (x, y, z, roll, pitch, yaw)
        :param target_pose: 目标坐标 (x, y, z, roll, pitch, yaw)
        :param num_points: 插值点数目，分割的路径点数量
        :param v: 速度百分比系数（1~100）
        :param r: 交融半径百分比系数（0~100）
        :param block: 是否阻塞，True为阻塞，False为非阻塞
        :return: 0 - 成功，非0 - 错误码
        """
        current_pose=self.get_current_arm_state()[2]
        print(f"当前位姿是{current_pose}")
        if not self.check_pose_validity(target_pose):
            print ("目标位姿超出有效范围")
            return -1  # 返回错误码
        # 步骤1：姿态对齐（旋转部分）
        # 这里只涉及旋转部分，位置不变化
        # 直接对目标点的姿态进行调整

        pose_stage1 = (current_pose[0], current_pose[1], current_pose[2], target_pose[3], target_pose[4], target_pose[5])
        result = self.movel_cmd(pose_stage1, v, r, trajectory_connect=0, block=block)
        if result != 0:
            print(f"姿态对齐失败，错误码：{result}")
            return result
        print("姿态对齐成功。")
        time.sleep(0.5)
        # 步骤2：Z轴对齐（深度方向）
        # 在姿态对齐之后，沿Z轴方向逐步移动，直到到达目标位置的Z轴
        pose_stage2 = (current_pose[0], current_pose[1], target_pose[2], target_pose[3], target_pose[4], target_pose[5])
        result = self.movel_cmd(pose_stage2, v, r, trajectory_connect=0, block=block)
        if result != 0:
            print(f"Z轴对齐失败，错误码：{result}")
            return result
        print("Z轴对齐成功。")
        time.sleep(0.5)
        # 步骤3：XY平面对齐
        # Z轴对齐之后，接下来沿XY平面对齐，直接到达目标位置的X、Y坐标
        pose_stage3 = (target_pose[0],current_pose[1], target_pose[2], target_pose[3], target_pose[4], target_pose[5])
        result = self.movel_cmd(pose_stage3, v, r, trajectory_connect=0, block=block)
        if result != 0:
            print(f"Y对齐失败，错误码：{result}")
            return result
        print("Y轴对齐成功。")
        pose_stage4 = (target_pose[0], target_pose[1], target_pose[2], target_pose[3], target_pose[4], target_pose[5])
        result = self.movel_cmd(pose_stage4, v, r, trajectory_connect=0, block=block)
        if result != 0:
            print(f"X平面对齐失败，错误码：{result}")
            return result
        print("X轴对齐成功。")
        print("已到目标点位")

        return 0  # 所有路径点成功执行，返回0表示成功


    def control_green_button(self, target_pose, v=10, r=0,speed=500,force=300, block=True):
        """
        根据识别位姿设计绿色按钮动作
        """
        print(f"绿色按钮动作开始。")
        target_pose1=[target_pose[0],target_pose[1]+0.05,target_pose[2]+0.02,target_pose[3],target_pose[4],target_pose[5]]
        target_pose2=[target_pose[0],target_pose[1]-0.025,target_pose[2]+0.02,target_pose[3],target_pose[4],target_pose[5]]
        target_pose3=[target_pose[0],target_pose[1]+0.05,target_pose[2]+0.02,target_pose[3],target_pose[4],target_pose[5]]
        self.move_to_target_sideway_with_splitting(target_pose1, v, r, block)
        print("已到达目标点前方。")
        #闭合夹爪
        self.control_gripper("close", speed, force, block)
        print("夹爪已闭合。")
        self.move_to_target_sideway_with_splitting(target_pose2, v, r, block)
        print("按钮成功按下。")
        time.sleep(0.5)
        self.move_to_target_sideway_with_splitting(target_pose3, v, r, block)
        print("操作已完成")
        # print("操作已完成")


    def control_red_button(self, target_pose, v=10, r=0,speed=500,force=300, block=True):
        """
        根据识别位姿设计红色按钮动作
        """
        print(f"红色按钮动作开始。")
        target_pose1=[target_pose[0],target_pose[1]+0.05,target_pose[2]+0.02,target_pose[3],target_pose[4],target_pose[5]]
        target_pose2=[target_pose[0],target_pose[1]-0.025,target_pose[2]+0.02,target_pose[3],target_pose[4],target_pose[5]]
        target_pose3=[target_pose[0],target_pose[1]+0.05,target_pose[2]+0.02,target_pose[3],target_pose[4],target_pose[5]]
        self.move_to_target_sideway_with_splitting(target_pose1, v, r, block)
        print("已到达目标点前方。")
        #闭合夹爪
        self.control_gripper("close", speed, force, block)
        print("夹爪已闭合。")
        self.move_to_target_sideway_with_splitting(target_pose2, v, r, block)
        print("按钮成功按下。")
        time.sleep(0.5)
        self.move_to_target_sideway_with_splitting(target_pose3, v, r, block)
        print("操作已完成")
        # print("操作已完成")
    
    def control_black_button(self, target_pose, v=10, r=0,speed=500,force=300, block=True):
        """
        根据识别位姿设计黑色旋钮动作
        """
        print(f"黑色旋钮动作开始。")
        target_pose1=[target_pose[0],target_pose[1]+0.05,target_pose[2]+0.02,target_pose[3],target_pose[4],target_pose[5]]
        target_pose2=[target_pose[0],target_pose[1]-0.014,target_pose[2]+0.02,target_pose[3],target_pose[4],target_pose[5]]
        self.move_to_target_sideway_with_splitting(target_pose1, v, r, block)
        print("已到达目标点前方。")
        #张开夹爪
        self.control_gripper("pose",position=400 ,speed=speed, force=force, block=block)    
        self.move_to_target_sideway_with_splitting(target_pose2, v, r, block)
        time.sleep(0.5)
        #闭合夹爪
        self.control_gripper("close", speed, force, block)
        #旋转第6关节
        #获取当前关节角度
        _,joint_state = self.robot.Get_Joint_Degree()
        print(f"当前关节角度为{joint_state}")
        joint_state_left = [joint_state[0],joint_state[1],joint_state[2],joint_state[3],joint_state[4],joint_state[5]+45]
        joint_state_right = [joint_state[0],joint_state[1],joint_state[2],joint_state[3],joint_state[4],joint_state[5]-45]
        joint_init = [joint_state[0],joint_state[1],joint_state[2],joint_state[3],joint_state[4],joint_state[5]]
        self.movej_cmd(joint_state_left, v, block)
        time.sleep(0.5)
        self.movej_cmd(joint_state_right, v, block)
        time.sleep(0.5)
        self.movej_cmd(joint_init, v, block)
        #松夹爪
        time.sleep(0.5)
        self.control_gripper(action="pose", position=500)
        self.move_to_target_sideway_with_splitting(target_pose1, v, r, block)
        print("操作已完成")

    def control_Red_oggle_switch(self, target_pose, action="close",v=10, r=0,speed=500,force=300, block=True):   
        """
        控制红色拨片开关
        """
        #到拨片开关前方
        if action == "close":
            target_pose12=[target_pose[0],target_pose[1]+0.05,target_pose[2]-0.003,target_pose[3],target_pose[4],target_pose[5]]
            target_pose11=[target_pose[0],target_pose[1]+0.005,target_pose[2]-0.003,target_pose[3],target_pose[4],target_pose[5]]
            target_pose0=[target_pose[0],target_pose[1]+0.05,target_pose[2]-0.003,target_pose[3],target_pose[4],target_pose[5]]
            target_pose1=[target_pose[0]+0.02,target_pose[1]-0.01,target_pose[2]-0.025,target_pose[3],target_pose[4],target_pose[5]]
            target_pose2=[target_pose[0]+0.02,target_pose[1]-0.01,target_pose[2]+0.015,target_pose[3],target_pose[4],target_pose[5]]
            target_pose3=[target_pose[0]+0.002,target_pose[1]-0.01,target_pose[2]+0.015,target_pose[3],target_pose[4],target_pose[5]]
            target_pose4=[target_pose[0]+0.02,target_pose[1],target_pose[2]+0.015,target_pose[3],target_pose[4],target_pose[5]]
            target_pose5=[target_pose[0]+0.02,target_pose[1]+0.04,target_pose[2]+0.015,target_pose[3],target_pose[4],target_pose[5]]
            target_pose6=[target_pose[0],target_pose[1]+0.05,target_pose[2]+0.032,target_pose[3],target_pose[4],target_pose[5]]
            target_pose7=[target_pose[0],target_pose[1]+0.01,target_pose[2]+0.032,target_pose[3],target_pose[4],target_pose[5]]
            
            self.move_to_target_sideway_with_splitting(target_pose12, v, r, block)
            print("已到达target12。")
            self.control_gripper(action="pose", position=250)
            print("夹爪张开。")
            self.move_to_target_sideway_with_splitting(target_pose11, v, r, block)
            print("已到达target11。")
            self.control_gripper(action="pose", position=100)
            print("夹爪闭合。")
             #关节六正向旋转180度
            _,joint_state0 = self.robot.Get_Joint_Degree()
            print(f"当前关节角度为{joint_state0}")
            joint_angle0 = [joint_state0[0],joint_state0[1],joint_state0[2],joint_state0[3],joint_state0[4],joint_state0[5]+80]
            self.movej_cmd(joint_angle0, v, block)
            self.control_gripper(action="pose", position=250)
            arm_state0 = self.get_current_arm_state(); ee_pos0 = arm_state0[2]
            target_pose13=[ee_pos0[0],ee_pos0[1]+0.04,ee_pos0[2],ee_pos0[3],ee_pos0[4],ee_pos0[5]]
            #机械臂后移10cm
            self.move_to_target_sideway_with_splitting(target_pose13, v, r, block)
            _,joint_state1 = self.robot.Get_Joint_Degree()
            print(f"当前关节角度为{joint_state1}")
            joint_angle1 = [joint_state1[0],joint_state1[1],joint_state1[2],joint_state1[3],joint_state1[4],joint_state1[5]-80]
            self.movej_cmd(joint_angle1, v, block)


            self.move_to_target_sideway_with_splitting(target_pose0, v, r, block)
            
            #闭合夹爪
            self.control_gripper("close", speed, force, block)
            self.move_to_target_sideway_with_splitting(target_pose1, v, r, block)
            print("已到达target1。")
            #机械臂上移两厘米
            self.move_to_target_sideway_with_splitting(target_pose2, v, r, block)
            print("已到达target2。")
            #机械臂右移1厘米
            self.move_to_target_sideway_with_splitting(target_pose3, v, r, block)
            print("已到达target3。")
            #机械臂上移1厘米
            self.move_to_target_sideway_with_splitting(target_pose4, v, r, block)
            print("已到达target4。")
            #机械臂右移1厘米
            self.move_to_target_sideway_with_splitting(target_pose5, v, r, block)
            print("已到达target5。")
            #机械臂上移1厘米
            self.move_to_target_sideway_with_splitting(target_pose6, v, r, block)
            print("已到达target6。")
            self.control_gripper("open", speed, force, block)
            print("夹爪打开。")
            #机械臂右移1厘米
            self.move_to_target_sideway_with_splitting(target_pose7, v, r, block)
            print("已到达target7。")
            #打开夹爪
            self.control_gripper("close", speed, force, block)
            #关节六正向旋转180度
            _,joint_state2 = self.robot.Get_Joint_Degree()
            print(f"当前关节角度为{joint_state2}")
            joint_angle2 = [joint_state2[0],joint_state2[1],joint_state2[2],joint_state2[3],joint_state2[4],joint_state2[5]+180]
            self.movej_cmd(joint_angle2, v, block)
            self.control_gripper("open", speed, force, block)
            arm_state2 = self.get_current_arm_state(); ee_pos2 = arm_state2[2]
            target_pose3=[ee_pos2[0],ee_pos2[1]+0.04,ee_pos2[2],ee_pos2[3],ee_pos2[4],ee_pos2[5]]
            #机械臂后移10cm
            self.move_to_target_sideway_with_splitting(target_pose3, v, r, block)
            _,joint_state1 = self.robot.Get_Joint_Degree()
            print(f"当前关节角度为{joint_state1}")
            joint_angle1 = [joint_state1[0],joint_state1[1],joint_state1[2],joint_state1[3],joint_state1[4],joint_state1[5]-180]
            self.movej_cmd(joint_angle1, v, block)

            
        if action == "open":
            target_pose1=[target_pose[0],target_pose[1]+0.05,target_pose[2]+0.032,target_pose[3],target_pose[4],target_pose[5]]
            target_pose2=[target_pose[0],target_pose[1]+0.01,target_pose[2]+0.032,target_pose[3],target_pose[4],target_pose[5]]
            

            #闭合夹爪
            self.control_gripper("open", speed, force, block)
            #首先，去到螺帽位置，扭旋钮
            self.move_to_target_sideway_with_splitting(target_pose1, v, r, block)
            print("已到达目标点附近。")
            #打开夹爪
            self.control_gripper("open", speed, force, block)
            #机械臂前移10cm
            self.move_to_target_sideway_with_splitting(target_pose2, v, r, block)
            #闭合夹爪
            self.control_gripper("close", speed, force, block)
            #机械臂逆时针旋转180度
            _,joint_state = self.robot.Get_Joint_Degree()
            print(f"当前关节角度为{joint_state}")
            joint_angle = [joint_state[0],joint_state[1],joint_state[2],joint_state[3],joint_state[4],joint_state[5]-180]
            #这里做这个赋值是因为joint——state为7关节，我们只要前6个数据
            joint_angle1 = [joint_state[0],joint_state[1],joint_state[2],joint_state[3],joint_state[4],joint_state[5]]
            self.movej_cmd(joint_angle, v, block)
            print("旋钮已拧开。")
            #打开夹爪
            self.control_gripper("open", speed, force, block)
            arm_state = self.get_current_arm_state(); ee_pos = arm_state[2]
            target_pose3=[ee_pos[0],ee_pos[1]+0.04,ee_pos[2],ee_pos[3],ee_pos[4],ee_pos[5]]
            #机械臂后移10cm
            self.move_to_target_sideway_with_splitting(target_pose3, v, r, block)
            _,joint_state1 = self.robot.Get_Joint_Degree()
            print(f"当前关节角度为{joint_state}")
            joint_angle1 = [joint_state1[0],joint_state1[1],joint_state1[2],joint_state1[3],joint_state1[4],joint_state1[5]+180]
            self.movej_cmd(joint_angle1, v, block)
            #self.move_to_target_sideway_with_splitting(target_pose4, v, r, block)
            #关闭夹爪
            self.control_gripper("close", speed, force, block)
            #目标点正前方并右移2cm后退10cm
            target_pose4=[target_pose[0]-0.02,target_pose[1]+0.03,target_pose[2]+0.015,target_pose[3],target_pose[4],target_pose[5]]
            target_pose5=[target_pose[0]-0.02,target_pose[1],target_pose[2]+0.015,target_pose[3],target_pose[4],target_pose[5]]
            target_pose6=[target_pose[0]+0.03,target_pose[1],target_pose[2]+0.015,target_pose[3],target_pose[4],target_pose[5]]
            target_pose6=[target_pose[0]+0.01,target_pose[1]+0.05,target_pose[2]+0.015,target_pose[3],target_pose[4],target_pose[5]]
            self.move_to_target_sideway_with_splitting(target_pose4, v, r, block)
            time.sleep(0.5)
            #机械臂前移10cm 
            self.move_to_target_sideway_with_splitting(target_pose5, v, r, block)
            time.sleep(0.5)
            #机械臂左移4cm
            self.move_to_target_sideway_with_splitting(target_pose6, v, r, block)
            time.sleep(0.5)
            # print("拨片开关已打开。")
            # #机械臂后移10cm
            # self.move_to_target_sideway_with_splitting(target_pose8, v, r, block)





            
            #机械臂到达目标点正前方，后退10cm，上移3厘米 定位到螺帽正上方并打开夹爪
            #机械臂前移10cm，到达螺帽位置并合闭夹爪
            #机械臂关节6逆时针旋转180度
            #打开夹爪
            #机械臂后移10cm
            #关闭夹爪
            #去到目标点正前方，右移2cm，后退10cm
            #机械臂前移10cm
            #机械臂左移4cm
            #机械臂后移10cm


    def get_current_arm_state(self):
        '''
        该函数用于获取机械臂当前状态
        参数
        返回值 成功返回：(0, joint, pose, Arm_Err, Sys_Err)
        joint：关节 1~7 角度数组
        pose：机械臂当前位姿[x,y,z,rx,ry,rz]
        Arm_Err：机械臂运行错误代码
        Sys_Err：控制器错误代码。
        失败返回：错误码，查询 API 错误类型。
        '''
        tag = self.robot.Get_Current_Arm_State()  # 修改为 self.robot
        
        print(f'get_current_arm_state:{tag}')
        return tag

    def set_hand_seq(self, seq_num, block=1):
        """
        设置灵巧手目标动作序列
        """
        tag = self.robot.Set_Hand_Seq(seq_num, block)  # 修改为 self.robot
        print(f'set_hand_seq:{tag}')
        return tag

    def gripper_init(self, min_limit=0, max_limit=1000, block=True):
        """初始化夹爪"""
        self.robot.Set_Gripper_Route(min_limit)
        print(f"夹爪初始化成功。最大开合度为：{max_limit}，最小开合度为：{min_limit}")
    
    def control_gripper(self, action, speed=500, force=300, position=1000, block=True, timeout=10):
        """控制夹爪"""
        if action == "open":
            self.robot.Set_Gripper_Release(speed, block, timeout)
            print("夹爪已松开。")
        elif action == "close":
            # 确保传递一个浮动类型的数值
            self.robot.Set_Gripper_Pick(speed, force, block, timeout)
            print(f"夹爪已夹取，力度为 {force}。")
        elif action == "pose":
            self.robot.Set_Gripper_Position(position, block, timeout)
        else:
            print("无效的夹爪动作。")

    def control_gripper_dh(self, action, speed=50, force=50, position=0, block=True, timeout=10,port=1,device=1):
        """控制大寰夹爪"""
        self.write_Single_Register(port,0x0101,force,device,block)    # 设置力值50%
        self.write_Single_Register(port,0x0104,speed,device,block)    # 设置速度30%
        if action == "open":
            self.write_Single_Register(port,0x0103,1000,device,block)    # 打开夹爪
            print("大寰夹爪已打开。")
        elif action == "close":
            self.write_Single_Register(port,0x0103,0,device,block)    # 关闭夹爪
            print("大寰夹爪已关闭。")
        elif action == "pose":
            self.write_Single_Register(port,0x0103,position,device,block)    # 设置夹爪位置
            print(f"大寰夹爪已设置到 {position} 位置。")
        else:
            print("无效的夹爪动作。")
    
    def control_lift(self, height=1000, lim_high=1240, lim_low=-1380, speed=50, block=True):
        """控制升降机"""
        if height > lim_high or height < lim_low:
            print("升降机高度超出范围。")
            return
        self.robot.Set_Lift_Height(height, speed, block)
        print(f"升降机已设置到 {height} 毫米高度。")
    def get_io_out_state(self):
        """获取IO状态"""
        tag,DO_state,AO_voltage = self.robot.Get_IO_Output()  # 修改为 self.robot
        print(f"DO_state:{DO_state},AO_voltage:{AO_voltage}")
        return tag,DO_state,AO_voltage
    def get_io_in_state(self):
        """获取IO状态"""
        tag,DO_state,AO_voltage = self.robot.Get_IO_Input()  # 修改为 self.robot
        print(f"DO_state:{DO_state},AO_voltage:{AO_voltage}")
        return tag,DO_state,AO_voltage
    
    def set_Modbus_Mode (self,port,baudrate,timeout,block):
        """设置Modbus模式"""
        tag = self.robot.Set_Modbus_Mode(port,baudrate,timeout,block)  # 修改为 self.robot
        #print(f"set_Modbus_Mode:{tag}")
        return tag

    def close_Modbus_Mode (self,port , block):
        """关闭Modbus模式"""
        tag = self.robot.Close_Modbus_Mode(port,block)  # 修改为 self.robot
        #print(f"close_Modbus_Mode:{tag}")
        return tag
    def set_Tool_Voltage(self,type, block):
        """设置工具电压"""
        tag = self.robot.Set_Tool_Voltage(type, block)  # 修改为 self.robot
        #print(f"set_Tool_Voltage:{tag}")
        return tag  
    def write_Single_Register(self,port,address,data,device,block):
        """写单个寄存器"""
        tag = self.robot.Write_Single_Register(port,address,data,device,block)  # 修改为 self.robot
        #print(f"write_Single_Register:{tag}")
        return tag
    
    def get_Read_Holding_Registers(self,port,address,device):
        """读保持寄存器"""
        tag,data = self.robot.Get_Read_Holding_Registers(port,address,device)  # 修改为 self.robot
        #print(f"get_Read_Holding_Registers:{tag},data:{data}")
        return tag,data

    def shutdown(self):
        """关闭机械臂连接"""
        self.robot.RM_API_UnInit()
        self.robot.Arm_Socket_Close()
        
        print("机械臂连接已关闭。")

def main():
    """主函数，用于控制机械臂。"""
    # 初始化机械臂连接
    
    init_joint = [84, 54, 121, 174, 86, 90]
    pos_init = [0.18, -0.54, 0.02, 1.57, -1.57, 0]
    arm = RobotArmController(ip_address="192.168.10.18", model=RM65)
    #切换工作坐标系
    #arm.change_work_frame(name="World")
    print("切换到World坐标系")
    #time.sleep(2)
    #切换工具坐标系
    #arm.change_tool_frame(name="grip")
    #print("切换到grip坐标系")
    
    #time.sleep(2)
    #arm.init_arm( pos_init,init_joint, v=35, r=0,  speed_gripper=500, height_lift=0, speed_lift=30, block=True,timeout=30)
    # arm.set_Tool_Voltage(3, block=True)
    # # 初始化夹爪（此时才启动Modbus通讯）
    # arm.set_Modbus_Mode(port=1, baudrate=115200, timeout=3, block=True)
    # #关闭io模式
    # arm.write_Single_Register(1, 0x0402, 0, 1, True)
    # print("关闭io")
    # # 完全初始化（重新标定最大最小行程）
    # arm.write_Single_Register(1, 0x0100, 0xA5, 1, True)
    # # print("完全初始化行程")
    # time.sleep(10)
    # arm.control_gripper_dh("close", speed=50, force=50,block=True, timeout=10,port=1,device=1)
    # time.sleep(2)
    # arm.control_gripper_dh("open", speed=50, force=50,block=True, timeout=10,port=1,device=1)
    # time.sleep(2)
    arm.control_gripper_dh("pose", speed=50, force=50, position=100, block=True, timeout=10,port=1,device=1)
    #arm.set_Tool_Voltage(0, block=True)
    arm.shutdown()

if __name__ == "__main__":
    main()
