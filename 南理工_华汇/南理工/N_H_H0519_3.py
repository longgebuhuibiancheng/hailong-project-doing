# -*- coding: utf-8 -*-
from paddleocr import PaddleOCR
import cv2
import time
import numpy as np
import rm_control0507 as rm
import D435_rgb_depth as rs
import convert as cam_to_base

# 初始化 OCR 实例
ocr = PaddleOCR(use_angle_cls=True, use_gpu=False, lang='en')
print("OCR 初始化完成")

# 初始化 RealSense 和机械臂
realsense_processor = rs.RealSenseProcessor()
print("RealSense 初始化完成")
#arm = rm.RobotArmController(ip_address="192.168.10.18", model=rm.RM65)
print("机械臂初始化完成")

# 初始姿态
init_joint = [84, 54, 121, 174, 86, 90]
pos_init = [0.18, -0.54, 0.02, 1.57, -1.57, 0]

# —— 第一步：标签与按钮位姿关系 ——  
# 每个标签包含控件类型、以及标签到按钮的偏移（单位：米）
label_info = {
    '1CLP10': {'type': 'black_button',        'offsets': {'x': 0.00, 'y': 0.015, 'z': -0.03}},
    '8CLP1':  {'type': 'red_button',          'offsets': {'x': 0.00, 'y': 0.015, 'z': -0.03}},
    '8LP1':   {'type': 'red_toggle_switch',   'offsets': {'x': 0.00, 'y': 0.015, 'z': -0.03}},
    '1CLP1':  {'type': 'green_button',        'offsets': {'x': 0.00, 'y': 0.015, 'z': -0.03}},
    '1CLP8':  {'type': 'black_button',        'offsets': {'x': 0.00, 'y': 0.015, 'z': -0.04}},
}

# --------- 辅助函数：打印 OCR 结果并显示 ---------
def ocr_print_and_show(image, results, target_label):
    while isinstance(results, list) and len(results) == 1:
        results = results[0]

    # 只绘制目标标签的文字框
    for i, item in enumerate(results, start=1):
        box, (text, conf) = item
        


        box_str = ', '.join(f"({int(x)},{int(y)})" for x, y in box)
        print(f"第{i}条识别结果:")
        print(f"  文字框坐标: [{box_str}]")
        print(f"  识别内容: {text}")
        print(f"  置信度: {conf:.4f}\n")
        
        # 将文本框转换为矩形框并绘制
        pts = np.array(box, dtype=np.int32)
        pts = pts.reshape((-1, 1, 2))
        # 绘制轮廓（多边形框）
        cv2.polylines(image, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        # 在文字框上添加文字
        cv2.putText(image, text, (int(box[0][0]), int(box[0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # 显示图像，并立即返回，不阻塞程序
    cv2.imshow("OCR Results", image)
    cv2.waitKey(1000)  # 等待1毫秒，立即返回，不会阻塞程序
    cv2.destroyAllWindows()  # 关闭窗口

# -------- 显示并过滤 OCR 结果 ---------
def show_ocr_results(image, target_label, conf_threshold=0.98):
    if isinstance(image, str):
        img = cv2.imread(image)
    else:
        img = image
    if img is None:
        raise ValueError("无法读取输入图像")
    
    # 执行 OCR
    results = ocr.ocr(img, cls=True)
    
    if not results:
        print("未检测到文字区域")
        return []
    
    filtered = []
    for line in results:
        if not line:
            continue
        items = line if isinstance(line[0][0], list) else [line]
        for item in items:
            _, (_, conf) = item
            if conf >= conf_threshold:
                filtered.append(item)
    
    if not filtered:
        print("未检测到符合置信度的文字结果")
        return []
    
    # 只绘制与目标标签匹配的框
    ocr_print_and_show(img, filtered, target_label)  # 修改这里，传入目标标签
    
    return filtered

# --------- 核心函数：提取目标标签像素位置与类型 ---------
def get_target_pose(image, ocr_target, conf_threshold=0.98):
    image_copy = image.copy() 
    filtered = show_ocr_results(image, ocr_target, conf_threshold)
    for box, (text, _) in filtered:
        if text.strip() == ocr_target:
            xs = [pt[0] for pt in box]; ys = [pt[1] for pt in box]
            x_center = int(sum(xs)/len(xs)); y_center = int(sum(ys)/len(ys))
            info = label_info.get(ocr_target)
            if info is None:                
                return (x_center, y_center), None, None 
            cv2.circle(image_copy, (x_center, y_center), radius=3, color=(0, 0, 255), thickness=-1)
            cv2.imshow("u_v_center", image_copy)
            cv2.waitKey(2000)  # 等待1毫秒，立即返回，不会阻塞程序
            cv2.destroyAllWindows()  # 关闭窗口      
            return (x_center, y_center), info['type'], info['offsets']
    print(f"未找到标签 '{ocr_target}'")
    return (None, None), None, None

# --------- 任务：OCR 检测并转换坐标 ---------
def ocr_detection(ocr_target, camera_index=0, timeout=5):
    intr, depth_intrin, color_image, depth_image, aligned = realsense_processor.get_aligned_images()
    (u, v), btn_type, offsets = get_target_pose(color_image, ocr_target)
    if u is None:
        return None, None, None  # 返回None，避免解包错误
    depth_pixel = [u, v]
    avg_depth, _, _, X_cam, Y_cam, Z_cam = realsense_processor.process_frame(depth_pixel)
    X_cam = X_cam + offsets.get('x', 0.0)
    Y_cam = Y_cam + offsets.get('y', 0.0)
    Z_cam = Z_cam + offsets.get('z', 0.0)
    ee_pos = [0.18, -0.54, 0.02, 1.57, -1.57, 0]  # 机械臂初始位置
    #arm_state = arm.get_current_arm_state(); ee_pos = arm_state[2]
    X_base, Y_base, Z_base = cam_to_base.convert(X_cam, Y_cam, Z_cam, *ee_pos)
    
    label_pose = [X_base, Y_base, Z_base, ee_pos[3], ee_pos[4], ee_pos[5]]
    return label_pose, btn_type, offsets

# --------- 新增函数：调整按钮位姿 ---------
def adjust_button_pose(label_pose, offsets):
    X, Y, Z, Rx, Ry, Rz = label_pose
    X=X+offsets.get('x', 0.0);Y=Y+offsets.get('y', 0.0);Z=Z+offsets.get('z', 0.0)
    print(f"调整后的位姿: X = {X:.3f}, Y = {Y:.3}, Z = {Z:.3f}, Rx = {Rx:.4f}, Ry = {Ry:.4f}, Rz = {Rz:.4f}")
    return [
        X,
        Y,
        Z,
        Rx, Ry, Rz
    ]   


# --------- 动作分发函数 ---------
def perform_button_action(label_pose, btn_type, offsets,action_type="open", v=10, r=0, speed=500, force=300, block=True):
    button_pose = adjust_button_pose(label_pose, offsets)
    dispatcher = {
        'green_button': arm.control_green_button,
        'red_button': arm.control_red_button,
        'black_button': arm.control_black_button,
        'red_toggle_switch': arm.control_Red_oggle_switch,
    }
    action = dispatcher.get(btn_type)
    if not action:
        print(f"未找到类型为 '{btn_type}' 的动作函数")
        return
    #action(button_pose, v=v, r=r, speed=speed, force=force, block=block)
    if btn_type == "red_toggle_switch" :
        action(button_pose,action=action_type, v=v, r=r, speed=speed, force=force, block=block)
    else:
        action(button_pose, v=v, r=r, speed=speed, force=force, block=block)


def ocr_control_arm(target_label, action_type="open", v=10, r=0, speed=500, force=300, block=True):
    label_pose, btn_type, offsets = ocr_detection(target_label)
    if btn_type is None:
        print("检测失败或未检测到标签")
        return
    print(f"标签位姿: {label_pose}, 类型: {btn_type}, 偏移: {offsets}")
    
    perform_button_action(label_pose, btn_type, offsets, action_type=action_type ,v=10, r=0, speed=500, force=300, block=True)
    #arm.movej_cmd(init_joint,v=10,r=0, block=True)
    #arm.movej_p_cmd(pose=pos_init, v=10, r=0, block=True)

# --------- 主流程示例 ---------
def main():
    try:
        #arm.change_tool_frame(name="grip")
        #arm.change_work_frame(name="World")
        #arm.movej_cmd(init_joint,v=10,r=0, block=True)
        #arm.init_arm(pose=pos_init, v=10, r=0, speed_gripper=500, height_lift=650, speed_lift=30, block=True)
        time.sleep(2)
        ocr_control_arm("1CLP10", v=10, r=0, speed=500, force=300, block=True)
        time.sleep(2)
        ocr_control_arm("8CLP1",  v=10, r=0, speed=500, force=300, block=True)
        time.sleep(2)
        ocr_control_arm("8LP1", action_type="open", v=10, r=0, speed=500, force=300, block=True)
        time.sleep(2)
        ocr_control_arm("8LP1", action_type="close", v=10, r=0, speed=500, force=300, block=True)
        time.sleep(2)
        ocr_control_arm("1CLP1",  v=10, r=0, speed=500, force=300, block=True)
        time.sleep(2)
        ocr_control_arm("1CLP8",  v=10, r=0, speed=500, force=300, block=True)
    except KeyboardInterrupt:
        print("检测到Ctrl+C，急停机械臂并退出程序！")
        #tar=arm.move_stop(block=True)  # 调用急停
        print(f"急停成功，返回值：{tar}")
        # 可以做一些清理操作
        exit(0)

if __name__ == "__main__":
    main()
