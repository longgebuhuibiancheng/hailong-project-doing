# -*- coding: utf-8 -*-
from paddleocr import PaddleOCR
import cv2
import numpy as np
import rm_control0507 as rm

# 初始化 OCR 实例
ocr = PaddleOCR(use_angle_cls=True, use_gpu=False, lang='en')

# —— 第一步：标签 → 类型 ——  
label_to_type = {
    '1CLP10': 'black_button',
    '8CLP1':  'red_button',
    '8LP1':   'red_toggle_switch',
    '1CLP1':  'green_button',
    '1CLP8':  'black_button',
}

# --------- 辅助函数：打印 OCR 结果 ---------
def ocr_print(result):
    while isinstance(result, list) and len(result) == 1:
        result = result[0]
    for i, item in enumerate(result, start=1):
        box = item[0]
        text, conf = item[1]
        box_str = ', '.join(f"({int(x)},{int(y)})" for x, y in box)
        print(f"第{i}条识别结果:")
        print(f"  文字框坐标: [{box_str}]")
        print(f"  识别内容: {text}")
        print(f"  置信度: {conf:.4f}\n")

# -------- 显示并过滤 OCR 结果 ---------
def show_ocr_results(image_path, conf_threshold=0.98):
    results = ocr.ocr(image_path, cls=True)
    filtered = []
    for line in results:
        # 单图或多图均返回(item)列表
        for item in (line if isinstance(line[0][0], list) else [line]):
            if item[1][1] >= conf_threshold:
                filtered.append(item)
    ocr_print(filtered)
    return filtered

# --------- 核心函数：从图片中提取目标标签位置和类型 ---------
def get_target_pose(image, ocr_target):
    """
    从 OCR 结果中提取目标位置
    输入:
        image (str): 本地图片路径
        ocr_target (str): 屏幕上唯一的文字标签内容
    返回:
        position (tuple): 文本框中心点坐标 (x, y)
        btn_type (str): 该标签对应的控件类型
        若未找到，返回 (None, None)
    """
    filtered = show_ocr_results(image)
    for box, (text, conf) in filtered:
        if text.strip() == ocr_target:
            xs = [pt[0] for pt in box]
            ys = [pt[1] for pt in box]
            x_center = sum(xs) / len(xs)
            y_center = sum(ys) / len(ys)
            return (x_center, y_center), label_to_type.get(ocr_target)
    print(f"未找到标签 '{ocr_target}'")
    return None, None

# -------- 测试示例 ---------
if __name__ == "__main__":
    image_path1 = 'C:/Users/hailong/Desktop/1920_Color.png'
    pose, btn_type = get_target_pose(image_path1, '8LP12')
    print('定位结果:', pose, btn_type)
