import cv2
import numpy as np
import cairosvg  # 用于将 SVG 转换为 PNG

def convert_svg_to_png(svg_path, png_path, dpi=300):
    """
    将 SVG 文件转换为 PNG 格式。

    :param svg_path: 输入 SVG 文件路径
    :param png_path: 输出 PNG 文件路径
    :param dpi: 打印分辨率，默认 300 DPI
    """
    try:
        cairosvg.svg2png(url=svg_path, write_to=png_path, dpi=dpi)
        print(f"SVG 文件已成功转换为 PNG：{png_path}")
    except Exception as e:
        print(f"无法转换 SVG 文件：{e}")
        return None
    return png_path


def place_aruco_on_a4(input_image_path, output_file, dpi=300):
    """
    将现有的 ArUco 图像居中放置在 A4 纸中央，并生成高分辨率的输出图像。

    :param input_image_path: 输入的 ArUco 图像路径
    :param output_file: 输出图像路径（保存为 PNG 或其他格式）
    :param dpi: 打印分辨率，默认 300 DPI
    """
    # A4 纸尺寸（以像素为单位）
    a4_width_mm = 210  # A4 纸宽度（毫米）
    a4_height_mm = 297  # A4 纸高度（毫米）
    a4_width_px = int(a4_width_mm / 25.4 * dpi)  # 转换为像素
    a4_height_px = int(a4_height_mm / 25.4 * dpi)

    # 创建空白 A4 图像（白色背景）
    a4_image = np.ones((a4_height_px, a4_width_px, 3), dtype=np.uint8) * 255

    # 读取输入图像（PNG 格式）
    aruco_image = cv2.imread(input_image_path)

    if aruco_image is None:
        print(f"错误：无法加载图像 {input_image_path}")
        return

    # 获取输入图像尺寸
    aruco_height, aruco_width = aruco_image.shape[:2]

    # 检查输入图像是否比 A4 纸还大
    if aruco_width > a4_width_px or aruco_height > a4_height_px:
        print("错误：输入图像尺寸大于 A4 纸，无法放置。")
        return

    # 计算 ArUco 图像在 A4 图像中的位置
    top_left_x = (a4_width_px - aruco_width) // 2
    top_left_y = (a4_height_px - aruco_height) // 2
    bottom_right_x = top_left_x + aruco_width
    bottom_right_y = top_left_y + aruco_height

    # 将 ArUco 图像绘制到 A4 图像中央
    a4_image[top_left_y:bottom_right_y, top_left_x:bottom_right_x] = aruco_image

    # 保存最终的 A4 图像
    cv2.imwrite(output_file, a4_image)

    print(f"已生成居中的 ArUco 图像并保存到 {output_file}")

    # 显示结果（可选）
    cv2.imshow("ArUco on A4", a4_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# 参数设置
svg_path = "/home/quintinumi/project/ws_lcr_cal/src/lcr_cal/src/demo/6x6_1000-12.svg"  # 输入的 SVG 文件路径
temp_png_path = "/home/quintinumi/project/ws_lcr_cal/src/lcr_cal/src/demo/temp_converted.png"  # 临时 PNG 文件路径
output_file = "/home/quintinumi/project/ws_lcr_cal/src/lcr_cal/src/demo/id_150__dictName_10_A4.png"  # 最终输出文件路径
dpi = 300  # 打印分辨率（DPI）

# 1. 将 SVG 转换为 PNG
converted_png_path = convert_svg_to_png(svg_path, temp_png_path, dpi)

# 2. 将 PNG 图像放置在 A4 中央
if converted_png_path:
    place_aruco_on_a4(converted_png_path, output_file, dpi)