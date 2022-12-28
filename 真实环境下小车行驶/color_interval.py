import numpy as np
# 白色区域检测
white_lower = np.array([90, 0, 0])
white_upper = np.array([255, 213, 233])

#黑色区间
black_lower = np.array([50, 15, 0])
black_upper = np.array([255 ,255 ,106])

# 红色区间
red_lower = np.array([0, 43, 46])
red_upper = np.array([10, 255, 255])

# #绿色区间
green_lower = np.array([35, 43, 46])
green_upper = np.array([77, 255, 255])

# #蓝色区间
blue_lower=np.array([100, 43, 46])
blue_upper = np.array([124, 255, 255])

# #黄色区间
yellow_lower = np.array([26, 43, 46])
yellow_upper = np.array([34, 255, 255])

# #橙色区间
orange_lower = np.array([11, 43, 46])
orange_upper = np.array([25, 255, 255])

color_dict={"red":[red_lower,red_upper],"green":[green_lower,green_upper],
            "blue":[blue_lower,blue_upper],"yellow":[yellow_lower,yellow_upper],
            "black":[black_lower,black_upper],"white":[white_lower,white_upper]}