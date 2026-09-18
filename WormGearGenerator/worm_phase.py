"""
worm_phase.py - 蜗杆装配变换（相位角 + 三轴基底）

相位是一个数，基底由旋转矩阵直接算出（不手抄分量，避免抄错符号）。

几何要求
--------
啮合点在蜗轮中心与蜗杆轴心的连线上：
    蜗轮中心 = 原点，蜗杆轴心 = (a, 0, 0)  ->  啮合方向（从蜗杆轴心看）= -X
蜗杆建模时在 z=0 截面，成型齿槽刀具沿局部 +X (θ=0) 切削，
因此局部 +X 是齿槽中心线，实心螺纹齿顶中心位于 θ = 180°/z1 + k*360°/z1。

装配矩阵 R 必须同时满足：
    实心齿顶中心线        -> 全局 (-1, 0, 0)   即 -X，朝向蜗轮齿槽
    局部 Z（蜗杆自身轴线）-> 全局 ( 0, 1, 0)   即 +Y，与蜗轮轴线(Z)正交
    局部 Y                -> 由右手系自动确定

基础装配姿态 R_base = R_X(-90°)，逐列展开即得三个基底向量：
    ax（局部 X -> 全局）= ( 1,  0,  0)   齿槽指向 +X (背离蜗轮)
    ay（局部 Y -> 全局）= ( 0,  0, -1)
    az（局部 Z -> 全局）= ( 0,  1,  0)   蜗杆轴线与蜗轮轴线正交
    在此姿态下，实心齿顶（局部 -X，即 θ=180°）天然对准全局 -X (朝向蜗轮)！

对多头蜗杆，绕全局 +Y 轴（蜗杆自身轴线）进行相位微调：
    R = R_Y(psi) · R_base
无论相位 psi 取何值，az 恒等于 (0, 1, 0)，绝不随自转倾斜或翻转。
行列式恒为 +1，三列单位正交 —— 是真旋转，不是反射。
"""

import math

# 相位角 ψ（度）：单头/奇数头蜗杆在基准姿态下实心齿顶已正对全局 -X，基准偏转角为 0°
PHASE_DEG = 0.0
PHASE_RAD = 0.0


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _rot_x(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def _rot_y(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def worm_phase_deg(z1=1):
    """装配相位 ψ (度)。

    根据头数 z1 计算使实心齿顶精确对准啮合中心线 (-X) 的自转相位角。
    蜗杆在 z=0 截面，刀具沿局部 +X (θ=0) 切削齿槽，
    因此实心螺纹齿顶位于 θ_tooth = 180°/z1 + k*360°/z1。
    在基准姿态 R_base = R_X(-90°) 下，局部 θ=180° (局部 -X) 精确对准全局 -X。
    对于奇数头 (z1=1, 3)：局部 180° 即为实心齿，psi = 0°；
    对于偶数头 (z1=2, 4, 6)：旋转 psi = (180° - θ_tooth) % 360° 使最近的实心齿顶对准全局 -X。
    """
    if z1 is None or z1 <= 1:
        return PHASE_DEG
    k = (z1 - 1) // 2
    phi_tooth = (180.0 / z1) + k * (360.0 / z1)
    return (180.0 - phi_tooth) % 360.0


def worm_phase_rad(z1=1):
    """装配相位 ψ (弧度)。"""
    return math.radians(worm_phase_deg(z1))


def worm_matrix(psi_deg=None, z1=1):
    """返回装配旋转矩阵 R = R_Y(psi) · R_X(-90°)。

    列的含义：第 k 列 = 局部第 k 轴在全局下的方向。
    局部 Z 轴（蜗杆自身轴线）恒对准全局 +Y 轴，不随 psi 产生倾斜或颠倒。
    实心齿顶在全局下精确对准全局 -X 轴（朝向蜗轮齿槽）。
    """
    if psi_deg is None:
        psi_deg = worm_phase_deg(z1)
    elif 0.0 < abs(psi_deg) <= 2 * math.pi + 1e-6:
        # 容错：若误传了弧度值（如 3.14159...），自动转换为角度度数
        psi_deg = math.degrees(psi_deg)

    r_base = _rot_x(-90.0)
    return _matmul(_rot_y(psi_deg), r_base)


def worm_basis(psi_deg=None, z1=1):
    """返回 (ax, ay, az)：局部 X / Y / Z 三轴在全局下的方向。

    直接取自 R 的三个列，不手写分量。
    """
    R = worm_matrix(psi_deg, z1)
    ax = (R[0][0], R[1][0], R[2][0])
    ay = (R[0][1], R[1][1], R[2][1])
    az = (R[0][2], R[1][2], R[2][2])
    return ax, ay, az


def describes_phase(z1=1):
    """给界面用的简短说明。"""
    return "蜗杆装配相位 ψ = {:.1f}°".format(worm_phase_deg(z1))

