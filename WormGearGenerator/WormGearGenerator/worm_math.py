"""
worm_math.py - 蜗轮蜗杆核心几何与力学参数计算模块
遵循国家标准 GB/T 10085-2018、GB/T 10087-2018 与 ISO/TS 14521。
"""

import math
from typing import Dict, Any, Tuple, List, Optional


class WormGearMath:
    """
    圆柱蜗杆与蜗轮传动几何与力学精确计算类。
    支持 ZA (阿基米德) 和 ZN (法向直廓) 齿形，支持右旋与左旋。
    """

    # GB/T 10085-2018 标准模数系列 (mm)
    MODULE_SERIES_1 = [1.0, 1.25, 1.6, 2.0, 2.5, 3.15, 4.0, 5.0, 6.3, 8.0, 10.0, 12.5, 16.0, 20.0, 25.0]
    MODULE_SERIES_2 = [1.5, 1.75, 2.24, 2.8, 3.5, 4.5, 5.5, 7.0, 9.0, 11.2, 14.0, 18.0, 22.4, 28.0]

    # GB/T 10085-2018 标准蜗杆直径系数 q 系列
    Q_SERIES = [6.3, 7.1, 8.0, 9.0, 10.0, 11.2, 12.5, 14.0, 16.0, 18.0, 20.0, 22.4, 25.0]

    # GB/T 10085-2018 优先中心距系列 a (mm)
    A_SERIES_1 = [40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500]
    A_SERIES_2 = [45, 56, 71, 90, 112, 140, 180, 224, 280, 355, 450]

    def __init__(
        self,
        m: float,
        z1: int,
        z2: int,
        q: float,
        x2: Optional[float] = None,
        a: Optional[float] = None,
        alpha_deg: float = 20.0,
        worm_type: str = "ZA",
        direction: str = "Right",
        ha_star: float = 1.0,
        c_star: float = 0.2,
        b1: Optional[float] = None,
        b2: Optional[float] = None,
        backlash: Optional[float] = None,
        wheel_chamfer: Optional[float] = None,
        worm_fillet: Optional[float] = None,
        n1: float = 1450.0,
    ):
        """
        :param m: 模数 (mm) (轴向模数 mx = m)
        :param z1: 蜗杆头数 (1, 2, 3, 4, 6)
        :param z2: 蜗轮齿数 (通常 >= 17, 建议 >= 28)
        :param q: 蜗杆直径系数 (q = d1 / m)
        :param x2: 蜗轮变位系数 (若给出目标中心距 a 且 x2=None，则自动反推)
        :param a: 目标中心距 (mm) (若未给出，按 x2 计算)
        :param alpha_deg: 基本压力角 (通常 20.0 度)
        :param worm_type: 齿廓类型 ('ZA' 阿基米德 或 'ZN' 法向直廓)
        :param direction: 旋向 ('Right' 右旋 或 'Left' 左旋)
        :param ha_star: 齿顶高系数 (标准取 1.0)
        :param c_star: 顶隙系数 (标准取 0.2)
        :param b1: 蜗杆螺纹部分长度 (mm) (若为 None 按 GB 标准自动推荐)
        :param b2: 蜗轮齿宽 (mm) (若为 None 按 GB 标准自动推荐)
        :param backlash: 法向齿侧间隙 jn (mm) (默认 0.0 mm)
        :param wheel_chamfer: 蜗轮边倒斜角 (mm) (若为 None 默认单边为总高度 b2 的 5%)
        :param worm_fillet: 蜗杆齿顶倒角半径 (mm) (默认 0.0 mm)
        :param n1: 蜗杆输入转速 (RPM, 用于动力学滑动速度与自锁校核)
        """
        self.m = float(m)
        self.z1 = int(z1)
        self.z2 = int(z2)
        self.q = float(q)
        self.alpha_deg = float(alpha_deg)
        self.worm_type = worm_type.upper() if worm_type else "ZA"
        self.direction = "Left" if "left" in str(direction).lower() else "Right"
        self.ha_star = float(ha_star)
        self.c_star = float(c_star)
        self.n1 = float(n1)

        # 变位系数 x2 与实际中心距 a 的互相换算
        if a is not None and x2 is None:
            self.a_target = float(a)
            self.x2 = (self.a_target / self.m) - 0.5 * (self.q + self.z2)
        elif x2 is not None:
            self.x2 = float(x2)
            self.a_target = 0.5 * self.m * (self.q + self.z2 + 2.0 * self.x2)
        else:
            self.x2 = 0.0
            self.a_target = 0.5 * self.m * (self.q + self.z2)

        self.custom_b1 = b1
        self.custom_b2 = b2
        self.custom_backlash = backlash
        self.custom_wheel_chamfer = wheel_chamfer
        self.custom_worm_fillet = worm_fillet

        # 入口防护：非法输入（非有限值、z1<1、m<=0）必须在这里就拦住。
        # 否则 _calculate() 内的 z2/z1、math.ceil(NaN) 等会直接抛
        # ZeroDivisionError / ValueError，用户看到的是崩溃堆栈而不是可读的校核提示。
        self._input_errors = self._check_inputs()
        self._backlash_ok = True   # 由 _calculate 按实际侧隙刷新
        self.results = self._calculate() if not self._input_errors else {}

    def _check_inputs(self) -> List[str]:
        """在进入几何计算之前拦截会让计算本身崩溃的非法输入。"""
        errors = []
        numeric = {
            "模数 m": self.m,
            "蜗杆头数 z1": self.z1,
            "蜗轮齿数 z2": self.z2,
            "直径系数 q": self.q,
            "压力角 α": self.alpha_deg,
            "变位系数 x2": self.x2,
            "中心距 a": self.a_target,
            "齿顶高系数 ha*": self.ha_star,
            "顶隙系数 c*": self.c_star,
        }
        bad = [
            name for name, val in numeric.items()
            if not isinstance(val, (int, float)) or not math.isfinite(float(val))
        ]
        if bad:
            errors.append("参数 " + "、".join(bad) + " 不是有限数值，无法计算。")
            return errors

        if self.m <= 0.0:
            errors.append(f"模数 m 必须为正数 (当前: {self.m})。")
        if self.z1 < 1:
            errors.append(f"蜗杆头数 z1 必须 >= 1 (当前: {self.z1})。")
        if self.z2 < 1:
            errors.append(f"蜗轮齿数 z2 必须 >= 1 (当前: {self.z2})。")
        if self.q <= 0.0:
            errors.append(f"直径系数 q 必须为正数 (当前: {self.q})。")
        for label, val in (("b1", self.custom_b1), ("b2", self.custom_b2),
                           ("backlash", self.custom_backlash),
                           ("wheel_chamfer", self.custom_wheel_chamfer),
                           ("worm_fillet", self.custom_worm_fillet)):
            if val is not None:
                try:
                    if not math.isfinite(float(val)):
                        errors.append(f"参数 {label} 不是有限数值 (当前: {val})。")
                except (TypeError, ValueError):
                    errors.append(f"参数 {label} 不是有效数值 (当前: {val})。")
        return errors

    def _calculate(self) -> Dict[str, Any]:
        m = self.m
        z1 = self.z1
        z2 = self.z2
        q = self.q
        x2 = self.x2
        ha_star = self.ha_star
        c_star = self.c_star

        # 1. 运动与节距参数
        i = z2 / z1
        px = math.pi * m          # 轴向齿距
        pz = math.pi * m * z1     # 导程 (节距)

        # 导程角 gamma
        tan_gamma = z1 / q
        gamma_rad = math.atan(tan_gamma)
        gamma_deg = math.degrees(gamma_rad)
        cos_gamma = math.cos(gamma_rad)
        sin_gamma = math.sin(gamma_rad)

        pn = px * cos_gamma       # 法向齿距
        mn = m * cos_gamma        # 法向模数

        # 2. 压力角关系 (ZA: 轴向标准20°; ZN: 法向标准20°)
        alpha_input_rad = math.radians(self.alpha_deg)
        if self.worm_type == "ZA":
            alpha_x_rad = alpha_input_rad
            alpha_x_deg = self.alpha_deg
            tan_alpha_n = math.tan(alpha_x_rad) * cos_gamma
            alpha_n_rad = math.atan(tan_alpha_n)
            alpha_n_deg = math.degrees(alpha_n_rad)
        else:
            # ZN
            alpha_n_rad = alpha_input_rad
            alpha_n_deg = self.alpha_deg
            tan_alpha_x = math.tan(alpha_n_rad) / cos_gamma
            alpha_x_rad = math.atan(tan_alpha_x)
            alpha_x_deg = math.degrees(alpha_x_rad)

        # 3. 蜗杆 (Worm) 尺寸
        d1 = q * m                                      # 分度圆直径
        ha1 = ha_star * m                               # 齿顶高
        hf1 = (ha_star + c_star) * m                    # 齿根高
        h1 = ha1 + hf1                                  # 全齿高
        da1 = d1 + 2.0 * ha1                            # 齿顶圆直径 = m * (q + 2)
        df1 = d1 - 2.0 * hf1                            # 齿根圆直径 = m * (q - 2.4)
        c = c_star * m                                  # 顶隙
        da0 = da1 + 2.0 * c                             # 切齿刀具顶圆直径 (蜗杆顶圆 + 顶隙补偿)

        # 4. 蜗轮 (Worm Wheel) 尺寸
        d2 = z2 * m                                     # 分度圆直径
        a = 0.5 * m * (q + z2 + 2.0 * x2)              # 实际中心距

        ha2 = (ha_star + x2) * m                        # 齿顶高
        hf2 = (ha_star + c_star - x2) * m               # 齿根高
        h2 = ha2 + hf2                                  # 全齿高

        # 变位后蜗轮齿顶与蜗杆齿根之间的实际顶隙余量。
        # 标准化设计满足 ha2 + c == hf1（即余量 == 0）；x2 != 0 时两者不再相等，
        # 余量 < 0 表示蜗轮齿顶与蜗杆齿根发生干涉，必须提示用户。
        clearance_margin = hf1 - ha2 - c

        da2 = d2 + 2.0 * ha2                            # 喉圆直径 (中间平面齿顶圆直径)
        df2 = d2 - 2.0 * hf2                            # 齿根圆直径
        rg2 = a - 0.5 * da2                             # 蜗轮喉母圆半径 = 0.5 * m * (q - 2)
        rf2 = a - 0.5 * df2                             # 蜗轮齿根圆弧半径 = 0.5 * da0

        # 蜗轮最大外圆直径 de2
        if z1 <= 3:
            de2_max = da2 + 1.5 * m
        else:
            de2_max = da2 + 1.0 * m
        de2 = round(de2_max, 2)

        # 蜗轮喉部啮合在蜗杆轴向的空间包络弦长
        r_worm_tip = da1 / 2.0
        r_wheel_tip = de2 / 2.0
        y_dist = a - r_worm_tip
        if r_wheel_tip > y_dist:
            throat_chord = 2.0 * math.sqrt(r_wheel_tip**2 - y_dist**2)
        else:
            throat_chord = 0.0

        # 蜗杆螺纹最小有效长度 b1_min (GB/T 10085 仅为最小有效啮合段)
        if z1 <= 2:
            b1_min = (11.0 + 0.06 * z2) * m
        elif z1 == 3:
            b1_min = (12.5 + 0.09 * z2) * m
        else:
            b1_min = (13.0 + 0.10 * z2) * m

        # 为保证蜗轮在进刀与出刀全过程均与完整蜗杆螺纹配合，避免端面截断与尖角切入：
        # 蜗杆螺纹长度需完全覆盖喉部啮合弦长，并留出进出刀各约 0.75~1.0 个轴向齿距余量与倒角
        b1_rec = max(math.ceil(b1_min), math.ceil(throat_chord + 1.5 * px), math.ceil(18.0 * m))
        b1 = float(self.custom_b1) if self.custom_b1 is not None else b1_rec

        # 齿厚与齿槽宽
        sx0 = 0.5 * math.pi * m                         # 标称轴向齿厚
        ex0 = 0.5 * math.pi * m                         # 标称轴向齿槽宽
        sn0 = sx0 * cos_gamma                           # 标称法向齿厚
        en0 = ex0 * cos_gamma                           # 标称法向齿槽宽

        # 蜗轮齿宽 b2
        if z1 <= 2:
            b2_max = 0.75 * da1
        elif z1 == 3:
            b2_max = 0.70 * da1
        else:
            b2_max = 0.67 * da1

        b2_rec = 0.65 * da1
        b2 = float(self.custom_b2) if self.custom_b2 is not None else round(b2_rec, 1)

        # 喉角 wrap angle 2*delta
        sin_delta = min(1.0, max(0.0, (b2 / 2.0) / rg2 if rg2 > 1e-4 else 0.5))
        delta_deg = math.degrees(math.asin(sin_delta))
        wrap_angle_2delta_deg = 2.0 * delta_deg

        # 5. 制造与工艺特征 (齿侧间隙、蜗轮倒斜角、蜗杆倒圆角)
        if self.custom_backlash is not None:
            jn = max(0.0, float(self.custom_backlash))
        else:
            jn = 0.0

        cos_alpha_n = math.cos(alpha_n_rad)
        jx = (jn / (cos_alpha_n * cos_gamma)) if (cos_alpha_n * cos_gamma) > 1e-6 else 0.0
        sx_eff = sx0 - jx
        # 侧隙不得吃掉全部齿厚，否则齿廓自交
        self._backlash_ok = sx_eff > 0.05 * sx0
        # 蜗轮倒斜角 (默认单边为总高度 b2 的 5%)
        if self.custom_wheel_chamfer is not None:
            wheel_chamfer_c = max(0.0, float(self.custom_wheel_chamfer))
        else:
            wheel_chamfer_c = round(0.05 * b2, 2)

        # 蜗杆齿顶倒角 (默认 0.0 mm)
        if self.custom_worm_fillet is not None:
            worm_fillet_r = max(0.0, float(self.custom_worm_fillet))
        else:
            worm_fillet_r = 0.0

        # 6. 滑动速度、摩擦系数与自锁校核 (ISO/TS 14521)
        v1 = (math.pi * d1 * self.n1) / 60000.0         # 蜗杆圆周速度 (m/s)
        vs = v1 / cos_gamma                             # 相对啮合滑动速度 (m/s)
        n2 = self.n1 / i                                # 蜗轮转速 (RPM)
        v2 = (math.pi * d2 * n2) / 60000.0              # 蜗轮圆周速度 (m/s)

        # 静摩擦系数与动摩擦系数 (ISO/TS 14521 / 《机械设计手册》)
        # 静态启动摩擦系数 (青铜/钢在边界油膜或静止状态下通常为 0.10 ~ 0.15，取典型值 0.115)
        mu_static = 0.115
        rho_static_rad = math.atan(mu_static / cos_alpha_n)
        rho_static_deg = math.degrees(rho_static_rad)

        # 动态运转摩擦系数 (随相对滑动速度衰减)
        mu_z = 0.02 + 0.06 / (1.0 + 0.8 * (vs ** 0.7))
        rho_prime_rad = math.atan(mu_z / cos_alpha_n)
        rho_prime_deg = math.degrees(rho_prime_rad)

        # 正向传动效率 (蜗杆主动)
        denom = math.tan(gamma_rad + rho_prime_rad)
        eta_mesh = (math.tan(gamma_rad) / denom) if denom > 1e-6 else 0.0
        eta_mesh = max(0.0, min(0.98, eta_mesh))

        # 逆向效率与自锁判定 (依据《机械设计手册》第3卷 蜗杆传动及 ISO/TR 14521 标准)
        # 1. 完全自锁 (强抗振): 导程角 γ ≤ 3.5° (无论静止或剧烈振动冲击均可靠自锁)
        # 2. 静态自锁 (静止不可逆): 3.5° < γ ≤ 6.5° 或 γ ≤ ρ'_static
        #    (单头蜗杆典型区间，静止状态下完全无法反向驱动；但在强烈持续振动下可能产生微量滑移)
        # 3. 临界自锁 (受扰动易滑移): 6.5° < γ ≤ 8.5° (静止有阻滞，受扰动易逆转，逆向效率极低)
        # 4. 可逆转 (双向传动): γ > 8.5° (通常为多头蜗杆 z1 ≥ 2，逆向传动良好)
        denom_rev = math.tan(gamma_rad)
        if gamma_deg <= 3.5:
            lock_status = f"完全自锁 (强抗振, γ={gamma_deg:.2f}° ≤ 3.5°)"
            self_locking = True
            eta_reverse = 0.0
        elif gamma_deg <= max(6.5, rho_static_deg) or (z1 == 1 and gamma_deg <= 7.0):
            lock_status = f"静态自锁 (静止不可逆, γ={gamma_deg:.2f}°)"
            self_locking = True
            eta_reverse = 0.0
        elif gamma_deg <= 8.5:
            lock_status = f"临界自锁 (受扰动易滑移, γ={gamma_deg:.2f}°)"
            self_locking = False
            eta_reverse = max(0.0, math.tan(gamma_rad - rho_prime_rad) / denom_rev) if denom_rev > 1e-6 else 0.0
        else:
            lock_status = f"可逆转 (双向传动, γ={gamma_deg:.2f}°)"
            self_locking = False
            eta_reverse = max(0.0, math.tan(gamma_rad - rho_prime_rad) / denom_rev) if denom_rev > 1e-6 else 0.0

        return {
            # 基础运动参数
            "module_m": round(m, 4),
            "normal_module_mn": round(mn, 4),
            "starts_z1": z1,
            "teeth_z2": z2,
            "ratio_i": round(i, 4),
            "diameter_factor_q": round(q, 4),
            "profile_shift_x2": round(x2, 4),
            "center_distance_a": round(a, 4),
            "worm_type": self.worm_type,
            "direction": self.direction,

            # 角度与齿距
            "lead_angle_gamma_deg": round(gamma_deg, 4),
            "lead_angle_gamma_rad": round(gamma_rad, 6),
            "axial_pitch_px": round(px, 4),
            "lead_pz": round(pz, 4),
            "normal_pitch_pn": round(pn, 4),
            "pressure_angle_normal_deg": round(alpha_n_deg, 4),
            "pressure_angle_axial_deg": round(alpha_x_deg, 4),

            # 齿形参数
            "ha_star": ha_star,
            "c_star": c_star,
            "clearance_c": round(c, 4),
            "addendum_ha1": round(ha1, 4),
            "dedendum_hf1": round(hf1, 4),
            "tooth_depth_h1": round(h1, 4),
            "addendum_ha2": round(ha2, 4),
            "dedendum_hf2": round(hf2, 4),
            "tooth_depth_h2": round(h2, 4),

            # 蜗杆尺寸 (mm)
            "worm_pitch_diameter_d1": round(d1, 4),
            "worm_tip_diameter_da1": round(da1, 4),
            "worm_root_diameter_df1": round(df1, 4),
            "hob_tip_diameter_da0": round(da0, 4),
            "worm_length_b1": round(b1, 2),
            "worm_length_b1_min": round(b1_min, 2),
            "worm_length_b1_rec": round(b1_rec, 2),
            "worm_nominal_thickness_sx": round(sx0, 4),
            "worm_effective_thickness_sx": round(sx_eff, 4),
            "worm_normal_space_en": round(en0, 4),

            # 蜗轮尺寸 (mm)
            "wheel_pitch_diameter_d2": round(d2, 4),
            "wheel_throat_diameter_da2": round(da2, 4),
            "wheel_tip_diameter_de2": round(de2, 4),
            "wheel_root_diameter_df2": round(df2, 4),
            "wheel_throat_radius_rg2": round(rg2, 4),
            "wheel_root_arc_radius_rf2": round(rf2, 4),
            "wheel_throat_chord": round(throat_chord, 2),
            "wheel_face_width_b2": round(b2, 2),
            "wheel_face_width_b2_max": round(b2_max, 2),
            "wheel_wrap_angle_2delta_deg": round(wrap_angle_2delta_deg, 2),
            "tip_clearance_margin": round(clearance_margin, 4),

            # 制造与工艺特征
            "normal_backlash_jn": round(jn, 4),
            "axial_backlash_jx": round(jx, 4),
            "wheel_chamfer_c": round(wheel_chamfer_c, 3),
            "worm_fillet_r": round(worm_fillet_r, 3),

            # 力学与自锁
            "sliding_velocity_vs_ms": round(vs, 4),
            "wheel_linear_velocity_v2_ms": round(v2, 4),
            "friction_angle_rho_deg": round(rho_prime_deg, 4),
            "mesh_efficiency_eta": round(eta_mesh, 4),
            "is_self_locking": self_locking,
            "self_locking_status": lock_status,
        }

    def validate(self) -> Tuple[bool, List[str], List[str]]:
        """校核参数合理性，返回 (is_valid, errors, warnings)"""
        errors = []
        warnings = []
        res = self.results

        # ---- 0. 入口防护：非法输入在构造阶段已被拦截，结果表为空 ----
        if self._input_errors:
            return False, list(self._input_errors), []

        # ---- 1. 基本参数 ----
        if self.m <= 0:
            errors.append(f"模数 m 必须为正数 (当前: {self.m})。")
        elif not self._in_series(self.m, self.MODULE_SERIES_1 + self.MODULE_SERIES_2):
            warnings.append(f"模数 m = {self.m} 非 GB/T 10085 标准模数。")

        if self.z1 < 1:
            errors.append(f"蜗杆头数 z1 = {self.z1} 必须 >= 1。")
        elif self.z1 > 6:
            errors.append(f"蜗杆头数 z1 = {self.z1} > 6，超出插件支持范围 (1~6)。")
        elif self.z1 not in [1, 2, 3, 4, 6]:
            warnings.append(f"蜗杆头数 z1 = {self.z1} 较少见，标准建议为 1、2、4、6。")

        if self.z2 < 17:
            errors.append(f"蜗轮齿数 z2 = {self.z2} < 17，将发生严重齿面根切！")
        elif self.z2 < 28:
            warnings.append(f"蜗轮齿数 z2 = {self.z2} < 28，建议增加正变位 x2 以消除根切。")

        if self.q <= 2.4:
            errors.append(f"直径系数 q = {self.q} <= 2.4 导致蜗杆齿根圆直径 <= 0！")
        elif not self._in_series(self.q, self.Q_SERIES):
            warnings.append(f"直径系数 q = {self.q} 非 GB/T 10085 标准推荐值。")

        if abs(self.x2) > 1.0:
            warnings.append(f"变位系数 |x2| = {abs(self.x2):.2f} > 1.0，超出常用工程范围 [-0.5, +0.5]。")

        # ---- 2. 轴向长度与齿宽的上下限 (旧版本完全遗漏了 b1/b2 的下限与工艺特征校核) ----
        if res["worm_length_b1"] <= 0.0:
            errors.append(
                f"蜗杆螺纹长度 b1 = {res['worm_length_b1']} mm 必须为正数 "
                f"(推荐值 {res['worm_length_b1_rec']} mm)。"
            )
        if res["wheel_face_width_b2"] <= 0.0:
            errors.append(
                f"蜗轮齿宽 b2 = {res['wheel_face_width_b2']} mm 必须为正数。"
            )

        if res["worm_length_b1"] > 0.0 and res["worm_length_b1"] < res["wheel_throat_chord"]:
            errors.append(
                f"蜗杆长度 b1 = {res['worm_length_b1']} mm 小于喉部啮合弦长 "
                f"{res['wheel_throat_chord']} mm，端部啮合将被截断！"
                f"建议 b1 ≥ {res['worm_length_b1_rec']} mm。"
            )

        if res["wheel_face_width_b2"] > res["wheel_face_width_b2_max"]:
            errors.append(
                f"蜗轮齿宽 b2 = {res['wheel_face_width_b2']} mm 超过标准上限 "
                f"{res['wheel_face_width_b2_max']} mm，齿顶将过薄。"
            )

        # ---- 3. 工艺特征校核 ----
        if res["wheel_chamfer_c"] >= res["wheel_face_width_b2"] / 2.0:
            errors.append(
                f"蜗轮倒斜角 {res['wheel_chamfer_c']} mm 必须小于齿宽的一半 "
                f"({res['wheel_face_width_b2'] / 2.0:.2f} mm)。"
            )

        if res["normal_backlash_jn"] > 0.0 and not self._backlash_ok:
            errors.append(
                f"法向侧隙 jn = {res['normal_backlash_jn']} mm 过大，"
                f"轴向侧隙 jx = {res['axial_backlash_jx']} mm 已吃掉标称齿厚 "
                f"{res['worm_nominal_thickness_sx']} mm 的绝大部分，齿廓将自交。"
            )

        if res["worm_fillet_r"] > 0.0:
            fillet_max = 0.35 * self.m
            if res["worm_fillet_r"] > fillet_max:
                warnings.append(
                    f"蜗杆齿顶倒角 {res['worm_fillet_r']} mm 超过安全上限 "
                    f"{fillet_max:.3f} mm (0.35m)，实际将按 {fillet_max:.3f} mm 施加。"
                )

        # ---- 4. 变位带来的顶隙变化 ----
        # 变位后 ha2 + c 不再等于 hf1。正值变位会让蜗轮齿顶侵入蜗杆齿根区，
        # 但正变位本身是常用的设计手段（凑配中心距 / 消除根切），
        # 因此这里给出明确警告而不是直接阻断生成。
        margin = res["tip_clearance_margin"]
        if margin < -0.05 * max(abs(res["clearance_c"]), 1e-6):
            warnings.append(
                f"变位系数 x2 = {self.x2:+.3f} 为正值，蜗轮齿顶相对蜗杆齿根少了 "
                f"{abs(margin):.4f} mm 顶隙（标准顶隙 c = {res['clearance_c']} mm）。"
                f"若装配后出现干涉，请减小 x2 或改用「指定中心距」模式重新配凑。"
            )
        elif abs(margin) > 1e-6:
            warnings.append(
                f"变位系数 x2 = {self.x2:+.3f} 使实际顶隙相对标准值偏差 "
                f"{margin:+.4f} mm（偏大属安全余量）。"
            )

        # ---- 5. 根切校核 ----
        alpha_x_rad = math.radians(res["pressure_angle_axial_deg"])
        x2_min = 1.0 - (self.z2 * (math.sin(alpha_x_rad) ** 2)) / 2.0
        if self.x2 < x2_min - 1e-4:
            warnings.append(f"x2 = {self.x2:.2f} 小于防根切极限 {x2_min:.2f}。")

        # ---- 6. 变位过大导致齿根变尖 ----
        if res["dedendum_hf2"] <= 0.0:
            errors.append(
                f"变位系数 x2 = {self.x2:+.3f} 过大，蜗轮齿根高 hf2 = "
                f"{res['dedendum_hf2']} mm <= 0。"
            )

        return len(errors) == 0, errors, warnings

    @staticmethod
    def _in_series(value: float, series: List[float], tol: float = 1e-6) -> bool:
        """按容差判断是否属于标准系列 (旧实现用精确浮点相等，输入框的浮点误差会误报)。"""
        return any(abs(value - s) <= tol * max(1.0, abs(s)) for s in series)

    def to_dict(self) -> Dict[str, Any]:
        return self.results

    def summary_html(self) -> str:
        """生成供 Fusion 360 UI 弹窗展示的紧凑 HTML 摘要"""
        res = self.results
        valid, errors, warnings = self.validate()

        # 输入非法时 results 为空，直接给出可读的错误列表，避免 KeyError
        if not res:
            html = "<font color='red'><b>参数错误:</b></font><br>"
            html += "".join(f"<font color='red'>· {e}</font><br>" for e in errors)
            return html

        lock_color = "#0066cc" if res["is_self_locking"] else "#333333"

        html = f"""
        <table style="width:100%; font-size:12px; border-collapse:collapse; line-height:1.5;">
            <tr><td style="padding:2px 4px;"><b>传动比 i:</b> {res['ratio_i']} ({res['teeth_z2']}/{res['starts_z1']})</td><td style="padding:2px 4px;"><b>中心距 a:</b> {res['center_distance_a']} mm</td></tr>
            <tr><td style="padding:2px 4px;"><b>导程角 γ:</b> {res['lead_angle_gamma_deg']}°</td><td style="padding:2px 4px;"><b>变位系数 x2:</b> {res['profile_shift_x2']}</td></tr>
            <tr><td style="padding:2px 4px;"><b>蜗杆顶圆 da1:</b> {res['worm_tip_diameter_da1']} mm</td><td style="padding:2px 4px;"><b>蜗轮喉径 da2:</b> {res['wheel_throat_diameter_da2']} mm</td></tr>
            <tr><td style="padding:2px 4px;"><b>蜗杆分度圆 d1:</b> {res['worm_pitch_diameter_d1']} mm</td><td style="padding:2px 4px;"><b>蜗轮分度圆 d2:</b> {res['wheel_pitch_diameter_d2']} mm</td></tr>
            <tr><td style="padding:2px 4px;"><b>法向模数 mn:</b> {res['normal_module_mn']} mm</td><td style="padding:2px 4px;"><b>导程 pz:</b> {res['lead_pz']} mm</td></tr>
            <tr><td style="padding:2px 4px;"><b>齿侧间隙 jn:</b> {res['normal_backlash_jn']} mm</td><td style="padding:2px 4px;"><b>蜗轮倒斜角:</b> {res['wheel_chamfer_c']} mm</td></tr>
            <tr><td style="padding:2px 4px;"><b>蜗杆齿顶倒角:</b> {res['worm_fillet_r']} mm</td><td style="padding:2px 4px;"><b>理论传动效率:</b> {res['mesh_efficiency_eta']*100:.1f}%</td></tr>
            <tr><td style="padding:2px 4px;" colspan="2"><b>自锁性能:</b> <font color="{lock_color}">{res['self_locking_status']}</font></td></tr>
        </table>
        """
        if errors:
            html += f"<br><font color='red'><b>错误:</b> {'; '.join(errors)}</font>"
        elif warnings:
            html += f"<br><font color='#cc8800'><b>提示:</b> {'; '.join(warnings)}</font>"

        return html
