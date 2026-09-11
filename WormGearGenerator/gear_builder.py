"""
gear_builder.py - Fusion 360 蜗轮 (Worm Wheel) 实体建模模块

采用**多截面共轭齿廓放样法 (Conjugate Section Lofting)**：
  1. 回转生成纯实心喉圆轮坯；
  2. 沿齿宽取 K 个平行截面，在每个截面上按理论啮合条件求解蜗轮齿槽的
     左 / 右齿廓点，连成封闭截面轮廓；
  3. 实体放样 (Solid Loft) 生成单个齿槽切削体；
  4. 将该切削体注入设计后用临时 BRep 管理器全周阵列差集，一次性切除 z2 个齿槽。

本模块不再包含任何"虚拟滚刀 / 展成包络"分支，全部几何均由上述放样方案产生。
"""

import math
import adsk.core
import adsk.fusion

# 展成搜索角的全域范围 (弧度)。真实啮合接触角对于任何工程可用参数都不会超出该范围，
# 区间取得足够宽是为了保证「材料边界必然落在区间内部」，从而杜绝静默夹断到边界的旧缺陷。
_GEN_LIMIT = math.radians(60.0)

# 全域粗扫描步长 (弧度)，用于定位材料边界所在区间，随后再二分细化。
_GEN_SCAN_STEP = math.radians(2.0)

# 黄金分割比
_GOLDEN = 0.6180339887498949

# 齿廓点使用 6 个光滑内插半径 (不含齿顶收口点)
_N_FLANK = 6


def _throat_arc_end(a: float, rg: float, re2: float, b2: float) -> float:
    """轮坯喉部圆弧的终止半宽 (cm)。

    喉母圆 (半径 rg，圆心位于蜗杆轴线上) 与两端半径 re2 的外圆柱面交于
    |zk| = sqrt(rg^2 - (a - re2)^2)。圆弧到此为止，之后由外圆柱面接管。
    若该交点超过半个齿宽，则圆弧在端面之前就被截断，需按齿宽限制。
    这里与 build_worm_wheel 共用同一套计算，确保放样轮廓与轮坯外缘永不脱节。
    """
    term = rg * rg - (a - re2) ** 2
    z_int = math.sqrt(term) if term > 1e-9 else 0.0
    return min(z_int, (b2 / 2.0) * 0.98)


def _blank_outer_radius(a: float, rg: float, re2: float, zk: float, z_arc_end: float) -> float:
    """给定轴向截面坐标 zk，返回蜗轮轮坯在该截面上的外半径 (cm)。

    与实际回转轮廓一致，且在喉部圆弧段内关于 |zk| 单调不减：
      - |zk| <= z_arc_end：喉部圆弧段，外半径 = a - sqrt(rg^2 - zk^2)
        （在圆弧真实交点处该值正好等于 re2；受齿宽截断时则小于 re2）
      - |zk| >  z_arc_end：两端外圆柱段，外半径 = re2

    旧实现用 |zk| < rg 作为分界（并在该处强行取 min 到 re2），与实际轮廓不符，
    会在圆弧末端附近给出偏小的外半径，使放样轮廓越过轮坯真实边界。
    """
    if rg > 1e-9 and abs(zk) <= z_arc_end:
        inner = rg * rg - zk * zk
        if inner > 0.0:
            return max(0.0, min(re2, a - math.sqrt(inner)))
    return re2


def _build_geom(params: dict, scale: float = 0.1) -> dict:
    """把 worm_math 的参数表换算为建模用的 cm 制几何常量。"""
    m = params["module_m"] * scale
    q = params["diameter_factor_q"]
    a = params["center_distance_a"] * scale
    direction = params.get("direction", "Right")
    px = params["axial_pitch_px"] * scale
    jx = params.get("axial_backlash_jx", 0.0) * scale

    return {
        "m": m,
        "z1": params["starts_z1"],
        "z2": params["teeth_z2"],
        "a": a,
        "r1": (m * q) / 2.0,
        "r2": (m * params["teeth_z2"]) / 2.0,
        "ra0": params["hob_tip_diameter_da0"] * scale / 2.0,
        "rf1": params["worm_root_diameter_df1"] * scale / 2.0,
        "ra2": params["wheel_throat_diameter_da2"] * scale / 2.0,
        "re2": params["wheel_tip_diameter_de2"] * scale / 2.0,
        "rg": params["wheel_throat_radius_rg2"] * scale,
        "b2": params["wheel_face_width_b2"] * scale,
        "tan_a": math.tan(math.radians(params["pressure_angle_axial_deg"])),
        "sx": (px / 2.0) + (jx / 2.0),
        "lead": params["lead_pz"] * scale,
        "lead_sign": 1.0 if direction == "Right" else -1.0,
    }


def _solve_flank_point(zk: float, R: float, right: bool, geom: dict) -> float:
    """求解单个齿廓点，返回该点在截面内的极角 theta (弧度)。

    原理：对任意给定的极角 theta，在可行的展成转角区间内取「间隙函数」的最大值
    max_g(theta)。max_g < 0 表示该点会被刀具切掉，max_g >= 0 表示该点属于保留的
    轮齿材料；因此 max_g 由负变正的那一点即为理论共轭齿廓。

    与旧实现的关键差别：
      * 搜索区间取 ±60° 全域，保证真实零点一定落在区间内部；
      * 使用「全域粗扫描 + 二分细化」代替裸二分，避免因初值区间不含零点而
        静默夹断到区间边界（那会产出退化甚至非单调的假齿廓）；
      * 若确实找不到材料边界，显式抛错而不是返回一个虚假的角度。
    """
    a = geom["a"]
    r1 = geom["r1"]
    r2 = geom["r2"]
    rf1 = geom["rf1"]
    ra0 = geom["ra0"]
    tan_a = geom["tan_a"]
    sx = geom["sx"]
    lead = geom["lead"]
    lead_sign = geom["lead_sign"]
    zk2 = zk * zk
    r_root_exact = geom["r_root_exact"]

    ratio = min(1.0, r_root_exact / R)
    d_phi_max = math.acos(ratio)
    if d_phi_max <= 1e-9:
        raise RuntimeError(
            "齿廓求解失败：截面 zk={:.4f}mm、半径 R={:.4f}mm 的展成区间退化"
            "（齿根起始半径必须大于该截面的喉底半径）。".format(zk * 10.0, R * 10.0)
        )

    def gap(th, pv):
        """统一「材料侧」为正的刀具间隙。

        刀具齿槽在蜗杆随动坐标系中的轴向范围为 [axial - w_r, axial + w_r]：
          - 右齿面：保留材料位于 Y_w >= axial - w_r
          - 左齿面：保留材料位于 Y_w <= axial + w_r
        取符号因子 s = +1 / -1，使两侧可共用同一套零点搜索代码。
        """
        s = 1.0 if right else -1.0
        ang = th + pv
        X_w = R * math.cos(ang)
        Y_w = R * math.sin(ang)
        r_w = math.sqrt((a - X_w) ** 2 + zk2)
        r_c = max(rf1, min(ra0, r_w))
        w_r = (sx / 2.0) - (r_c - r1) * tan_a
        sin_p = max(-0.999, min(0.999, zk / r_c))
        axial = r2 * pv + lead_sign * lead * math.asin(sin_p) / (2.0 * math.pi)
        return s * (Y_w - axial) + w_r

    # 齿廓 = 保留材料区的边界。统一间隙函数 gap 的正负号在两侧相反：
    #   右齿面：材料位于 Y_w >= axial - w_r  →  gap <= 0 侧
    #   左齿面：材料位于 Y_w <= axial + w_r  →  gap >= 0 侧
    # 两侧沿 theta 增大方向都是「先材料、后切除」，故扫描方向一致，
    # 仅材料判据 inside(mg) 不同。
    if right:
        def inside(mg):
            return mg <= 0.0
    else:
        def inside(mg):
            return mg >= 0.0

    def max_g(th):
        """该极角下、在可行展成区间内间隙函数的最大值。"""
        p_lo = -th - d_phi_max
        p_hi = -th + d_phi_max
        if p_lo >= p_hi:
            return -1.0
        c_p = p_hi - _GOLDEN * (p_hi - p_lo)
        d_p = p_lo + _GOLDEN * (p_hi - p_lo)
        f_c = gap(th, c_p)
        f_d = gap(th, d_p)
        for _ in range(24):
            if f_c > f_d:
                p_hi = d_p
                d_p = c_p
                f_d = f_c
                c_p = p_hi - _GOLDEN * (p_hi - p_lo)
                f_c = gap(th, c_p)
            else:
                p_lo = c_p
                c_p = d_p
                f_c = f_d
                d_p = p_lo + _GOLDEN * (p_hi - p_lo)
                f_d = gap(th, d_p)
        return gap(th, 0.5 * (p_lo + p_hi))

    # ---- 全域粗扫描：定位 max_g 穿越「材料/切除」分界的第一个区间 ----
    span = 2.0 * _GEN_LIMIT
    n_steps = max(1, int(span / _GEN_SCAN_STEP))

    t_prev = -_GEN_LIMIT
    f_prev = max_g(t_prev)
    bracket = None
    for k in range(1, n_steps + 1):
        t = -_GEN_LIMIT + span * k / float(n_steps)
        f_t = max_g(t)
        if inside(f_prev) and not inside(f_t):
            bracket = (t_prev, t)
            break
        t_prev = t
        f_prev = f_t

    if bracket is None:
        raise RuntimeError(
            "齿廓求解失败：截面 zk={:.4f}mm、半径 R={:.4f}mm ({}齿面) 在 ±{:.0f}° 范围内"
            "未找到材料边界。该参数组合超出本啮合模型的适用范围，"
            "请检查模数 / 直径系数 / 变位系数 / 齿宽 b2 的组合是否合理。".format(
                zk * 10.0, R * 10.0, "右" if right else "左", math.degrees(_GEN_LIMIT)
            )
        )

    # ---- 二分细化：lo 侧仍为材料，hi 侧已被切除 ----
    lo, hi = bracket
    for _ in range(48):
        mid = 0.5 * (lo + hi)
        if inside(max_g(mid)):
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _compute_section_curves(params: dict, K: int = 7) -> list:
    """
    在 K 个沿齿宽平行的截面上求解单齿槽的理论共轭齿廓点集。

    返回列表，每个元素包含:
      - 'zk': 截面在世界 Z 轴坐标 (单位: cm)
      - 'pts_right': 右齿面点集 [(X, Y), ...] (单位: cm, 从齿根到齿顶)
      - 'pts_left': 左齿面点集 [(X, Y), ...] (单位: cm, 从齿根到齿顶)
    """
    geom = _build_geom(params)
    a = geom["a"]
    m = geom["m"]
    rg = geom["rg"]
    re2 = geom["re2"]
    ra2 = geom["ra2"]
    ra0 = geom["ra0"]
    b2 = geom["b2"]

    if K < 3:
        raise RuntimeError("截面数量 K 必须 >= 3（实体放样至少需要 3 个截面）。")

    # 截面在 Z 方向的覆盖范围：覆盖 105% 齿宽以确保完全切透两端侧面
    z_max = (b2 / 2.0) * 1.05
    # 与实际回转轮廓共用的喉部圆弧终止位置，保证放样轮廓不越过轮坯外缘
    z_arc_end = _throat_arc_end(a, rg, re2, b2)
    sections_data = []

    for idx in range(K):
        zk = -z_max + 2.0 * z_max * idx / float(K - 1)
        zk2 = zk * zk

        # 该截面上轮坯的真实外半径：放样轮廓到此为止即可，
        # 不再向外空切（空切只会产生与轮坯无关的直边，徒增畸变）
        r_blank = _blank_outer_radius(a, rg, re2, zk, z_arc_end)
        r_flank_top = min(re2, max(ra2, r_blank))
        # 切削体顶部高度：高出轮坯最高外缘 re2 约 0.6*m (约 1.2mm 空切区)，
        # 确保两个端面截面也能切透外圆柱段，杜绝齿顶残留鳍片薄膜
        r_cut_top = re2 + 0.6 * m

        # 真实喉底根部半径 (在 zk 截面上)
        r_root_exact = a - math.sqrt(max(0.1, ra0 ** 2 - zk2))
        geom["r_root_exact"] = r_root_exact
        # 极精密根部起始点：留出 0.01*m 贴合理论根径，杜绝奇异截断
        r_start = r_root_exact + 0.01 * m

        if r_flank_top <= r_start + 1e-9:
            raise RuntimeError(
                "蜗轮齿宽 b2 过大或模数过小：截面 zk={:.4f}mm 上轮坯外半径 "
                "{:.4f}mm 已不大于齿根起始半径 {:.4f}mm，无法生成齿廓。"
                "请减小齿宽 b2 或增大模数 m。".format(zk * 10.0, r_blank * 10.0, r_start * 10.0)
            )

        pts_right = []
        pts_left = []

        for i_R in range(_N_FLANK):
            R = r_start + (r_flank_top - r_start) * i_R / float(_N_FLANK - 1)
            th_R = _solve_flank_point(zk, R, True, geom)
            th_L = _solve_flank_point(zk, R, False, geom)
            pts_right.append((R * math.cos(th_R), R * math.sin(th_R)))
            pts_left.append((R * math.cos(th_L), R * math.sin(th_L)))

        # 齿顶收口：沿最后一点的走向延伸到「超出轮坯最高外缘」的高度。
        # 必须超出 re2，而不是只到本截面的轮坯外半径：两端截面的喉部外半径
        # 明显小于外圆柱半径 re2，若只切到该处，外圆柱段会残留一圈未切净的材料。
        th_R_top = math.atan2(pts_right[-1][1], pts_right[-1][0])
        th_L_top = math.atan2(pts_left[-1][1], pts_left[-1][0])
        pts_right.append((r_cut_top * math.cos(th_R_top), r_cut_top * math.sin(th_R_top)))
        pts_left.append((r_cut_top * math.cos(th_L_top), r_cut_top * math.sin(th_L_top)))

        sections_data.append({
            "zk": zk,
            "pts_right": pts_right,
            "pts_left": pts_left,
        })

    return sections_data


def build_worm_wheel(
    component: adsk.fusion.Component,
    params: dict,
    quality: str = "standard"
) -> adsk.fusion.BRepBody:
    """
    在指定组件中生成纯实心蜗轮实体 (无内轴孔，便于后续按需开孔或加工键槽)。

    :param component: Fusion 360 目标组件 (Component)
    :param params: worm_math.to_dict() 输出的参数字典
    :param quality: 精度质量 ('fast': 5截面, 'standard': 7截面, 'fine': 9截面, 'ultra': 11截面)
    :return: 生成的 BRepBody 蜗轮实体对象
    """
    scale = 0.1  # mm 转 cm

    geom = _build_geom(params)
    a = geom["a"]
    b2 = geom["b2"]
    re2 = geom["re2"]
    da2 = geom["ra2"] * 2.0
    rg = geom["rg"]

    features = component.features
    sketches = component.sketches

    # =========================================================================
    # 1. 建立纯实心蜗轮轮坯 (Revolve)
    # 在 XZ 构造平面上绘制外廓，绕中心线回转 360 度 (实心，不打任何轴孔)
    # =========================================================================
    blank_sketch = sketches.add(component.xZConstructionPlane)
    lines = blank_sketch.sketchCurves.sketchLines
    arcs = blank_sketch.sketchCurves.sketchArcs

    # 喉母圆半径 rg = a - da2 / 2
    # z_arc_end 与 _compute_section_curves 共用同一个辅助函数：
    # 任何一侧被修改而另一侧忘记同步，都会导致齿槽放样轮廓越过轮坯真实外缘。
    z_arc_end = _throat_arc_end(a, rg, re2, b2)
    if (b2 / 2.0) - z_arc_end < 1e-4:
        # 圆弧已经延伸到端面，无需倒斜角
        z_arc_end = (b2 / 2.0)
    x_arc_end = a - math.sqrt(max(1e-6, rg ** 2 - z_arc_end ** 2))

    # 在 3D 模型空间定义轮廓关键顶点 (全部位于 XZ 平面，Y = 0)
    p_arc_low_m = adsk.core.Point3D.create(x_arc_end, 0.0, -z_arc_end)
    p_arc_mid_m = adsk.core.Point3D.create(da2 / 2.0, 0.0, 0.0)
    p_arc_top_m = adsk.core.Point3D.create(x_arc_end, 0.0, z_arc_end)

    # 蜗轮端面倒斜角尺寸 (默认单边为总高度 b2 的 5%)
    wheel_chamfer_c = params.get(
        "wheel_chamfer_c", 0.05 * params["wheel_face_width_b2"]
    ) * scale
    dz_rim = (b2 / 2.0) - z_arc_end
    c_eff = min(wheel_chamfer_c, dz_rim * 0.95, (b2 / 2.0) * 0.45) if dz_rim > 1e-4 else 0.0

    has_chamfer = (c_eff > 1e-4)
    if has_chamfer:
        t_ch = ((b2 / 2.0) - c_eff - z_arc_end) / dz_rim
        x_chamfer_rim = x_arc_end + t_ch * (re2 - x_arc_end)
        z_chamfer_rim = (b2 / 2.0) - c_eff

        p_chamfer_rim_top_m = adsk.core.Point3D.create(x_chamfer_rim, 0.0, z_chamfer_rim)
        p_chamfer_face_top_m = adsk.core.Point3D.create(re2 - c_eff, 0.0, b2 / 2.0)

        p_chamfer_face_low_m = adsk.core.Point3D.create(re2 - c_eff, 0.0, -b2 / 2.0)
        p_chamfer_rim_low_m = adsk.core.Point3D.create(x_chamfer_rim, 0.0, -z_chamfer_rim)
    else:
        p_rim_top_m = adsk.core.Point3D.create(re2, 0.0, b2 / 2.0)
        p_rim_low_m = adsk.core.Point3D.create(re2, 0.0, -b2 / 2.0)

    p_center_top_m = adsk.core.Point3D.create(0.0, 0.0, b2 / 2.0)
    p_center_low_m = adsk.core.Point3D.create(0.0, 0.0, -b2 / 2.0)

    # 转换至草图本地二维坐标空间
    s_arc_low = blank_sketch.modelToSketchSpace(p_arc_low_m)
    s_arc_mid = blank_sketch.modelToSketchSpace(p_arc_mid_m)
    s_arc_top = blank_sketch.modelToSketchSpace(p_arc_top_m)
    s_center_top = blank_sketch.modelToSketchSpace(p_center_top_m)
    s_center_low = blank_sketch.modelToSketchSpace(p_center_low_m)

    # 绘制喉部圆弧
    throat_arc = arcs.addByThreePoints(s_arc_low, s_arc_mid, s_arc_top)
    curr_pt = throat_arc.endSketchPoint

    if has_chamfer:
        s_ch_rim_top = blank_sketch.modelToSketchSpace(p_chamfer_rim_top_m)
        s_ch_face_top = blank_sketch.modelToSketchSpace(p_chamfer_face_top_m)
        s_ch_face_low = blank_sketch.modelToSketchSpace(p_chamfer_face_low_m)
        s_ch_rim_low = blank_sketch.modelToSketchSpace(p_chamfer_rim_low_m)

        # 1. 顶端斜面轮廓
        if p_arc_top_m.distanceTo(p_chamfer_rim_top_m) > 1e-4:
            l_rim = lines.addByTwoPoints(curr_pt, s_ch_rim_top)
            curr_pt = l_rim.endSketchPoint
        # 2. 顶端倒斜角 (Chamfer)
        l_chamfer_top = lines.addByTwoPoints(curr_pt, s_ch_face_top)
        curr_pt = l_chamfer_top.endSketchPoint
        # 3. 顶端平端面
        l_top = lines.addByTwoPoints(curr_pt, s_center_top)
        # 4. 中心回转轴线
        center_line = lines.addByTwoPoints(l_top.endSketchPoint, s_center_low)
        curr_pt = center_line.endSketchPoint
        # 5. 底端平端面
        l_bot = lines.addByTwoPoints(curr_pt, s_ch_face_low)
        curr_pt = l_bot.endSketchPoint
        # 6. 底端倒斜角 (Chamfer)
        l_chamfer_bot = lines.addByTwoPoints(curr_pt, s_ch_rim_low)
        curr_pt = l_chamfer_bot.endSketchPoint
        # 7. 底端斜面轮廓闭合回喉部圆弧
        lines.addByTwoPoints(curr_pt, throat_arc.startSketchPoint)
    else:
        s_rim_top = blank_sketch.modelToSketchSpace(p_rim_top_m)
        s_rim_low = blank_sketch.modelToSketchSpace(p_rim_low_m)
        if p_arc_top_m.distanceTo(p_rim_top_m) > 1e-4:
            l1 = lines.addByTwoPoints(curr_pt, s_rim_top)
            curr_pt = l1.endSketchPoint
        l2 = lines.addByTwoPoints(curr_pt, s_center_top)
        center_line = lines.addByTwoPoints(l2.endSketchPoint, s_center_low)
        curr_pt = center_line.endSketchPoint
        if p_arc_low_m.distanceTo(p_rim_low_m) > 1e-4:
            l4 = lines.addByTwoPoints(curr_pt, s_rim_low)
            curr_pt = l4.endSketchPoint
        lines.addByTwoPoints(curr_pt, throat_arc.startSketchPoint)

    if blank_sketch.profiles.count == 0:
        raise RuntimeError("蜗轮轮坯草图未形成闭合轮廓")

    # 取面积最大的封闭轮廓作为回转截面 (中心线两侧的碎面被自动忽略)
    max_prof = None
    max_area = -1.0
    for prof in blank_sketch.profiles:
        try:
            area = prof.areaProperties().area
            if area > max_area:
                max_area = area
                max_prof = prof
        except Exception:
            pass

    if max_prof is None:
        max_prof = blank_sketch.profiles.item(0)

    revolves = features.revolveFeatures
    rev_input = revolves.createInput(
        max_prof, center_line, adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    )
    rev_input.setAngleExtent(False, adsk.core.ValueInput.createByString("360 deg"))
    blank_feat = revolves.add(rev_input)

    wheel_body = blank_feat.bodies.item(0)
    wheel_body.name = "WormWheel"

    # =========================================================================
    # 2. 多截面共轭齿廓实体放样生成单齿槽 (Conjugate Section Loft)
    # 在平行截面绘制共轭轮廓，一次性实体放样 (Solid Loft)，极速高光洁。
    # =========================================================================
    if quality == "ultra":
        K_sections = 11
    elif quality == "fine":
        K_sections = 9
    elif quality == "fast":
        K_sections = 5
    else:
        K_sections = 7

    sections_data = _compute_section_curves(params, K=K_sections)
    if len(sections_data) < 3:
        raise RuntimeError("计算单齿槽共轭截面失败，未能获取足够的有效截面")

    # 在临时子组件中进行截面草图与放样创建，保持主时间线绝对干净
    temp_occ = component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    temp_mgr = adsk.fusion.TemporaryBRepManager.get()
    slot_ref = None
    try:
        temp_comp = temp_occ.component
        temp_planes = temp_comp.constructionPlanes
        temp_sketches = temp_comp.sketches
        loft_feats = temp_comp.features.loftFeatures
        loft_input = loft_feats.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

        for s in sections_data:
            plane_input = temp_planes.createInput()
            offset_val = adsk.core.ValueInput.createByReal(s["zk"])
            plane_input.setByOffset(temp_comp.xYConstructionPlane, offset_val)
            c_plane = temp_planes.add(plane_input)

            sk = temp_sketches.add(c_plane)
            sk_lines = sk.sketchCurves.sketchLines
            sk_splines = sk.sketchCurves.sketchFittedSplines

            # 清洗微小重合点
            def _clean(pts):
                out = [pts[0]]
                for pt in pts[1:]:
                    if math.hypot(pt[0] - out[-1][0], pt[1] - out[-1][1]) > 1e-4:
                        out.append(pt)
                return out

            clean_r = _clean(s["pts_right"])
            clean_l = _clean(s["pts_left"])
            if len(clean_r) < 2 or len(clean_l) < 2:
                raise RuntimeError(
                    "单齿槽截面 zk={:.4f}mm 的有效齿廓点不足，无法放样。".format(s["zk"] * 10.0)
                )

            def _add_curve(sk_obj, clean_pts, splines, ln_coll):
                coll = adsk.core.ObjectCollection.create()
                for pt in clean_pts:
                    coll.add(adsk.core.Point3D.create(pt[0], pt[1], 0.0))
                try:
                    spline = splines.add(coll)
                    return spline.startSketchPoint, spline.endSketchPoint
                except Exception:
                    # 退化情况：用折线代替样条
                    prev_p = None
                    start_p = None
                    for pt in clean_pts:
                        sp = sk_obj.sketchPoints.add(adsk.core.Point3D.create(pt[0], pt[1], 0.0))
                        if prev_p is None:
                            start_p = sp
                        else:
                            ln_coll.addByTwoPoints(prev_p, sp)
                        prev_p = sp
                    return start_p, prev_p

            start_r, end_r = _add_curve(sk, clean_r, sk_splines, sk_lines)
            start_l, end_l = _add_curve(sk, clean_l, sk_splines, sk_lines)

            # 封闭齿顶边界与齿根边界
            sk_lines.addByTwoPoints(end_r, end_l)
            sk_lines.addByTwoPoints(start_l, start_r)

            if sk.profiles.count == 0:
                raise RuntimeError(
                    "单齿槽截面 zk={:.4f}mm 的草图未形成闭合轮廓。".format(s["zk"] * 10.0)
                )

            best_prof = None
            max_a = -1.0
            for p in sk.profiles:
                try:
                    pa = p.areaProperties().area
                    if pa > max_a:
                        max_a = pa
                        best_prof = p
                except Exception:
                    pass
            if best_prof is None:
                best_prof = sk.profiles.item(0)

            loft_input.loftSections.add(best_prof)

        if loft_input.loftSections.count < 3:
            raise RuntimeError("单齿槽放样截面数量不足，放样未能生成有效截面集合")

        loft_input.isSolid = True
        try:
            loft_input.isTangentEdgesMerged = True
        except Exception:
            pass
        loft_feat = loft_feats.add(loft_input)
        if loft_feat.bodies.count == 0:
            raise RuntimeError("单齿槽实体放样未生成任何实体。")
        slot_body = loft_feat.bodies.item(0)
        slot_ref = temp_mgr.copy(slot_body)
    finally:
        try:
            temp_occ.deleteMe()
        except Exception:
            pass

    if slot_ref is None or slot_ref.volume <= 1e-5:
        raise RuntimeError("单齿槽实体放样未能生成有效三维几何体")

    # =========================================================================
    # 3. 全周 z2 次环向差集阵列切除完整蜗轮 (Circular Pattern of Single Slot)
    # =========================================================================
    blank_copy = temp_mgr.copy(wheel_body)
    v_init = blank_copy.volume

    n_failed = 0
    for k in range(geom["z2"]):
        angle_k = k * (2.0 * math.pi / float(geom["z2"]))
        mat_k = adsk.core.Matrix3D.create()
        mat_k.setToRotation(
            angle_k,
            adsk.core.Vector3D.create(0.0, 0.0, 1.0),
            adsk.core.Point3D.create(0.0, 0.0, 0.0)
        )
        slot_k = temp_mgr.copy(slot_ref)
        temp_mgr.transform(slot_k, mat_k)
        try:
            temp_mgr.booleanOperation(
                blank_copy, slot_k, adsk.fusion.BooleanTypes.DifferenceBooleanType
            )
        except Exception:
            n_failed += 1

    v_final = blank_copy.volume
    cut_vol = v_init - v_final
    if cut_vol <= 1e-4:
        raise RuntimeError(
            "蜗轮切削未能有效切除材料 (初始: {:.3f} cm³, 最终: {:.3f} cm³, "
            "布尔失败 {} / {} 次)".format(v_init, v_final, n_failed, geom["z2"])
        )
    if n_failed > 0:
        raise RuntimeError(
            "蜗轮齿槽布尔差集有 {} / {} 次失败，几何不完整，已中止以避免输出错误模型。"
            "请尝试减小齿数 z2、增大模数或降低齿面精度后重试。".format(n_failed, geom["z2"])
        )

    # =========================================================================
    # 4. 将最终光滑连续蜗轮实体注入设计环境并移除初始轮坯
    # =========================================================================
    remove_feats = features.removeFeatures

    design = adsk.fusion.Design.cast(component.parentDesign)
    final_body = None

    if design and design.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        # 参数化设计：必须通过 BaseFeature 注入外部 BRep，再删除原轮坯
        base_feat = None
        try:
            base_feat = features.baseFeatures.add()
            base_feat.startEdit()
            try:
                component.bRepBodies.add(blank_copy, base_feat)
            finally:
                base_feat.finishEdit()
        except Exception:
            # 注入失败则回退到非参数化路径，避免静默产出空结果
            if base_feat is not None:
                try:
                    base_feat.deleteMe()
                except Exception:
                    pass
            final_body = component.bRepBodies.add(blank_copy)
        else:
            if base_feat.bodies.count == 0:
                raise RuntimeError("蜗轮实体注入设计环境失败：BaseFeature 未产生任何实体。")
            max_vol = -1.0
            main_body = None
            for b in base_feat.bodies:
                try:
                    if b.volume > max_vol:
                        max_vol = b.volume
                        main_body = b
                except Exception:
                    pass
            if main_body is None:
                main_body = base_feat.bodies.item(0)
            for b in base_feat.bodies:
                if b != main_body:
                    try:
                        features.removeFeatures.add(b)
                    except Exception:
                        try:
                            b.isVisible = False
                        except Exception:
                            pass
            final_body = main_body
    else:
        final_body = component.bRepBodies.add(blank_copy)

    if final_body is None:
        raise RuntimeError("蜗轮实体注入设计环境失败。")

    # 移除初始轮坯
    try:
        remove_feats.add(wheel_body)
    except Exception:
        try:
            wheel_body.isVisible = False
        except Exception:
            pass

    # 隐藏轮坯旋转草图与所有构造基准面，保持视口绝对整洁
    for sk in component.sketches:
        try:
            sk.isLightBulbOn = False
        except Exception:
            pass
    for pl in component.constructionPlanes:
        try:
            pl.isLightBulbOn = False
        except Exception:
            pass

    final_body.name = "WormWheel"
    return final_body
