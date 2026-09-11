"""
worm_builder.py - Fusion 360 蜗杆 (Worm) 实体建模模块

采用「3D 拟合螺旋线 (分度圆路径 + 外圆导引轨) + 精确法向截面垂直扫掠切削」：
保证 ZA/ZN 齿形精度与几何稳定性，消除畸变与端面尖角。

本模块只生成**纯圆柱蜗杆**本体（长度严格等于 b1，无外伸轴颈），
不包含任何虚拟滚刀 / 展成包络分支。
"""

import math
import adsk.core
import adsk.fusion


def build_worm(
    component: adsk.fusion.Component,
    params: dict,
) -> adsk.fusion.BRepBody:
    """
    在指定组件中生成纯蜗杆实体 (长度严格等于 b1，无外伸轴颈)。

    :param component: Fusion 360 目标组件 (Component)
    :param params: worm_math.to_dict() 输出的参数字典
    :return: 生成的 BRepBody 实体对象
    """
    scale = 0.1  # mm 转 cm (Fusion 内部标准单位为 cm)

    m = params["module_m"] * scale
    z1 = params["starts_z1"]
    direction = params.get("direction", "Right")

    # 尺寸提取
    r0 = (params["worm_pitch_diameter_d1"] / 2.0) * scale
    ra = (params["worm_tip_diameter_da1"] / 2.0) * scale
    hf1 = params["dedendum_hf1"] * scale

    b1 = params["worm_length_b1"] * scale
    if b1 <= 0.0:
        raise RuntimeError(
            "蜗杆螺纹长度 b1 = {:.3f} mm 必须为正数。".format(params["worm_length_b1"])
        )

    px = params["axial_pitch_px"] * scale
    pz = params["lead_pz"] * scale
    alpha_n = math.radians(params["pressure_angle_normal_deg"])
    en = params["worm_normal_space_en"] * scale
    backlash_jn = params.get("normal_backlash_jn", 0.0) * scale
    # 齿侧间隙处理：将法向侧隙的 50% 作用于蜗杆 (加宽切槽刀具，使生成的蜗杆齿厚减少 jn / 2)
    en_eff = en + (backlash_jn / 2.0)

    features = component.features
    sketches = component.sketches

    # =========================================================================
    # 1. 建立圆柱毛坯实体
    # 毛坯直接拉伸到**精确理论长度 b1**（对称，z ∈ [-b1/2, +b1/2]），
    # 端面由拉伸本身确定，不再需要事后剖切。
    #
    # 说明：旧做法是先拉到 b1 + 2*pz 再用两个基准面 SplitBody 剖平，
    # 但删除分段（removeFeatures.add）会使剖切特征返回的体代理失效，
    # 第二次剖切必然抛 BODY_REFERENCE_LOST。既然螺旋扫掠路径两端本就
    # 超出毛坯各 1.0*pz，刀具能完整进入/退出，直接拉伸到 b1 即可得到
    # 两端切平的精确长度蜗杆，彻底避开实体参考失效的问题。
    # =========================================================================
    ext_margin = 1.0 * pz          # 仅用于螺旋路径的越程量（刀具出入刀）
    total_stock = b1

    if total_stock <= 0.0:
        raise RuntimeError(
            "蜗杆螺纹长度 b1 = {:.3f} mm 必须为正数。".format(b1 * 10.0)
        )

    extrudes = features.extrudeFeatures
    blank_sketch = sketches.add(component.xYConstructionPlane)
    blank_sketch.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(0, 0, 0), ra
    )
    blank_prof = blank_sketch.profiles.item(0)
    ext = extrudes.createInput(blank_prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ext.setSymmetricExtent(adsk.core.ValueInput.createByReal(total_stock / 2.0), False)
    blank_feat = extrudes.add(ext)
    worm_body = blank_feat.bodies.item(0)
    worm_body.name = "Worm"

    # =========================================================================
    # 2. 生成高精度 3D 分度圆螺旋线 (Pitch Helix) 与 外圆导引线 (Guide Rail)
    # 采用双螺旋线轨迹 (Path + GuideRail)，消除三维螺旋线微分挠率导致的截面扭转
    # 保证齿槽刀具在整长范围内严格沿径向姿态运行，根除实体自交 (ASM_SELF_INTER)
    # 螺旋线起点终点均超出毛坯端面 (ext_margin + 0.2*pz)，保证刀具完整出入刀
    # =========================================================================
    z_start = -(b1 / 2.0 + ext_margin + 0.2 * pz)
    z_end = +(b1 / 2.0 + ext_margin + 0.2 * pz)
    total_length_z = z_end - z_start
    total_turns = total_length_z / pz

    pts_per_turn = 36
    num_points = max(24, int(total_turns * pts_per_turn)) + 1

    is_rh = (direction == "Right")
    y_sign = 1.0 if is_rh else -1.0

    path_points = adsk.core.ObjectCollection.create()
    rail_points = adsk.core.ObjectCollection.create()

    for i in range(num_points):
        th = (z_start / pz) * 2.0 * math.pi + i * (total_turns * 2.0 * math.pi) / (num_points - 1)
        z = (pz / (2.0 * math.pi)) * th
        cos_th = math.cos(th)
        sin_th = y_sign * math.sin(th)
        path_points.add(adsk.core.Point3D.create(r0 * cos_th, r0 * sin_th, z))
        rail_points.add(adsk.core.Point3D.create(ra * cos_th, ra * sin_th, z))

    path_sketch = sketches.add(component.xYConstructionPlane)
    path_sketch.is3D = True
    path_spline = path_sketch.sketchCurves.sketchFittedSplines.add(path_points)
    helix_path = features.createPath(path_spline)

    rail_spline = path_sketch.sketchCurves.sketchFittedSplines.add(rail_points)
    rail_path = features.createPath(rail_spline)

    # =========================================================================
    # 3. 螺旋线起点建立法向截面基准面
    # =========================================================================
    planes = component.constructionPlanes
    plane_input = planes.createInput()
    plane_input.setByDistanceOnPath(helix_path, adsk.core.ValueInput.createByReal(0.0))
    norm_plane = planes.add(plane_input)
    norm_plane.isLightBulbOn = False

    # =========================================================================
    # 4. 基于解析三维正交基底绘制精确法向齿槽梯形草图 (Trapezoid Cutter Profile)
    # 利用微分几何切向量与径向向量外积求得副法向，消除局部投影偏角
    # =========================================================================
    th0 = (z_start / pz) * 2.0 * math.pi
    p0 = adsk.core.Point3D.create(r0 * math.cos(th0), y_sign * r0 * math.sin(th0), z_start)

    # 严格径向向量 ur (指向外)
    ur = [math.cos(th0), y_sign * math.sin(th0), 0.0]

    # 切向量 T = (-r0 * sin(th0), y_sign * r0 * cos(th0), pz / (2*pi))
    c_pitch = pz / (2.0 * math.pi)
    # 副法向 ut = (T x ur) / ||T x ur||
    ut_raw = [-c_pitch * y_sign * math.sin(th0), c_pitch * math.cos(th0), -y_sign * r0]
    ut_len = math.sqrt(ut_raw[0] ** 2 + ut_raw[1] ** 2 + ut_raw[2] ** 2)
    ut = [ut_raw[0] / ut_len, ut_raw[1] / ut_len, ut_raw[2] / ut_len]

    delta_top = ra - r0
    w_top = en_eff / 2.0 + delta_top * math.tan(alpha_n)

    # 根圆深度严格为 -hf1 (hf1 已包含标准顶隙 c = 0.2m，严禁重复扣减)
    delta_bot = -hf1
    w_bot = max(0.05 * m, en_eff / 2.0 + delta_bot * math.tan(alpha_n))

    # 在三维模型空间生成 4 个绝对共面的梯形角点
    v1 = adsk.core.Point3D.create(p0.x + delta_top * ur[0] + w_top * ut[0], p0.y + delta_top * ur[1] + w_top * ut[1], p0.z + delta_top * ur[2] + w_top * ut[2])
    v2 = adsk.core.Point3D.create(p0.x + delta_top * ur[0] - w_top * ut[0], p0.y + delta_top * ur[1] - w_top * ut[1], p0.z + delta_top * ur[2] - w_top * ut[2])
    v3 = adsk.core.Point3D.create(p0.x + delta_bot * ur[0] - w_bot * ut[0], p0.y + delta_bot * ur[1] - w_bot * ut[1], p0.z + delta_bot * ur[2] - w_bot * ut[2])
    v4 = adsk.core.Point3D.create(p0.x + delta_bot * ur[0] + w_bot * ut[0], p0.y + delta_bot * ur[1] + w_bot * ut[1], p0.z + delta_bot * ur[2] + w_bot * ut[2])

    cutter_sketch = sketches.add(norm_plane)
    p1_2d = cutter_sketch.modelToSketchSpace(v1)
    p2_2d = cutter_sketch.modelToSketchSpace(v2)
    p3_2d = cutter_sketch.modelToSketchSpace(v3)
    p4_2d = cutter_sketch.modelToSketchSpace(v4)

    c_lines = cutter_sketch.sketchCurves.sketchLines
    l1 = c_lines.addByTwoPoints(p1_2d, p2_2d)
    l2 = c_lines.addByTwoPoints(l1.endSketchPoint, p3_2d)
    l3 = c_lines.addByTwoPoints(l2.endSketchPoint, p4_2d)
    c_lines.addByTwoPoints(l3.endSketchPoint, l1.startSketchPoint)

    if cutter_sketch.profiles.count == 0:
        raise RuntimeError("齿槽刀具草图未形成有效封闭轮廓")
    cutter_prof = cutter_sketch.profiles.item(0)

    # =========================================================================
    # 5. 导引轨道约束扫掠切削 (Path + GuideRail Sweep Cut, No Scaling)
    # 利用外圆导引轨固定主法向姿态，ProfileNoScalingOption 维持精确标称齿形
    # =========================================================================
    sweeps = features.sweepFeatures
    sweep_input = sweeps.createInput(
        cutter_prof, helix_path, adsk.fusion.FeatureOperations.CutFeatureOperation
    )
    try:
        sweep_input.guideRail = rail_path
        sweep_input.profileScaling = adsk.fusion.SweepProfileScalingOptions.SweepProfileNoScalingOption
    except Exception:
        sweep_input.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType

    try:
        sweep_input.participantBodies = [worm_body]
    except Exception:
        pass

    try:
        sweep_feat = sweeps.add(sweep_input)
    except Exception as e_sweep:
        # 若 guideRail 特性在部分版本受限，回退至纯垂直扫掠
        sweep_input_retry = sweeps.createInput(
            cutter_prof, helix_path, adsk.fusion.FeatureOperations.CutFeatureOperation
        )
        sweep_input_retry.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
        try:
            sweep_input_retry.participantBodies = [worm_body]
        except Exception:
            pass
        try:
            sweep_feat = sweeps.add(sweep_input_retry)
        except Exception as e_retry:
            raise RuntimeError(f"蜗杆螺旋扫掠切削失败: {e_sweep} (回退垂直扫掠失败: {e_retry})")

    # =========================================================================
    # 6. 多头环向阵列 (z1 > 1)
    # =========================================================================
    if z1 > 1:
        patterns = features.circularPatternFeatures
        pattern_entities = adsk.core.ObjectCollection.create()
        pattern_entities.add(sweep_feat)
        pattern_input = patterns.createInput(pattern_entities, component.zConstructionAxis)
        pattern_input.quantity = adsk.core.ValueInput.createByReal(z1)
        pattern_input.totalAngle = adsk.core.ValueInput.createByString("360 deg")
        pattern_input.isSymmetric = False
        try:
            patterns.add(pattern_input)
        except Exception as e_pat:
            # 阵列失败绝不能静默吞掉：否则只会切出 1 条螺旋槽，
            # 用户拿到的是一根单头蜗杆，却以为 z1 头已生成。
            raise RuntimeError(
                f"蜗杆 {z1} 头环向阵列失败: {e_pat}。已中止以避免输出头数错误的蜗杆。"
            )

    # =========================================================================
    # 7. 端面说明
    # 毛坯在第 1 步已直接拉伸到精确长度 b1，两端端面由拉伸自动切平，
    # 因此这里不再需要 SplitBody 剖切。螺旋扫掠路径两端各超出毛坯 1.0*pz，
    # 刀具能完整进入与退出，因此端面处的螺纹也是完整的。
    # 这样彻底避开了「删除分段后实体代理失效 (BODY_REFERENCE_LOST)」的问题。
    # =========================================================================
    try:
        bb = worm_body.boundingBox
        if bb.minPoint.z < -(b1 / 2.0) - 1e-3 or bb.maxPoint.z > (b1 / 2.0) + 1e-3:
            raise RuntimeError(
                "蜗杆实体长度超出理论值 b1：实际 z ∈ [{:.3f}, {:.3f}] mm，理论为 [{:.3f}, {:.3f}] mm。".format(
                    bb.minPoint.z * 10.0, bb.maxPoint.z * 10.0, -b1 * 5.0, b1 * 5.0
                )
            )
    except RuntimeError:
        raise
    except Exception:
        pass

    # =========================================================================
    # 8. 蜗杆齿顶两条螺旋棱边倒圆角/倒角特征 (默认 0.0 即保持尖锐齿顶)
    # 精确匹配齿顶外圆面与左右两侧螺旋齿面相交生成的 3D 螺旋棱边 (Helical Crest Edges)
    # =========================================================================
    worm_fillet_r = params.get("worm_fillet_r", 0.0) * scale
    if worm_fillet_r > 1e-4:
        # 限制安全倒角/倒圆半径，确保两边倒角不会在齿顶中央重叠冲突 (不超过 0.35 * m)
        safe_r = min(worm_fillet_r, 0.35 * m)
        tol = 3e-2  # 0.3 mm 径向容差

        crest_edges = adsk.core.ObjectCollection.create()
        for edge in worm_body.edges:
            try:
                bb = edge.boundingBox
                z_span = bb.maxPoint.z - bb.minPoint.z
                # 1. 螺旋棱边必定具有显著的 Z 向跨度 (排除端面上 Z 坐标恒定的平面圆弧/线段)
                if z_span < 0.05:
                    continue

                # 2. 棱边几何类型必须为空间拟合曲线，排除沿轴向的直线接缝 (Line3D)
                geom = edge.geometry
                if isinstance(geom, adsk.core.Line3D):
                    continue
                if "Line3D" in str(getattr(geom, "objectType", "")):
                    continue

                # 3. 在 XY 平面内必须具有显著跨度 (螺旋环绕特性)，排除圆柱轴向接缝 (Seam Line)
                xy_span = math.hypot(bb.maxPoint.x - bb.minPoint.x, bb.maxPoint.y - bb.minPoint.y)
                if xy_span < 0.05:
                    continue

                # 4. 棱边上的中点径向半径必须处于蜗杆顶圆柱面 ra 处
                p_m = edge.pointOnEdge
                r_m = math.hypot(p_m.x, p_m.y)
                if abs(r_m - ra) > tol:
                    continue

                # 5. 棱边起点与终点顶点也必须均处于顶圆柱面 ra 处
                p_s = edge.startVertex.geometry if edge.startVertex else None
                p_e = edge.endVertex.geometry if edge.endVertex else None
                if p_s and abs(math.hypot(p_s.x, p_s.y) - ra) > tol:
                    continue
                if p_e and abs(math.hypot(p_e.x, p_e.y) - ra) > tol:
                    continue

                # 6. 相邻拓扑面校验：齿顶棱边必须恰好一侧为顶圆柱面，另一侧为切削齿侧面
                faces = edge.faces
                if faces.count == 2:
                    g0 = faces.item(0).geometry
                    g1 = faces.item(1).geometry
                    f0_cyl = isinstance(g0, adsk.core.Cylinder) or "Cylinder" in str(getattr(g0, "objectType", ""))
                    f1_cyl = isinstance(g1, adsk.core.Cylinder) or "Cylinder" in str(getattr(g1, "objectType", ""))
                    # 若两侧均为圆柱面，说明是圆柱实体本身的平滑接缝线，排除
                    if f0_cyl and f1_cyl:
                        continue
                    # 必须有一侧是圆柱面
                    if not (f0_cyl or f1_cyl):
                        continue

                crest_edges.add(edge)
            except Exception:
                pass

        if crest_edges.count > 0:
            fillets = features.filletFeatures
            chamfers = features.chamferFeatures
            applied = False

            def is_healthy(feat) -> bool:
                try:
                    if not feat or not feat.isValid:
                        return False
                    hs = getattr(feat, "healthState", None)
                    if hs is not None:
                        if hs == adsk.fusion.FeatureHealthStates.WarningFeatureHealthState:
                            return False
                        if hs == adsk.fusion.FeatureHealthStates.ErrorFeatureHealthState:
                            return False
                    return True
                except Exception:
                    return True

            # 策略 1: 整体恒定半径倒圆角 (Fillet)
            try:
                f_in = fillets.createInput()
                f_in.edgeSetInputs.addConstantRadiusEdgeSet(
                    crest_edges, adsk.core.ValueInput.createByReal(safe_r), False
                )
                f_feat = fillets.add(f_in)
                if is_healthy(f_feat):
                    applied = True
                else:
                    try:
                        f_feat.deleteMe()
                    except Exception:
                        pass
            except Exception:
                pass

            # 策略 2: 若整体倒圆角失败，尝试逐条边施加倒圆角
            if not applied:
                single_fillets = []
                all_single_ok = True
                for i in range(crest_edges.count):
                    try:
                        s_coll = adsk.core.ObjectCollection.create()
                        s_coll.add(crest_edges.item(i))
                        f_in = fillets.createInput()
                        f_in.edgeSetInputs.addConstantRadiusEdgeSet(
                            s_coll, adsk.core.ValueInput.createByReal(safe_r), False
                        )
                        sf = fillets.add(f_in)
                        if is_healthy(sf):
                            single_fillets.append(sf)
                        else:
                            try:
                                sf.deleteMe()
                            except Exception:
                                pass
                            all_single_ok = False
                    except Exception:
                        all_single_ok = False

                if len(single_fillets) > 0 and all_single_ok:
                    applied = True
                elif not all_single_ok:
                    for sf in single_fillets:
                        try:
                            sf.deleteMe()
                        except Exception:
                            pass

            # 策略 3: 若倒圆角因螺旋曲面微分曲率限制失败，降级为等距倒斜角 (Chamfer)
            if not applied:
                try:
                    ch_in = chamfers.createInput(crest_edges, False)
                    ch_in.setToEqualDistance(adsk.core.ValueInput.createByReal(safe_r))
                    ch_feat = chamfers.add(ch_in)
                    if is_healthy(ch_feat):
                        applied = True
                    else:
                        try:
                            ch_feat.deleteMe()
                        except Exception:
                            pass
                except Exception:
                    pass

            # 策略 4: 逐条边施加等距倒斜角保底
            if not applied:
                for i in range(crest_edges.count):
                    try:
                        s_coll = adsk.core.ObjectCollection.create()
                        s_coll.add(crest_edges.item(i))
                        ch_in = chamfers.createInput(s_coll, False)
                        ch_in.setToEqualDistance(adsk.core.ValueInput.createByReal(safe_r))
                        ch_feat = chamfers.add(ch_in)
                        if is_healthy(ch_feat):
                            applied = True
                        else:
                            try:
                                ch_feat.deleteMe()
                            except Exception:
                                pass
                    except Exception:
                        pass

            # 四种策略全部失败时给出明确提示，而不是静默输出尖锐齿顶
            if not applied:
                raise RuntimeError(
                    "蜗杆齿顶倒角/倒圆全部策略失败。请将「蜗杆齿顶倒角」设为 0 mm 后重试"
                    "（保持尖锐齿顶），或减小该倒角值。"
                )

    # =========================================================================
    # 9. 隐藏所有用于剖切/导引的构造基准面与草图，保持视口绝对整洁
    # =========================================================================
    for pl in component.constructionPlanes:
        try:
            pl.isLightBulbOn = False
        except Exception:
            pass
    for sk in component.sketches:
        try:
            sk.isLightBulbOn = False
        except Exception:
            pass

    return worm_body
