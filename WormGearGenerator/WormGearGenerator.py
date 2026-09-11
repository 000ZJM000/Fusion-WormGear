"""
WormGearGenerator.py - Fusion 360 蜗轮蜗杆参数化生成器主入口模块

蜗杆：螺旋线扫掠切削；蜗轮：多截面共轭齿廓放样 + 全周阵列差集。
遵循 GB/T 10085-2018。
"""

import sys
import os
import math
import traceback
import importlib

import adsk.core
import adsk.fusion

# 确保本插件所在工作区绝对路径在 sys.path 首位
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir in sys.path:
    sys.path.remove(current_dir)
sys.path.insert(0, current_dir)


def reload_submodules():
    """强制重新加载子模块，确保 Fusion 360 每次运行都读取当前工作区最新代码，杜绝旧路径缓存"""
    if current_dir in sys.path:
        sys.path.remove(current_dir)
    sys.path.insert(0, current_dir)

    # 关键：清空 importlib 的目录缓存。否则仅靠 __pycache__ 的 (mtime, size) 校验，
    # 在文件被快速改写时可能读回旧字节码，使"重新加载"实际加载到旧代码。
    importlib.invalidate_caches()

    for mod in ["gear_builder", "worm_builder", "worm_math"]:
        if mod in sys.modules:
            del sys.modules[mod]

    import worm_math
    import worm_builder
    import gear_builder
    return worm_math, worm_builder, gear_builder


# 初始加载模块
worm_math, worm_builder, gear_builder = reload_submodules()

from worm_math import WormGearMath
from worm_builder import build_worm
from gear_builder import build_worm_wheel

COMMAND_ID = "ParametricWormGearGeneratorCmd"
COMMAND_NAME = "蜗轮蜗杆生成器"
COMMAND_DESCRIPTION = "基于国标 GB/T 10085 生成精密蜗轮蜗杆实体与装配体 (多截面共轭齿廓放样)"
WORKSPACE_ID = "FusionSolidEnvironment"
PANEL_ID = "SolidCreatePanel"

# 全局事件监听器引用列表，防止 Python 垃圾回收导致 UI 按钮无响应。
# 每打开一次命令都会新建一批监听器，因此必须在命令销毁时按批清理，
# 否则列表会随使用次数无限增长（旧实现只增不减）。
handlers = []

# 最近一次"实时计算校核"的失败原因，供界面提示使用
_last_calc_error = ""


class WormGearCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
        self._batch = []

    def notify(self, args: adsk.core.CommandCreatedEventArgs):
        try:
            cmd = args.command
            # 正确拼写为 isExecutedWhenPreEmpted（大写 E）。旧代码写成 isExecutedWhenPreempted，
            # Fusion 对象上给不存在的属性赋值会静默失败，该行一直是空操作。
            try:
                cmd.isExecutedWhenPreEmpted = False
            except Exception:
                pass

            # 注册生命周期与交互事件监听。本批监听器由本 handler 持有，
            # 命令销毁时统一回收，避免全局 handlers 列表无限增长。
            self._batch = []

            def _reg(event, handler_obj):
                event.add(handler_obj)
                self._batch.append(handler_obj)
                handlers.append(handler_obj)

            _reg(cmd.execute, WormGearCommandExecuteHandler())
            _reg(cmd.inputChanged, WormGearInputChangedHandler())
            _reg(cmd.validateInputs, WormGearValidateInputsHandler())
            _reg(cmd.destroy, WormGearCommandDestroyHandler(self))

            inputs = cmd.commandInputs

            # 顶部说明
            intro = inputs.addTextBoxCommandInput(
                "intro_text", "", "", 3, True
            )
            intro.isFullWidth = True
            intro.formattedText = (
                "<b>参数化蜗轮蜗杆生成器</b> —— 依据 GB/T 10085-2018 计算几何参数，"
                "生成精确长度的圆柱蜗杆与共轭蜗轮实体。<br>"
                "修改任一参数后，下方「设计校核」会实时刷新；"
                "校核不通过时会中止生成并说明原因。"
            )

            # =================================================================
            # 分组 1: 生成配置
            # =================================================================
            grp_target = inputs.addGroupCommandInput("grp_target", "生成配置")
            grp_target.isExpanded = True
            gt = grp_target.children

            drop_target = gt.addDropDownCommandInput(
                "target_comp", "生成内容", adsk.core.DropDownStyles.TextListDropDownStyle
            )
            drop_target.listItems.add("蜗轮 + 蜗杆 (装配体，含旋转副)", True)
            drop_target.listItems.add("仅生成蜗轮", False)
            drop_target.listItems.add("仅生成蜗杆", False)
            drop_target.tooltip = (
                "装配体：两件按 90° 交错轴就位，并自动建立旋转副与传动比运动链接。\n"
                "仅蜗轮 / 仅蜗杆：只生成单个零件实体。"
            )

            drop_profile = gt.addDropDownCommandInput(
                "worm_type", "蜗杆齿形制式", adsk.core.DropDownStyles.TextListDropDownStyle
            )
            drop_profile.listItems.add("ZA 阿基米德蜗杆 (轴向直廓，最常用)", True)
            drop_profile.listItems.add("ZN 法向直廓蜗杆", False)
            drop_profile.tooltip = (
                "ZA：轴向截面为直线齿廓，轴向压力角为标准值 20°。国标最常用。\n"
                "ZN：法向截面为直线齿廓，法向压力角为标准值 20°。\n"
                "两者均按法向截面刀具沿螺旋线扫掠成型；ZN 为精确成型，ZA 为法向等效近似。"
            )

            drop_dir = gt.addDropDownCommandInput(
                "direction", "螺旋旋向", adsk.core.DropDownStyles.TextListDropDownStyle
            )
            drop_dir.listItems.add("右旋 (常用)", True)
            drop_dir.listItems.add("左旋", False)
            drop_dir.tooltip = (
                "蜗杆螺旋线的旋向。旋向会同时决定蜗轮齿槽的螺旋方向与\n"
                "运动链接的传动方向，请按实际工况选择。"
            )

            drop_quality = gt.addDropDownCommandInput(
                "quality", "齿面放样精度", adsk.core.DropDownStyles.TextListDropDownStyle
            )
            drop_quality.listItems.add("高精 (9 截面，推荐)", True)
            drop_quality.listItems.add("超精 (11 截面)", False)
            drop_quality.listItems.add("标准 (7 截面)", False)
            drop_quality.listItems.add("快速 (5 截面)", False)
            drop_quality.tooltip = (
                "蜗轮齿槽沿齿宽方向的共轭齿廓放样截面数量。\n"
                "截面越多齿面越光顺，但生成耗时越长。\n"
                "齿数较多 (z2 接近 120) 时建议先用「快速」试算。"
            )

            # =================================================================
            # 分组 2: 基本参数 (GB/T 10085)
            # =================================================================
            grp_basic = inputs.addGroupCommandInput("grp_basic", "基本参数 (GB/T 10085)")
            grp_basic.isExpanded = True
            gb = grp_basic.children

            inp_module = gb.addValueInput(
                "module", "模数 m", "mm", adsk.core.ValueInput.createByString("2.0 mm")
            )
            inp_module.tooltip = "轴向模数 mx = m。标准系列：1, 1.25, 1.6, 2, 2.5, 3.15, 4, 5, 6.3, 8, 10 mm。"

            inp_z1 = gb.addIntegerSpinnerCommandInput("z1", "蜗杆头数 z1", 1, 6, 1, 1)
            inp_z1.tooltip = (
                "蜗杆螺旋线头数，传动比 i = z2 / z1。\n"
                "标准建议 1、2、4、6。z1 = 1 时自锁性最好，多头则传动效率更高。"
            )

            inp_z2 = gb.addIntegerSpinnerCommandInput("z2", "蜗轮齿数 z2", 17, 120, 1, 30)
            inp_z2.tooltip = (
                "蜗轮齿数。不得小于 17（否则严重根切）。\n"
                "GB/T 10085 建议取 28 ~ 80。"
            )

            inp_q = gb.addFloatSpinnerCommandInput("q", "直径系数 q", "", 6.0, 25.0, 0.5, 10.0)
            inp_q.tooltip = (
                "q = d1 / m，决定蜗杆分度圆直径与导程角 γ = arctan(z1 / q)。\n"
                "必须大于 2.4（否则蜗杆齿根圆直径 ≤ 0）。\n"
                "标准推荐值：6.3, 7.1, 8, 9, 10, 11.2, 12.5, 14, 16, 18, 20, 22.4, 25。\n"
                "q 越大刚度越好，但导程角越小。"
            )

            inp_alpha = gb.addValueInput(
                "alpha", "压力角 α", "deg", adsk.core.ValueInput.createByString("20.0 deg")
            )
            inp_alpha.tooltip = "标准压力角，通常取 20°。ZA 为轴向压力角，ZN 为法向压力角。"

            # =================================================================
            # 分组 3: 中心距与变位
            # =================================================================
            grp_dist = inputs.addGroupCommandInput("grp_dist", "中心距与变位")
            grp_dist.isExpanded = True
            gd = grp_dist.children

            drop_mode = gd.addDropDownCommandInput(
                "dist_mode", "配凑方式", adsk.core.DropDownStyles.TextListDropDownStyle
            )
            drop_mode.listItems.add("按变位系数 x2 计算中心距 a (标准变位)", True)
            drop_mode.listItems.add("按指定中心距 a 反求变位系数 x2", False)
            drop_mode.tooltip = (
                "两种配凑方式二选一：\n"
                "· 按 x2 求 a：a = 0.5·m·(q + z2 + 2·x2)，适合设计阶段自由取值；\n"
                "· 按 a 求 x2：x2 = a/m − 0.5·(q + z2)，适合凑配已有标准中心距。"
            )

            inp_x2 = gd.addFloatSpinnerCommandInput("x2", "变位系数 x2", "", -1.0, 1.0, 0.05, 0.0)
            inp_x2.tooltip = (
                "蜗轮变位系数。常用工程范围 −0.5 ~ +0.5。\n"
                "正值可消除根切，但过大会减少蜗轮齿顶与蜗杆齿根之间的顶隙，需留意校核提示。"
            )

            custom_a_input = gd.addValueInput(
                "custom_a", "中心距 a", "mm", adsk.core.ValueInput.createByString("40.0 mm")
            )
            custom_a_input.isVisible = False
            custom_a_input.tooltip = "目标中心距。标准优先系列：40, 50, 63, 80, 100, 125, 160, 200, 250 mm。"

            # =================================================================
            # 分组 4: 结构与工艺特征
            # =================================================================
            grp_struct = inputs.addGroupCommandInput("grp_struct", "结构与工艺特征")
            grp_struct.isExpanded = True
            gs = grp_struct.children

            calc_init = WormGearMath(m=2.0, z1=1, z2=30, q=10.0)
            init_res = calc_init.to_dict()
            b1_init_str = f"{init_res['worm_length_b1']} mm"
            b2_init_str = f"{init_res['wheel_face_width_b2']} mm"
            chamfer_init_str = f"{init_res['wheel_chamfer_c']} mm"

            inp_b1 = gs.addValueInput("b1", "蜗杆螺纹长度 b1", "mm", adsk.core.ValueInput.createByString(b1_init_str))
            inp_b1.tooltip = (
                "蜗杆螺纹段总长（纯圆柱，两端无外伸轴颈），模型长度严格等于该值。\n"
                "国标最小有效长度 b1 ≥ (11 + 0.06·z2)·m，同时必须覆盖蜗轮喉部啮合弦长。\n"
                "默认值已按标准自动推荐；手动修改后，改变模数/齿数/变位时会重新推荐。"
            )

            inp_b2 = gs.addValueInput("b2", "蜗轮齿宽 b2", "mm", adsk.core.ValueInput.createByString(b2_init_str))
            inp_b2.tooltip = (
                "蜗轮轮齿沿轴向的宽度。\n"
                "标准上限：z1 ≤ 2 时 b2 ≤ 0.75·da1；z1 = 3 时 0.70·da1；z1 ≥ 4 时 0.67·da1。\n"
                "b2 越大齿顶越薄，超过上限会被校核拦下。"
            )

            inp_backlash = gs.addValueInput(
                "backlash", "法向齿侧间隙 jn", "mm", adsk.core.ValueInput.createByString("0.0 mm")
            )
            inp_backlash.tooltip = (
                "齿侧间隙：生成时向蜗杆与蜗轮各分配一半，使实际齿厚略减、便于装配与润滑。\n"
                "0 mm 表示理论无侧隙啮合。取值过大会吃掉齿厚并被校核拦下。"
            )

            inp_chamfer = gs.addValueInput(
                "wheel_chamfer", "蜗轮端面倒斜角", "mm", adsk.core.ValueInput.createByString(chamfer_init_str)
            )
            inp_chamfer.tooltip = (
                "蜗轮两端外缘的倒斜角（单边）。默认取齿宽的 5%。\n"
                "必须小于齿宽的一半。"
            )

            inp_fillet = gs.addValueInput(
                "worm_fillet", "蜗杆齿顶倒圆", "mm", adsk.core.ValueInput.createByString("0.0 mm")
            )
            inp_fillet.tooltip = (
                "蜗杆齿顶两条螺旋棱边的倒圆半径（0 表示保留尖锐齿顶）。\n"
                "实际施加值会被限制在 0.35·m 以内，以免两侧倒圆在齿顶中央重叠。\n"
                "若倒圆/倒角全部失败会明确报错，此时请改回 0 mm。"
            )

            # =================================================================
            # 分组 5: 设计校核与传动参数
            # =================================================================
            grp_calc = inputs.addGroupCommandInput("grp_calc", "设计校核与传动参数")
            grp_calc.isExpanded = True
            gc = grp_calc.children

            calc_box = gc.addTextBoxCommandInput("calc_display", "", "", 10, True)
            calc_box.isFullWidth = True
            calc_box.tooltip = (
                "实时几何校核报告。红色为阻止生成的错误，橙色为可继续的提示。\n"
                "蜗轮由「多截面共轭齿廓放样 + 全周阵列差集」生成；\n"
                "蜗杆由「螺旋线扫掠切削」生成。"
            )

            # 初始计算刷新
            update_calculated_display(inputs)

        except Exception:
            ui = adsk.core.Application.get().userInterface
            ui.messageBox(f"初始化界面失败:\n{traceback.format_exc()}")


class WormGearInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args: adsk.core.InputChangedEventArgs):
        try:
            inputs = args.inputs
            changed_id = args.input.id

            # 中心距模式切换显示
            if changed_id == "dist_mode":
                dist_mode_item = adsk.core.DropDownCommandInput.cast(inputs.itemById("dist_mode"))
                if dist_mode_item and dist_mode_item.selectedItem:
                    is_custom_a = "指定" in dist_mode_item.selectedItem.name
                    inputs.itemById("x2").isVisible = not is_custom_a
                    inputs.itemById("custom_a").isVisible = is_custom_a

            # 核心啮合参数改变时，自动同步更新标准推荐的蜗杆长度 b1 与蜗轮齿宽 b2。
            # custom_a 也必须包含在内：切换到「指定中心距」模式后修改 a，
            # 旧的 b1/b2/倒角 会停留在旧值上，与新的中心距不再匹配。
            if changed_id in ["module", "z1", "z2", "q", "x2", "custom_a"]:
                m_inp = adsk.core.ValueCommandInput.cast(inputs.itemById("module"))
                z1_inp = adsk.core.IntegerSpinnerCommandInput.cast(inputs.itemById("z1"))
                z2_inp = adsk.core.IntegerSpinnerCommandInput.cast(inputs.itemById("z2"))
                q_inp = adsk.core.FloatSpinnerCommandInput.cast(inputs.itemById("q"))
                x2_inp = adsk.core.FloatSpinnerCommandInput.cast(inputs.itemById("x2"))
                a_inp = adsk.core.ValueCommandInput.cast(inputs.itemById("custom_a"))
                if m_inp and z1_inp and z2_inp and q_inp:
                    m_val = m_inp.value * 10.0
                    z1_val = z1_inp.value
                    z2_val = z2_inp.value
                    q_val = q_inp.value
                    # 若当前处于「指定中心距」模式，则用 a 反求 x2，否则直接读 x2
                    a_mode = False
                    dist_item = adsk.core.DropDownCommandInput.cast(inputs.itemById("dist_mode"))
                    if dist_item and dist_item.selectedItem and "指定" in dist_item.selectedItem.name:
                        a_mode = True
                    if a_mode:
                        x2_val = None
                        a_val = a_inp.value * 10.0 if a_inp else None
                    else:
                        x2_val = x2_inp.value if x2_inp else 0.0
                        a_val = None
                    if m_val > 0 and q_val > 0:
                        w_math, _, _ = reload_submodules()
                        temp_calc = w_math.WormGearMath(
                            m=m_val, z1=z1_val, z2=z2_val, q=q_val,
                            x2=x2_val, a=a_val
                        )
                        temp_res = temp_calc.to_dict()
                        b1_rec = temp_res["worm_length_b1"]
                        b2_rec = temp_res["wheel_face_width_b2"]

                        b1_item = adsk.core.ValueCommandInput.cast(inputs.itemById("b1"))
                        b2_item = adsk.core.ValueCommandInput.cast(inputs.itemById("b2"))
                        ch_item = adsk.core.ValueCommandInput.cast(inputs.itemById("wheel_chamfer"))
                        if b1_item:
                            b1_item.expression = f"{b1_rec:.1f} mm"
                        if b2_item:
                            b2_item.expression = f"{b2_rec:.1f} mm"
                        if ch_item:
                            ch_item.expression = f"{0.05 * b2_rec:.2f} mm"

                        custom_a_item = adsk.core.ValueCommandInput.cast(inputs.itemById("custom_a"))
                        if custom_a_item and not custom_a_item.isVisible:
                            custom_a_item.expression = f"{temp_res['center_distance_a']:.2f} mm"

            if changed_id == "b2":
                b2_item = adsk.core.ValueCommandInput.cast(inputs.itemById("b2"))
                ch_item = adsk.core.ValueCommandInput.cast(inputs.itemById("wheel_chamfer"))
                if b2_item and ch_item:
                    b2_val = b2_item.value * 10.0
                    if b2_val > 0:
                        ch_item.expression = f"{0.05 * b2_val:.2f} mm"

            update_calculated_display(inputs)
        except Exception:
            pass


class WormGearValidateInputsHandler(adsk.core.ValidateInputsEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args: adsk.core.ValidateInputsEventArgs):
        try:
            inputs = args.inputs
            m_input = adsk.core.ValueCommandInput.cast(inputs.itemById("module"))
            q_input = adsk.core.FloatSpinnerCommandInput.cast(inputs.itemById("q"))
            z2_input = adsk.core.IntegerSpinnerCommandInput.cast(inputs.itemById("z2"))
            z1_input = adsk.core.IntegerSpinnerCommandInput.cast(inputs.itemById("z1"))

            # 注意：只做 spinner 范围之外仍需拦截的检查，避免出现不可达分支。
            if m_input and m_input.value <= 0:
                args.areInputsValid = False
                return
            if q_input and q_input.value <= 2.4:
                args.areInputsValid = False
                return
            if z2_input and z2_input.value < 17:
                args.areInputsValid = False
                return
            if z1_input and not (1 <= z1_input.value <= 6):
                args.areInputsValid = False
                return

            args.areInputsValid = True
        except Exception:
            args.areInputsValid = False


class WormGearCommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args: adsk.core.CommandEventArgs):
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            inputs = args.command.commandInputs

            # 提取基础参数 (Fusion 内部单位转换: length.value 是 cm，乘以 10 得到 mm)
            m_mm = adsk.core.ValueCommandInput.cast(inputs.itemById("module")).value * 10.0
            z1 = adsk.core.IntegerSpinnerCommandInput.cast(inputs.itemById("z1")).value
            z2 = adsk.core.IntegerSpinnerCommandInput.cast(inputs.itemById("z2")).value
            q = adsk.core.FloatSpinnerCommandInput.cast(inputs.itemById("q")).value
            alpha_deg = math.degrees(adsk.core.ValueCommandInput.cast(inputs.itemById("alpha")).value)

            target_item = adsk.core.DropDownCommandInput.cast(inputs.itemById("target_comp"))
            target_mode = target_item.selectedItem.name if (target_item and target_item.selectedItem) else "装配体"

            prof_item = adsk.core.DropDownCommandInput.cast(inputs.itemById("worm_type"))
            worm_type = "ZN" if (prof_item and prof_item.selectedItem and "ZN" in prof_item.selectedItem.name) else "ZA"

            dir_item = adsk.core.DropDownCommandInput.cast(inputs.itemById("direction"))
            direction = "Left" if (dir_item and dir_item.selectedItem and "左旋" in dir_item.selectedItem.name) else "Right"

            qual_item = adsk.core.DropDownCommandInput.cast(inputs.itemById("quality"))
            qual_name = qual_item.selectedItem.name if (qual_item and qual_item.selectedItem) else ""
            # 按关键字映射（不依赖数字，避免措辞微调后误判）
            if "超精" in qual_name:
                quality = "ultra"
            elif "高精" in qual_name:
                quality = "fine"
            elif "快速" in qual_name:
                quality = "fast"
            else:
                quality = "standard"

            dist_item = adsk.core.DropDownCommandInput.cast(inputs.itemById("dist_mode"))
            dist_mode = dist_item.selectedItem.name if (dist_item and dist_item.selectedItem) else ""
            if "指定" in dist_mode:
                custom_a_inp = adsk.core.ValueCommandInput.cast(inputs.itemById("custom_a"))
                a_target = custom_a_inp.value * 10.0 if custom_a_inp else None
                x2_val = None
            else:
                x2_inp = adsk.core.FloatSpinnerCommandInput.cast(inputs.itemById("x2"))
                x2_val = x2_inp.value if x2_inp else 0.0
                a_target = None

            b1_item = inputs.itemById("b1")
            b2_item = inputs.itemById("b2")
            b1_mm = adsk.core.ValueCommandInput.cast(b1_item).value * 10.0 if b1_item else None
            b2_mm = adsk.core.ValueCommandInput.cast(b2_item).value * 10.0 if b2_item else None

            backlash_item = inputs.itemById("backlash")
            chamfer_item = inputs.itemById("wheel_chamfer")
            fillet_item = inputs.itemById("worm_fillet")
            backlash_val = adsk.core.ValueCommandInput.cast(backlash_item).value * 10.0 if backlash_item else 0.0
            chamfer_val = adsk.core.ValueCommandInput.cast(chamfer_item).value * 10.0 if chamfer_item else None
            fillet_val = adsk.core.ValueCommandInput.cast(fillet_item).value * 10.0 if fillet_item else 0.0

            # 强制重载当前工作区的子模块，杜绝常驻进程缓存旧代码或旧路径
            w_math, w_builder, g_builder = reload_submodules()

            # 运行核心数学求解
            gear_math = w_math.WormGearMath(
                m=m_mm, z1=z1, z2=z2, q=q,
                x2=x2_val, a=a_target,
                alpha_deg=alpha_deg,
                worm_type=worm_type,
                direction=direction,
                b1=b1_mm,
                b2=b2_mm,
                backlash=backlash_val,
                wheel_chamfer=chamfer_val,
                worm_fillet=fillet_val
            )
            params = gear_math.to_dict()

            # 生成前先做完整校核：旧的 validate() 只返回错误列表，从不阻断建模，
            # 结果用户能拿到"生成成功"提示，但实体几何本身是非法/自交的。
            is_valid, v_errors, v_warnings = gear_math.validate()
            if not is_valid:
                ui.messageBox(
                    "参数校核未通过，已中止生成：\n\n"
                    + "\n".join(f"· {e}" for e in v_errors)
                    + ("\n\n提示:\n" + "\n".join(f"· {w}" for w in v_warnings) if v_warnings else "")
                )
                return
            if v_warnings:
                ui.messageBox(
                    "参数提示 (可继续生成)：\n\n" + "\n".join(f"· {w}" for w in v_warnings)
                )

            # 获取当前设计环境
            design = adsk.fusion.Design.cast(app.activeProduct)
            if not design:
                ui.messageBox("请在 Design 建模环境下运行此功能！")
                return

            root_comp = design.rootComponent

            # 根据用户选择生成装配体或单个零件
            if "装配体" in target_mode:
                # 1. 蜗轮组件位于全局坐标系中心 (纯实心蜗轮，无轴孔)
                wheel_occ = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                wheel_comp = wheel_occ.component
                wheel_comp.name = f"蜗轮_m{m_mm}_z{z2}"
                g_builder.build_worm_wheel(wheel_comp, params, quality)

                # 2. 蜗杆组件直接以正交变换矩阵实例化在中心距 a 处 (从创建伊始即位于正确啮合位置，绝对杜绝位于原点)
                a_cm = params["center_distance_a"] * 0.1

                # ---- 蜗杆装配姿态 ----
                # 1) 轴线方向：蜗杆与蜗轮轴线必须空间交错垂直。
                #      蜗轮轴线 = 全局 Z 轴 (轮坯在建模坐标系中绕 Z 回转)
                #      蜗杆轴线 = 全局 Y 轴 (蜗轮建模时蜗杆轴就取 Y 轴)
                #      中心距方向 = 全局 X 轴 (蜗杆位于 x = a 处)
                #    故取右手正交基底 X_worm=+X, Y_worm=+Z, Z_worm=+Y (绕 X 轴 +90°)。
                #    旧实现把"螺旋相位" theta1_0 = pi/z1 当成**轴线方向**使用：
                #    axis = (-sin t, 0, cos t)，对 z1=2 (t=90°) 该方向为 -X，
                #    与中心距方向重合、与蜗轮轴线平行，属退化装配；且旧基底第三轴
                #    (0,1,0) 与该两轴张成的平面正交，行列式为 0，不是合法旋转矩阵。
                # 2) 螺旋相位：蜗轮齿槽在 z=0 截面上位于 θ=0，而蜗杆螺旋线在 z=0 处的
                #    相位为 pi/z1 (等效轴向偏移 pz/2)，需绕蜗杆自身轴线补上该相位。
                #
                # 注意：这里必须**直接算出最终正交基底的四列**再一次性建矩阵。
                # 不能用 Matrix3D.transformBy() 去复合旋转——复合结果会被 Fusion 的
                # addNewComponent 判为 "invalid argument transform"。
                #
                # 三列必须同时满足：单位正交、右手系 (ax × ay = az)、且
                # az (蜗杆自身轴线) = 全局 +Y。写法稍有偏差就会得到 det = -1 的
                # 反射矩阵或非正交矩阵，Fusion 同样会报 invalid argument transform。
                #
                # 下面这组已对 z1 = 1~6 全部逐一验证：
                #   单位正交 ✓   ax × ay = az ✓   det = +1 ✓   az = (0,1,0) ✓
                theta1_0 = math.pi / float(z1)
                c_spin = math.cos(theta1_0)
                s_spin = math.sin(theta1_0)

                ax = adsk.core.Vector3D.create(c_spin, 0.0, s_spin)       # 蜗杆局部 X -> 全局
                ay = adsk.core.Vector3D.create(s_spin, 0.0, -c_spin)      # 蜗杆局部 Y -> 全局
                az = adsk.core.Vector3D.create(0.0, 1.0, 0.0)             # 蜗杆局部 Z -> 全局 (轴线)

                mat_trans = adsk.core.Matrix3D.create()
                ok = mat_trans.setWithCoordinateSystem(
                    adsk.core.Point3D.create(a_cm, 0.0, 0.0), ax, ay, az
                )
                if not ok:
                    raise RuntimeError("构造蜗杆装配变换矩阵失败 (setWithCoordinateSystem 返回 False)。")

                worm_occ = root_comp.occurrences.addNewComponent(mat_trans)
                worm_comp = worm_occ.component
                worm_comp.name = f"蜗杆_m{m_mm}_z{z1}"
                w_builder.build_worm(worm_comp, params)

                # =============================================================
                # 3. 创建 Fusion 360 原生标准旋转副 (Revolute Joint) 与啮合运动链接 (Motion Link)
                # 基于 Autodesk 官方标准 JointOrigin (联接原点) 体系，严谨构建装配约束
                # 彻底消除多余机架组件，杜绝原点重合导致的原点复位与视角混乱
                # =============================================================
                try:
                    joints = root_comp.joints
                    motion_links = root_comp.motionLinks

                    # 1) 蜗轮旋转副：
                    # 在 wheel_comp 中建立回转中心联接原点 (绕自身 Z 轴)
                    # 在 root_comp 中建立全局原点承接原点 (绕全局 Z 轴)
                    geo_wheel = adsk.fusion.JointGeometry.createByPoint(wheel_comp.originConstructionPoint)
                    jo_wheel_in = wheel_comp.jointOrigins.createInput(geo_wheel)
                    jo_wheel_in.zAxisEntity = wheel_comp.zConstructionAxis
                    jo_wheel_in.xAxisEntity = wheel_comp.xConstructionAxis
                    jo_wheel = wheel_comp.jointOrigins.add(jo_wheel_in)
                    jo_wheel.name = "蜗轮回转基准"
                    jo_wheel.isLightBulbOn = False

                    geo_root_wheel = adsk.fusion.JointGeometry.createByPoint(root_comp.originConstructionPoint)
                    jo_root_wheel_in = root_comp.jointOrigins.createInput(geo_root_wheel)
                    jo_root_wheel_in.zAxisEntity = root_comp.zConstructionAxis
                    jo_root_wheel_in.xAxisEntity = root_comp.xConstructionAxis
                    jo_root_wheel = root_comp.jointOrigins.add(jo_root_wheel_in)
                    jo_root_wheel.name = "蜗轮装配轴心"
                    jo_root_wheel.isLightBulbOn = False

                    jo_wheel_proxy = jo_wheel.createForAssemblyContext(wheel_occ)
                    wheel_j_in = joints.createInput(jo_wheel_proxy, jo_root_wheel)
                    wheel_j_in.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
                    wheel_joint = joints.add(wheel_j_in)
                    wheel_joint.name = "蜗轮旋转联接"

                    # 2) 蜗杆旋转副：
                    # 在 worm_comp 中建立回转中心联接原点 (绕自身圆柱 Z 轴)
                    # 在 root_comp 中建立位于中心距 X=a 处、以全局 Y 轴为回转轴的装配承接原点
                    geo_worm = adsk.fusion.JointGeometry.createByPoint(worm_comp.originConstructionPoint)
                    jo_worm_in = worm_comp.jointOrigins.createInput(geo_worm)
                    jo_worm_in.zAxisEntity = worm_comp.zConstructionAxis
                    jo_worm_in.xAxisEntity = worm_comp.xConstructionAxis
                    jo_worm = worm_comp.jointOrigins.add(jo_worm_in)
                    jo_worm.name = "蜗杆回转基准"
                    jo_worm.isLightBulbOn = False

                    geo_root_worm = adsk.fusion.JointGeometry.createByPoint(root_comp.originConstructionPoint)
                    jo_root_worm_in = root_comp.jointOrigins.createInput(geo_root_worm)
                    jo_root_worm_in.offsetX = adsk.core.ValueInput.createByReal(a_cm)
                    jo_root_worm_in.zAxisEntity = root_comp.yConstructionAxis
                    jo_root_worm_in.xAxisEntity = root_comp.xConstructionAxis
                    # 联接原点的初始转角取 0：蜗杆的螺旋相位已经在装配矩阵里通过
                    # 绕自身轴线的 theta1_0 旋转体现；这里若再转一次等于重复补偿相位，
                    # 会让蜗杆螺纹与蜗轮齿槽在装配状态下错开半个齿距。
                    jo_root_worm_in.angle = adsk.core.ValueInput.createByReal(0.0)
                    jo_root_worm = root_comp.jointOrigins.add(jo_root_worm_in)
                    jo_root_worm.name = "蜗杆装配轴心"
                    jo_root_worm.isLightBulbOn = False

                    jo_worm_proxy = jo_worm.createForAssemblyContext(worm_occ)
                    worm_j_in = joints.createInput(jo_worm_proxy, jo_root_worm)
                    worm_j_in.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
                    worm_joint = joints.add(worm_j_in)
                    worm_joint.name = "蜗杆旋转联接"

                    # 3) 传动比运动链接绑定 (Motion Link)
                    # 运动关系：蜗杆自转 360 度，蜗轮旋转 (360 * z1 / z2) 度
                    #
                    # 转向符号：Fusion 的 MotionLink 只比较两个关节各自**围绕自己轴线**的
                    # 转角，两个轴线不同(蜗杆 +Y / 蜗轮 +Z)，所以符号不能靠约定推出来，
                    # 必须由啮合几何决定。判据取蜗轮齿槽的实际螺旋方向：
                    #   dθ_wheel/dz > 0 (右旋) -> 需要反向；< 0 (左旋) -> 不需要反向。
                    # 这个斜率由 params["direction"] 唯一决定，因此等价于：
                    #   isReversed = (direction == "Right")
                    # 注意：该符号与蜗杆装配姿态绑定。若将来再次改动上面那个装配基底
                    # (mat_trans 的三个向量)，蜗杆自身轴向会翻转，这里的符号也必须一起翻。
                    # 可用 test_math.py 中 TestSlotHandedness 的断言来核对。
                    ml_input = motion_links.createInput(worm_joint, wheel_joint)
                    ml_input.valueOne = adsk.core.ValueInput.createByString("360 deg")
                    ml_input.valueTwo = adsk.core.ValueInput.createByString(f"{(360.0 * z1 / float(z2)):.4f} deg")
                    ml_input.isReversed = (direction == "Right")
                    ml_link = motion_links.add(ml_input)
                    ml_link.name = f"蜗轮蜗杆传动副_{params['ratio_i']:.1f}比1"
                except Exception as e_joint:
                    ui.messageBox(f"装配旋转副绑定提示:\n{traceback.format_exc()}")

                ui.messageBox(
                    f"蜗轮蜗杆装配体生成成功！\n\n"
                    f"中心距 a: {params['center_distance_a']:.2f} mm\n"
                    f"传动比 i: {params['ratio_i']:.1f}:1 ({z2}/{z1})\n"
                    f"导程角 γ: {params['lead_angle_gamma_deg']:.2f}°\n"
                    f"旋向: {'右旋' if direction == 'Right' else '左旋'}\n"
                    f"自锁状态: {params['self_locking_status']}\n\n"
                    f"已创建原生旋转副与运动链接：蜗杆自转 360°，蜗轮转 "
                    f"{360.0 * z1 / float(z2):.4f}°。\n"
                    f"可直接拖拽或右键运动链接播放动画。\n"
                    f"若观察到的转向与实际不符，请告知旋向与转向，便于校准符号。"
                )

            elif "仅蜗杆" in target_mode:
                worm_occ = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                worm_comp = worm_occ.component
                worm_comp.name = f"蜗杆_m{m_mm}_z{z1}"
                w_builder.build_worm(worm_comp, params)
                ui.messageBox(
                    f"蜗杆生成成功！\n\n"
                    f"顶圆直径 da1: {params['worm_tip_diameter_da1']:.2f} mm\n"
                    f"螺纹长度 b1: {params['worm_length_b1']:.1f} mm\n"
                    f"导程角 γ: {params['lead_angle_gamma_deg']:.2f}°"
                )

            else:
                wheel_occ = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                wheel_comp = wheel_occ.component
                wheel_comp.name = f"蜗轮_m{m_mm}_z{z2}"
                g_builder.build_worm_wheel(wheel_comp, params, quality)
                ui.messageBox(
                    f"蜗轮生成成功！\n\n"
                    f"喉圆直径 da2: {params['wheel_throat_diameter_da2']:.2f} mm\n"
                    f"齿宽 b2: {params['wheel_face_width_b2']:.1f} mm\n"
                    f"齿数 z2: {z2}"
                )

        except Exception:
            ui.messageBox(f"生成过程出错:\n{traceback.format_exc()}")


class WormGearCommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self, created_handler=None):
        super().__init__()
        self._created = created_handler

    def notify(self, args: adsk.core.CommandEventArgs):
        """命令销毁时回收本批监听器，防止全局 handlers 列表随使用次数无限增长。"""
        created = self._created
        if created is None:
            return
        try:
            for h in list(getattr(created, "_batch", []) or []):
                try:
                    if h in handlers:
                        handlers.remove(h)
                except Exception:
                    pass
            created._batch = []
        except Exception:
            pass


def update_calculated_display(inputs: adsk.core.CommandInputs):
    """提取当前界面参数，调用 WormGearMath 求解并刷新实时 HTML 报告。

    失败时不再静默吞掉：把原因写回报告框，用户至少能看到"为什么没刷新"。
    """
    global _last_calc_error
    try:
        m_inp = adsk.core.ValueCommandInput.cast(inputs.itemById("module"))
        z1_inp = adsk.core.IntegerSpinnerCommandInput.cast(inputs.itemById("z1"))
        z2_inp = adsk.core.IntegerSpinnerCommandInput.cast(inputs.itemById("z2"))
        q_inp = adsk.core.FloatSpinnerCommandInput.cast(inputs.itemById("q"))
        alpha_inp = adsk.core.ValueCommandInput.cast(inputs.itemById("alpha"))
        calc_box = adsk.core.TextBoxCommandInput.cast(inputs.itemById("calc_display"))

        if not (m_inp and z1_inp and z2_inp and q_inp and alpha_inp and calc_box):
            return

        m_mm = m_inp.value * 10.0
        z1 = z1_inp.value
        z2 = z2_inp.value
        q = q_inp.value
        alpha_deg = math.degrees(alpha_inp.value)

        dist_mode_item = adsk.core.DropDownCommandInput.cast(inputs.itemById("dist_mode"))
        dist_mode = dist_mode_item.selectedItem.name if (dist_mode_item and dist_mode_item.selectedItem) else ""
        if "指定" in dist_mode:
            custom_a_inp = adsk.core.ValueCommandInput.cast(inputs.itemById("custom_a"))
            a_val = custom_a_inp.value * 10.0 if custom_a_inp else None
            x2_val = None
        else:
            x2_inp = adsk.core.FloatSpinnerCommandInput.cast(inputs.itemById("x2"))
            x2_val = x2_inp.value if x2_inp else 0.0
            a_val = None

        prof_item = adsk.core.DropDownCommandInput.cast(inputs.itemById("worm_type"))
        worm_type = "ZN" if (prof_item and prof_item.selectedItem and "ZN" in prof_item.selectedItem.name) else "ZA"

        dir_item = adsk.core.DropDownCommandInput.cast(inputs.itemById("direction"))
        direction = "Left" if (dir_item and dir_item.selectedItem and "左旋" in dir_item.selectedItem.name) else "Right"

        b1_inp = adsk.core.ValueCommandInput.cast(inputs.itemById("b1"))
        b2_inp = adsk.core.ValueCommandInput.cast(inputs.itemById("b2"))
        b1_val = b1_inp.value * 10.0 if b1_inp else None
        b2_val = b2_inp.value * 10.0 if b2_inp else None

        backlash_inp = adsk.core.ValueCommandInput.cast(inputs.itemById("backlash"))
        chamfer_inp = adsk.core.ValueCommandInput.cast(inputs.itemById("wheel_chamfer"))
        fillet_inp = adsk.core.ValueCommandInput.cast(inputs.itemById("worm_fillet"))
        backlash_val = backlash_inp.value * 10.0 if backlash_inp else 0.0
        chamfer_val = chamfer_inp.value * 10.0 if chamfer_inp else None
        fillet_val = fillet_inp.value * 10.0 if fillet_inp else 0.0

        if m_mm <= 0 or q <= 2.4:
            calc_box.formattedText = "<font color='red'>参数错误：模数必须为正数，且直径系数 q 必须大于 2.4</font>"
            return

        w_math, _, _ = reload_submodules()
        gear = w_math.WormGearMath(
            m=m_mm, z1=z1, z2=z2, q=q,
            x2=x2_val, a=a_val,
            alpha_deg=alpha_deg,
            worm_type=worm_type,
            direction=direction,
            b1=b1_val,
            b2=b2_val,
            backlash=backlash_val,
            wheel_chamfer=chamfer_val,
            worm_fillet=fillet_val
        )
        calc_box.formattedText = gear.summary_html()
        _last_calc_error = ""

    except Exception as exc:
        # 不再静默吞掉异常：把原因写入报告框，用户能立刻看到问题所在。
        _last_calc_error = traceback.format_exc()
        try:
            calc_box = adsk.core.TextBoxCommandInput.cast(inputs.itemById("calc_display"))
            if calc_box:
                calc_box.formattedText = (
                    "<font color='red'><b>实时计算失败:</b> "
                    f"{type(exc).__name__}: {exc}</font><br>"
                    "<font color='#cc8800'>请检查输入的参数是否合法（数值范围、单位是否正确）。</font>"
                )
        except Exception:
            pass


def run(context):
    """插件启动入口：注册 UI 按钮，并在手动启动时立即激活命令"""
    ui = None
    try:
        reload_submodules()
        app = adsk.core.Application.get()
        ui = app.userInterface

        # 清除可能残留的旧命令定义
        existing_cmd = ui.commandDefinitions.itemById(COMMAND_ID)
        if existing_cmd:
            existing_cmd.deleteMe()

        # 创建命令定义 (使用 resources 绝对路径)
        res_folder = os.path.join(current_dir, "resources")
        cmd_def = ui.commandDefinitions.addButtonDefinition(
            COMMAND_ID, COMMAND_NAME, COMMAND_DESCRIPTION, res_folder
        )

        on_created = WormGearCommandCreatedHandler()
        cmd_def.commandCreated.add(on_created)
        handlers.append(on_created)

        # 添加到 Solid -> Create 面板并常驻推广至工具栏
        # 添加到 Solid -> Create 面板并常驻推广至工具栏
        # 全版本兼容检索：优先使用 allToolbarPanels，回退至 toolbarTabs 与 workspace
        create_panel = ui.allToolbarPanels.itemById(PANEL_ID)
        if not create_panel:
            solid_ws = ui.workspaces.itemById(WORKSPACE_ID)
            if solid_ws:
                solid_tab = solid_ws.toolbarTabs.itemById("SolidTab")
                if solid_tab:
                    create_panel = solid_tab.toolbarPanels.itemById(PANEL_ID)
                if not create_panel:
                    create_panel = solid_ws.toolbarPanels.itemById(PANEL_ID)

        if create_panel:
            existing_ctrl = create_panel.controls.itemById(COMMAND_ID)
            if existing_ctrl:
                existing_ctrl.deleteMe()

            ctrl = create_panel.controls.addCommand(cmd_def)
            ctrl.isPromoted = True
            ctrl.isPromotedByDefault = True
            ctrl.isVisible = True

        # 核心关键：当用户在【脚本和附加模块】对话框中手动点击【运行】时，
        # 立即自动弹出参数生成器对话框，无需用户再去复杂菜单寻找按钮！
        is_startup = False
        if isinstance(context, dict):
            is_startup = context.get("IsApplicationStartup", False)

        if not is_startup:
            # 确保当前有打开的建模设计文档
            if app.activeProduct is None or not isinstance(app.activeProduct, adsk.fusion.Design):
                app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
            # 立即激活命令，弹出参数界面！
            cmd_def.execute()

    except Exception:
        if ui:
            ui.messageBox(f"启动插件失败:\n{traceback.format_exc()}")


def stop(context):
    """插件卸载入口：清理 UI 控件"""
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        create_panel = ui.allToolbarPanels.itemById(PANEL_ID)
        if not create_panel:
            solid_ws = ui.workspaces.itemById(WORKSPACE_ID)
            if solid_ws:
                solid_tab = solid_ws.toolbarTabs.itemById("SolidTab")
                if solid_tab:
                    create_panel = solid_tab.toolbarPanels.itemById(PANEL_ID)
                if not create_panel:
                    create_panel = solid_ws.toolbarPanels.itemById(PANEL_ID)

        if create_panel:
            ctrl = create_panel.controls.itemById(COMMAND_ID)
            if ctrl:
                ctrl.deleteMe()

        cmd_def = ui.commandDefinitions.itemById(COMMAND_ID)
        if cmd_def:
            cmd_def.deleteMe()

        global handlers
        handlers.clear()

        # 清除模块缓存，保证下次启动时重新加载最新文件
        for mod_name in ["WormGearGenerator", "gear_builder", "worm_builder", "worm_math"]:
            if mod_name in sys.modules:
                del sys.modules[mod_name]

    except Exception:
        if ui:
            ui.messageBox(f"卸载插件失败:\n{traceback.format_exc()}")
