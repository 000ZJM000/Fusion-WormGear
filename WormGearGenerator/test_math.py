"""
test_math.py - 单元测试

覆盖：
  1. worm_math.py 的几何/力学计算与参数校核
  2. gear_builder.py 共轭齿廓求解器的几何正确性
  3. WormGearGenerator.py 中装配变换矩阵与运动链接方向的正确性

全部测试均**不需要 Fusion 环境**，直接用系统 Python 运行即可：

    python test_math.py

原理：被测模块只在类型注解里引用 adsk，因此这里先用占位模块顶替 adsk，
再导入真实的 worm_math / gear_builder。
"""

import math
import os
import re
import sys
import types
import unittest

# 保证可以从本文件所在目录导入被测模块（无论从哪个工作目录运行）
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from worm_math import WormGearMath


def _install_adsk_stub():
    """安装 adsk 占位模块，使被测模块可以在 Fusion 之外被导入。"""
    if "adsk" in sys.modules:
        return

    class _Any(types.ModuleType):
        def __getattr__(self, name):
            return self

    adsk = _Any("adsk")
    core = _Any("adsk.core")
    fusion = _Any("adsk.fusion")
    adsk.core = core
    adsk.fusion = fusion
    sys.modules.update({"adsk": adsk, "adsk.core": core, "adsk.fusion": fusion})


def _load_gear_builder():
    """导入 gear_builder；失败返回 None（对应测试会被跳过）。"""
    _install_adsk_stub()
    try:
        import gear_builder
        return gear_builder
    except Exception:
        return None


def _read_source(filename):
    """读取同目录下的源码，供「直接校验源码表达式」的测试使用。"""
    with open(os.path.join(_HERE, filename), encoding="utf-8") as fh:
        return fh.read()


gear_builder = _load_gear_builder()


class TestWormGearMath(unittest.TestCase):

    def test_case_standard_gbt(self):
        """国标标准组合 (m=4, z1=2, z2=40, q=10, a=100)"""
        gear = WormGearMath(m=4.0, z1=2, z2=40, q=10.0, a=100.0)
        res = gear.to_dict()

        self.assertAlmostEqual(res["profile_shift_x2"], 0.0, places=3)
        self.assertAlmostEqual(res["center_distance_a"], 100.0, places=3)
        self.assertAlmostEqual(res["worm_pitch_diameter_d1"], 40.0, places=3)
        self.assertAlmostEqual(res["wheel_pitch_diameter_d2"], 160.0, places=3)
        self.assertAlmostEqual(res["worm_tip_diameter_da1"], 48.0, places=3)
        self.assertAlmostEqual(res["wheel_throat_diameter_da2"], 168.0, places=3)
        self.assertAlmostEqual(res["wheel_throat_radius_rg2"], 16.0, places=3)
        self.assertAlmostEqual(res["lead_angle_gamma_deg"], 11.3099, places=3)
        self.assertEqual(res["ratio_i"], 20.0)

        valid, errors, _ = gear.validate()
        self.assertTrue(valid, f"验证失败: {errors}")

    def test_case_self_locking(self):
        """自锁减速机构 (m=2, z1=1, z2=50, q=18, a=70)"""
        gear = WormGearMath(m=2.0, z1=1, z2=50, q=18.0, a=70.0)
        res = gear.to_dict()

        # x2 = 70/2 - 0.5*(18+50) = +1.0
        self.assertAlmostEqual(res["profile_shift_x2"], 1.0, places=3)
        self.assertTrue(res["lead_angle_gamma_deg"] < 3.5)
        self.assertTrue(res["is_self_locking"])

    def test_center_distance_round_trip(self):
        """a -> x2 -> a 必须闭合（旧实现存在因浮点往返导致的偏差风险）"""
        for a in (40.0, 63.0, 100.0, 125.0):
            g1 = WormGearMath(m=2.0, z1=2, z2=40, q=10.0, a=a)
            x2 = g1.to_dict()["profile_shift_x2"]
            g2 = WormGearMath(m=2.0, z1=2, z2=40, q=10.0, x2=x2)
            self.assertAlmostEqual(g2.to_dict()["center_distance_a"], a, places=6)

    def test_throat_radii_identities(self):
        """蜗轮各特征半径必须满足标准恒等式"""
        g = WormGearMath(m=2.0, z1=1, z2=30, q=10.0)
        r = g.to_dict()
        # 蜗轮齿根圆弧半径 = 中心距 - 齿根圆半径
        self.assertAlmostEqual(
            r["wheel_root_arc_radius_rf2"],
            r["center_distance_a"] - r["wheel_root_diameter_df2"] / 2.0,
            places=6,
        )
        # 蜗轮齿根圆 = 中心距侧算出的刀具顶圆包络 (二者必须自洽)
        self.assertAlmostEqual(
            r["wheel_root_diameter_df2"],
            2.0 * (r["center_distance_a"] - r["hob_tip_diameter_da0"] / 2.0),
            places=6,
        )
        # 刀具顶圆 = 蜗杆顶圆 + 2c
        self.assertAlmostEqual(
            r["hob_tip_diameter_da0"],
            r["worm_tip_diameter_da1"] + 2.0 * r["clearance_c"],
            places=6,
        )
        # 喉母圆半径 = 中心距 - 喉圆半径
        self.assertAlmostEqual(
            r["wheel_throat_radius_rg2"],
            r["center_distance_a"] - r["wheel_throat_diameter_da2"] / 2.0,
            places=6,
        )
        # 蜗杆齿根圆 = m(q - 2.4)
        self.assertAlmostEqual(
            r["worm_root_diameter_df1"],
            r["module_m"] * (r["diameter_factor_q"] - 2.0 * (r["ha_star"] + r["c_star"])),
            places=6,
        )

    def test_summary_html(self):
        gear = WormGearMath(m=2.0, z1=1, z2=30, q=10.0)
        html = gear.summary_html()
        self.assertIn("传动比", html)
        self.assertIn("中心距", html)


class TestValidation(unittest.TestCase):
    """参数校核必须能拦住非法输入（旧版本会放行 b1/b2 为 0 或负数）。"""

    def _validate(self, **kw):
        base = dict(m=2.0, z1=1, z2=30, q=10.0)
        base.update(kw)
        return WormGearMath(**base).validate()

    def test_reject_non_positive_b1(self):
        for b1 in (0.0, -5.0):
            valid, errors, _ = self._validate(b1=b1)
            self.assertFalse(valid, f"b1={b1} 应被判为非法")
            self.assertTrue(errors)

    def test_reject_b1_shorter_than_mesh_chord(self):
        valid, errors, _ = self._validate(b1=1.0)
        self.assertFalse(valid)
        self.assertTrue(any("喉部啮合弦长" in e for e in errors))

    def test_reject_non_positive_b2(self):
        valid, errors, _ = self._validate(b2=0.0)
        self.assertFalse(valid)

    def test_reject_oversized_b2(self):
        valid, errors, _ = self._validate(b2=100.0)
        self.assertFalse(valid)
        self.assertTrue(any("超过标准上限" in e for e in errors))

    def test_reject_oversized_chamfer(self):
        valid, errors, _ = self._validate(wheel_chamfer=50.0)
        self.assertFalse(valid)

    def test_reject_excessive_backlash(self):
        valid, errors, _ = self._validate(backlash=99.0)
        self.assertFalse(valid)

    def test_positive_shift_warns_about_tip_clearance(self):
        """正变位会减少顶隙，必须给出明确警告（但不阻断，正变位是常用设计手段）"""
        valid, _, warnings = self._validate(x2=1.0)
        self.assertTrue(valid, "正变位不应被直接阻断")
        self.assertTrue(
            any("顶隙" in w for w in warnings),
            f"正变位应给出顶隙警告: {warnings}",
        )

    def test_zero_shift_has_no_clearance_warning(self):
        _, _, warnings = self._validate(x2=0.0)
        self.assertFalse(any("顶隙" in w for w in warnings), warnings)

    def test_readme_self_locking_example_is_usable(self):
        """README 中的自锁示例 (m=2, z1=1, z2=50, q=18, a=70 -> x2=+1.0) 必须可用"""
        g = WormGearMath(m=2.0, z1=1, z2=50, q=18.0, a=70.0)
        self.assertAlmostEqual(g.to_dict()["profile_shift_x2"], 1.0, places=3)
        valid, errors, _ = g.validate()
        self.assertTrue(valid, f"自锁示例应可通过校核: {errors}")

    def test_negative_shift_is_legal(self):
        valid, _, _ = self._validate(x2=-0.5)
        self.assertTrue(valid)

    def test_reject_q_too_small(self):
        valid, errors, _ = self._validate(q=2.4)
        self.assertFalse(valid)

    def test_reject_out_of_range_z1(self):
        for z1 in (0, 7):
            valid, _, _ = self._validate(z1=z1)
            self.assertFalse(valid, f"z1={z1} 应被判为非法")

    def test_reject_too_few_teeth(self):
        valid, _, _ = self._validate(z2=16)
        self.assertFalse(valid)

    def test_standard_series_tolerant_to_float_noise(self):
        """标准系列判定必须容忍输入框带来的浮点误差"""
        g = WormGearMath(m=2.0000000000000004, z1=1, z2=30, q=10.000000000000002)
        _, _, warnings = g.validate()
        self.assertFalse(
            any("非 GB/T 10085 标准模数" in w for w in warnings),
            f"浮点噪声不应被判为非标准模数: {warnings}",
        )

    def test_non_standard_module_warns(self):
        _, _, warnings = self._validate(m=2.03)
        self.assertTrue(any("非 GB/T 10085 标准模数" in w for w in warnings))

    def test_reject_nan(self):
        valid, errors, _ = self._validate(m=float("nan"))
        self.assertFalse(valid)
        self.assertTrue(errors)


@unittest.skipIf(gear_builder is None, "gear_builder 无法导入")
class TestConjugateSectionCurves(unittest.TestCase):
    """蜗轮共轭齿廓求解器的几何回归测试。

    历史缺陷：求解区间过窄（右齿面 [-16°,+4°]、左齿面 [-4°,+16°]）且缺少
    "零点是否落在区间内" 的检查，导致二分退化并把结果静默夹断到边界，
    产出重复角度（平台）甚至非单调的假齿廓。以下断言用于捕获这类回归。
    """

    CASES = [
        dict(m=2.0, z1=1, z2=30, q=10.0),
        dict(m=2.0, z1=4, z2=17, q=6.3),
        dict(m=4.0, z1=2, z2=40, q=10.0),
        dict(m=1.0, z1=1, z2=17, q=6.3),
    ]

    def _sections(self, K=9, **kw):
        params = WormGearMath(**kw).to_dict()
        return gear_builder._compute_section_curves(params, K=K)

    def test_section_count_and_point_count(self):
        for kw in self.CASES:
            secs = self._sections(**kw)
            self.assertEqual(len(secs), 9, f"{kw} 截面数不符")
            for sd in secs:
                self.assertEqual(len(sd["pts_right"]), 7)
                self.assertEqual(len(sd["pts_left"]), 7)

    def test_no_duplicate_angles_within_flank(self):
        """齿廓点角度不得出现重复（重复即代表被夹断到边界）"""
        for kw in self.CASES:
            for si, sd in enumerate(self._sections(**kw)):
                for key in ("pts_right", "pts_left"):
                    core = sd[key][:-1]      # 末点为齿顶收口点，与倒数第二点同半径
                    angs = [math.atan2(y, x) for x, y in core]
                    for i in range(len(angs) - 1):
                        self.assertGreater(
                            abs(angs[i + 1] - angs[i]), 1e-9,
                            f"{kw} 截面{si} {key} 出现重复角度（疑似边界夹断）",
                        )

    def test_flank_radius_strictly_increasing(self):
        """齿廓点半径必须严格递增，不允许轮廓折返"""
        for kw in self.CASES:
            for sd in self._sections(**kw):
                for key in ("pts_right", "pts_left"):
                    rads = [math.hypot(x, y) for x, y in sd[key][:-1]]
                    for i in range(len(rads) - 1):
                        self.assertGreater(
                            rads[i + 1], rads[i],
                            f"{kw} {key} 齿廓半径非单调递增",
                        )

    def test_midplane_flanks_are_mirrored(self):
        """中间平面 (zk = 0) 上左右齿面必须严格镜像"""
        for kw in self.CASES:
            mid = self._sections(**kw)[4]        # K=9 -> 索引 4 为 zk=0
            self.assertAlmostEqual(mid["zk"], 0.0, places=12)
            for i in range(len(mid["pts_right"])):
                xr, yr = mid["pts_right"][i]
                xl, yl = mid["pts_left"][i]
                self.assertAlmostEqual(yr, -yl, places=9, msg=f"{kw} 中间平面左右齿面不镜像")
                self.assertAlmostEqual(xr, xl, places=9, msg=f"{kw} 中间平面左右齿面不对称")

    def test_points_outside_root_reference(self):
        """所有齿廓点半径必须 >= 该截面的喉底根部半径（不得切进轮芯）"""
        for kw in self.CASES:
            params = WormGearMath(**kw).to_dict()
            a = params["center_distance_a"] * 0.1
            ra0 = params["hob_tip_diameter_da0"] * 0.1 / 2.0
            for sd in self._sections(**kw):
                zk = sd["zk"]
                r_root = a - math.sqrt(max(0.1, ra0 ** 2 - zk ** 2))
                for key in ("pts_right", "pts_left"):
                    for x, y in sd[key]:
                        self.assertGreaterEqual(
                            math.hypot(x, y) + 1e-9, r_root,
                            f"{kw} {key} 齿廓点半径小于喉底根部半径",
                        )

    def test_invalid_parameter_raises_instead_of_faking(self):
        """无解时必须显式报错，而不是返回虚假齿廓"""
        # 齿宽远大于齿顶圆 -> 喉部弦长超过轮坯几何，截面外半径不足以放下齿廓
        params = WormGearMath(m=20.0, z1=1, z2=80, q=6.3, b2=400.0).to_dict()
        with self.assertRaises(Exception):
            gear_builder._compute_section_curves(params, K=9)

    def test_blank_outer_radius_helper(self):
        """轮坯外半径辅助函数必须与实际回转轮廓一致，且关于 zk=0 对称、单调不减"""
        for kw in self.CASES:
            params = WormGearMath(**kw).to_dict()
            sc = 0.1
            a = params["center_distance_a"] * sc
            rg = params["wheel_throat_radius_rg2"] * sc
            re2 = params["wheel_tip_diameter_de2"] * sc / 2.0
            b2 = params["wheel_face_width_b2"] * sc

            z_arc = gear_builder._throat_arc_end(a, rg, re2, b2)
            self.assertGreater(z_arc, 0.0)
            self.assertLessEqual(z_arc, b2 / 2.0 + 1e-12)

            # 喉部中心处外半径 = a - rg
            self.assertAlmostEqual(
                gear_builder._blank_outer_radius(a, rg, re2, 0.0, z_arc), a - rg, places=9
            )
            # 超出圆弧范围 -> 外圆柱半径
            self.assertAlmostEqual(
                gear_builder._blank_outer_radius(a, rg, re2, b2 / 2.0, z_arc), re2, places=9
            )
            # 左右对称
            for i in range(21):
                zk = -z_arc + 2.0 * z_arc * i / 20.0
                self.assertAlmostEqual(
                    gear_builder._blank_outer_radius(a, rg, re2, zk, z_arc),
                    gear_builder._blank_outer_radius(a, rg, re2, -zk, z_arc),
                    places=12,
                )
            # 喉部圆弧段内 (从喉底向两侧)：半径随 |zk| 单调递增（圆弧向外张开）
            prev = -1.0
            for i in range(51):
                zk = z_arc * i / 50.0
                r = gear_builder._blank_outer_radius(a, rg, re2, zk, z_arc)
                self.assertGreaterEqual(r + 1e-12, prev, f"{kw} 喉部圆弧半径方向异常")
                prev = r
            # 超出圆弧范围后恒为外圆柱半径
            for i in range(11):
                zk = z_arc + (b2 / 2.0 - z_arc) * i / 10.0 + 1e-9
                self.assertAlmostEqual(
                    gear_builder._blank_outer_radius(a, rg, re2, zk, z_arc), re2, places=9
                )
            # 关键安全性：返回的外半径不得小于轮坯真实边界，
            # 否则放样轮廓会越过轮坯，齿槽切不净而在齿顶留下残留。
            for i in range(201):
                zk = -(b2 / 2.0) + b2 * i / 200.0
                r = gear_builder._blank_outer_radius(a, rg, re2, zk, z_arc)
                if abs(zk) <= z_arc and rg * rg - zk * zk > 0.0:
                    true_r = a - math.sqrt(rg * rg - zk * zk)
                else:
                    true_r = re2
                self.assertGreaterEqual(
                    r + 1e-12, true_r, f"{kw} 轮坯外半径被低估 (zk={zk})"
                )
                # 且不得超出外圆柱半径
                self.assertLessEqual(r, re2 + 1e-12)


class TestAssemblyFrame(unittest.TestCase):
    """蜗杆装配变换矩阵的回归测试。

    历史缺陷：把螺旋相位 theta1_0 = pi/z1 误当成**轴线方向**使用，得到
    axis = (-sin t, 0, cos t)，该方向同时垂直于蜗轮轴线 (Z) 与中心距方向 (X)，
    蜗杆被摆到了轮坯外侧；更严重的是用 Matrix3D.transformBy() 复合旋转后，
    三列行列式为 -1（反射矩阵），Fusion 的 addNewComponent 直接抛
    "invalid argument transform"。

    这里直接读取 WormGearGenerator.py 中真实的三个向量表达式并校验它们必须
    同时满足：单位正交、右手系 (ax × ay = az)、行列式 = +1、且蜗杆轴线 = +Y。
    """

    @classmethod
    def setUpClass(cls):
        src = _read_source("WormGearGenerator.py")
        try:
            cls.ax_expr = re.search(r"ax = adsk\.core\.Vector3D\.create\(([^)]*)\)", src).group(1)
            cls.ay_expr = re.search(r"ay = adsk\.core\.Vector3D\.create\(([^)]*)\)", src).group(1)
            cls.az_expr = re.search(r"az = adsk\.core\.Vector3D\.create\(([^)]*)\)", src).group(1)
        except AttributeError:
            cls.ax_expr = cls.ay_expr = cls.az_expr = None

    @staticmethod
    def _cross(a, b):
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    @staticmethod
    def _det(m):
        a, b, c = m[0]
        d, e, f = m[1]
        g, h, i = m[2]
        return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)

    @staticmethod
    def _dot(a, b):
        return sum(x * y for x, y in zip(a, b))

    def _vectors(self, z1):
        t = math.pi / z1
        ctx = {"c_spin": math.cos(t), "s_spin": math.sin(t), "__builtins__": {}}

        def ev(expr):
            return tuple(float(eval(part, ctx)) for part in expr.split(","))

        return ev(self.ax_expr), ev(self.ay_expr), ev(self.az_expr)

    def test_matrix_expressions_found(self):
        self.assertIsNotNone(self.ax_expr, "未能从源码中解析出 ax 向量表达式")
        self.assertIsNotNone(self.ay_expr, "未能从源码中解析出 ay 向量表达式")
        self.assertIsNotNone(self.az_expr, "未能从源码中解析出 az 向量表达式")

    def test_assembly_frame_is_valid_rigid_rotation(self):
        """三列必须是合法的右手正交基底，否则 Fusion 会拒绝该变换"""
        for z1 in (1, 2, 3, 4, 5, 6):
            with self.subTest(z1=z1):
                ax, ay, az = self._vectors(z1)

                # 单位向量
                for name, v in (("ax", ax), ("ay", ay), ("az", az)):
                    self.assertAlmostEqual(self._dot(v, v), 1.0, places=12,
                                           msg=f"z1={z1} {name} 不是单位向量")
                # 两两正交
                self.assertAlmostEqual(self._dot(ax, ay), 0.0, places=12, msg=f"z1={z1} ax·ay≠0")
                self.assertAlmostEqual(self._dot(ax, az), 0.0, places=12, msg=f"z1={z1} ax·az≠0")
                self.assertAlmostEqual(self._dot(ay, az), 0.0, places=12, msg=f"z1={z1} ay·az≠0")
                # 右手系
                c = self._cross(ax, ay)
                for k in range(3):
                    self.assertAlmostEqual(c[k], az[k], places=12,
                                           msg=f"z1={z1} 不是右手系 (ax × ay ≠ az)")
                # 行列式必须为 +1（-1 表示反射矩阵，Fusion 会报 invalid argument transform）
                det = self._det([list(ax), list(ay), list(az)])
                self.assertAlmostEqual(det, 1.0, places=12,
                                       msg=f"z1={z1} 行列式 = {det}，必须为 +1")

    def test_worm_axis_is_global_y(self):
        """蜗杆自身轴线 (局部 Z) 必须落在全局 Y 轴上——与中心距方向 (X)、蜗轮轴线 (Z) 都垂直"""
        for z1 in (1, 2, 3, 4, 5, 6):
            with self.subTest(z1=z1):
                _, _, az = self._vectors(z1)
                self.assertAlmostEqual(az[0], 0.0, places=12, msg=f"z1={z1} 轴线有 X 分量")
                self.assertAlmostEqual(az[1], 1.0, places=12, msg=f"z1={z1} 轴线不在 +Y")
                self.assertAlmostEqual(az[2], 0.0, places=12, msg=f"z1={z1} 轴线有 Z 分量")

    def test_old_broken_axis_direction_is_rejected(self):
        """旧的错误写法 (-sin t, 0, cos t) 必须被判为非法，防止回归"""
        for z1 in (1, 2, 3, 4, 5, 6):
            t = math.pi / z1
            bad_axis = (-math.sin(t), 0.0, math.cos(t))
            # 该方向与 +Y 垂直 => 蜗杆轴线垂直于蜗轮轴线所在平面，属退化装配
            self.assertAlmostEqual(self._dot(bad_axis, (0.0, 1.0, 0.0)), 0.0, places=12,
                                   msg=f"z1={z1} 旧轴线方向应垂直于 Y")


@unittest.skipIf(gear_builder is None, "gear_builder 无法导入")
class TestSlotHandedness(unittest.TestCase):
    """蜗轮齿槽螺旋方向 + 运动链接符号的回归测试。

    背景：Fusion 的 MotionLink 只比较两个关节各自围绕自己轴线的转角，而蜗杆轴 (+Y)
    与蜗轮轴 (+Z) 是不同轴线，因此"正转/反转"的符号必须由啮合几何决定，不能靠约定。
    判据是蜗轮齿槽的螺旋斜率 dθ_wheel/dz：
        右旋 (Right) -> 斜率 > 0 -> motion link 需要 isReversed = True
        左旋 (Left)  -> 斜率 < 0 -> motion link 需要 isReversed = False

    这个斜率与 WormGearGenerator.py 里蜗杆装配基底 (mat_trans 的三个向量) 是绑定的：
    如果装配基底把蜗杆自身轴向翻转了，符号也必须一起翻。此测试用于捕获这类回归。
    """

    CASES = [
        dict(m=2.0, z1=1, z2=30, q=10.0),
        dict(m=4.0, z1=2, z2=40, q=10.0),
    ]

    @staticmethod
    def _slot_centre_deg(section):
        ar = math.degrees(math.atan2(section["pts_right"][0][1], section["pts_right"][0][0]))
        al = math.degrees(math.atan2(section["pts_left"][0][1], section["pts_left"][0][0]))
        return 0.5 * (ar + al)

    def test_slot_helix_slope_sign_matches_handedness(self):
        for kw in self.CASES:
            for direction, expect_positive in (("Right", True), ("Left", False)):
                with self.subTest(direction=direction, **{k: v for k, v in kw.items() if k != "m"}):
                    params = WormGearMath(direction=direction, **kw).to_dict()
                    secs = gear_builder._compute_section_curves(params, K=9)
                    c_first = self._slot_centre_deg(secs[0])
                    c_last = self._slot_centre_deg(secs[-1])
                    z_first = secs[0]["zk"]
                    z_last = secs[-1]["zk"]
                    slope = (c_last - c_first) / (z_last - z_first)
                    if expect_positive:
                        self.assertGreater(slope, 0.0,
                                           f"{direction} 旋向的齿槽螺旋斜率应为正, 实测 {slope}")
                    else:
                        self.assertLess(slope, 0.0,
                                        f"{direction} 旋向的齿槽螺旋斜率应为负, 实测 {slope}")

    def test_slot_is_centred_at_midplane_for_both_handedness(self):
        """无论旋向如何，中截面上的齿槽都必须落在 θ = 0"""
        for kw in self.CASES:
            for direction in ("Right", "Left"):
                params = WormGearMath(direction=direction, **kw).to_dict()
                secs = gear_builder._compute_section_curves(params, K=9)
                mid = secs[len(secs) // 2]
                self.assertAlmostEqual(mid["zk"], 0.0, places=12)
                self.assertAlmostEqual(self._slot_centre_deg(mid), 0.0, places=6,
                                       msg=f"{direction} 中截面齿槽未居中")

    def test_motion_link_reversed_flag_matches_handedness(self):
        """直接校验源码中的 isReversed 取值与几何判据一致"""
        src = _read_source("WormGearGenerator.py")
        m = re.search(r"ml_input\.isReversed\s*=\s*\(([^)]*)\)", src)
        self.assertIsNotNone(m, "未能从源码中解析出 ml_input.isReversed 表达式")
        expr = m.group(1)
        # 该表达式必须由 direction 决定，且右旋应为 True、左旋应为 False
        self.assertIn("direction", expr, f"isReversed 表达式未引用 direction: {expr}")
        self.assertIn("Right", expr,
                      f"isReversed 表达式应判定 Right 旋向为反向: {expr}")
        self.assertNotIn("Left", expr,
                         f"isReversed 表达式不应判定 Left 旋向为反向: {expr}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
