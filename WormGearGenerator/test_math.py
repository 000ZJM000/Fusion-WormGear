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
    """蜗杆装配基底：必须是真旋转，且同时满足两个几何要求。

    历史缺陷（都是本文件作者造成的）：
      * 手写基向量写出过 (0,0,1)/(0,±1,0)/(0,0,-1) 等组合 ——
        有的是反射矩阵（det = -1，Fusion 拒绝或镜像模型），
        有的把蜗杆摆到轮坯外侧，装配看起来完全错位；
      * 旋转次序写反（R_Y·R_X 而非 R_X·R_Y）会让轴线随 ψ 转到 -Z，把蜗杆立起来。

    现在基底由 worm_phase.worm_matrix() 直接相乘得出，不再手写分量。
    """

    @classmethod
    def setUpClass(cls):
        import worm_phase
        cls.wp = worm_phase

    @staticmethod
    def _cross(a, b):
        return (a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0])

    @staticmethod
    def _det(m):
        a, b, c = m[0]
        d, e, f = m[1]
        g, h, i = m[2]
        return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)

    @staticmethod
    def _dot(a, b):
        return sum(x * y for x, y in zip(a, b))

    def test_phase_is_zero_for_odd_starts(self):
        """奇数头蜗杆在基准姿态下实心齿顶已正对全局 -X，基准偏转角为 0°"""
        self.assertAlmostEqual(self.wp.worm_phase_deg(1), 0.0, places=9)
        self.assertAlmostEqual(self.wp.worm_phase_rad(1), 0.0, places=12)
        self.assertAlmostEqual(self.wp.worm_phase_deg(3), 0.0, places=9)

    def test_basis_is_a_proper_rotation(self):
        """三列必须单位正交、右手系、行列式 = +1（-1 表示反射，会把模型镜像）"""
        ax, ay, az = self.wp.worm_basis()
        for name, v in (("ax", ax), ("ay", ay), ("az", az)):
            self.assertAlmostEqual(self._dot(v, v), 1.0, places=12, msg=f"{name} 非单位向量")
        self.assertAlmostEqual(self._dot(ax, ay), 0.0, places=12)
        self.assertAlmostEqual(self._dot(ax, az), 0.0, places=12)
        self.assertAlmostEqual(self._dot(ay, az), 0.0, places=12)
        chk = self._cross(ax, ay)
        for k in range(3):
            self.assertAlmostEqual(chk[k], az[k], places=12, msg="不是右手系 (ax × ay ≠ az)")
        self.assertAlmostEqual(
            self._det([list(ax), list(ay), list(az)]), 1.0, places=12,
            msg="行列式必须为 +1；-1 表示反射矩阵，Fusion 会拒绝或镜像模型"
        )

    def test_thread_crest_points_to_wheel(self):
        """实心螺纹齿顶（局部 θ=180°）必须指向全局 -X —— 蜗轮在蜗杆的 -X 侧。

        蜗杆建模时在 z=0 截面沿局部 +X (θ=0) 切削齿槽，因此局部 +X 是齿槽中心线，
        实心齿顶位于局部 -X (θ=180°)。基准姿态 R_base = R_X(-90°) 将局部 -X
        严格映射到全局 -X，等价于 ax 指向全局 +X (+1, 0, 0)。
        """
        ax, _, _ = self.wp.worm_basis()
        self.assertAlmostEqual(ax[0], 1.0, places=12)
        self.assertAlmostEqual(ax[1], 0.0, places=12)
        self.assertAlmostEqual(ax[2], 0.0, places=12)

    def test_worm_axis_is_global_y(self):
        """蜗杆自身轴线（局部 Z 的像）必须指向全局 +Y，且与蜗轮轴线(Z)正交"""
        _, _, az = self.wp.worm_basis()
        self.assertAlmostEqual(az[0], 0.0, places=12)
        self.assertAlmostEqual(az[1], 1.0, places=12)
        self.assertAlmostEqual(az[2], 0.0, places=12)

    def test_basis_comes_from_rotation_matrix_not_handwritten(self):
        """基底必须由旋转矩阵相乘得出，不得手写分量。

        这条守护针对"手抄分量抄错符号"这一类事故。
        """
        src = _read_source("worm_phase.py")
        self.assertIn("_matmul(", src)
        self.assertIn("_rot_x(-90.0)", src)
        self.assertIn("_rot_y(", src)
        # worm_basis 内部必须取自矩阵列，而不是字面量三元组
        body = src[src.index("def worm_basis("):]
        body = body[:body.index("def describes_phase(")]
        self.assertIn("worm_matrix(", body)
        self.assertNotIn("math.cos(", body)


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


class TestDocIntent(unittest.TestCase):
    """文档类别判定与三级生成策略。

    背景：Fusion 的「零件设计」文档只允许一个零部件，在其中调用
    occurrences.addNewComponent 会抛
        "零件设计文档只能包含一个零部件"
    因此必须先判定文档类别，再决定走哪条路径。此测试锁定该判定与分派逻辑。

    真实事故：在零件设计文档中生成装配体时直接崩溃报错，未给出可用指引。
    """

    def setUp(self):
        import doc_intent
        self.di = doc_intent

    # ---------- 枚举比较的健壮性 ----------

    @staticmethod
    def _make_intent_types():
        """模拟 SWIG 枚举：同一枚举值每次返回同一对象，__str__ 形如
        <DesignIntentTypes.PartDesignIntentType: 1>"""
        class IntentVal(object):
            def __init__(self, name, val):
                self._name = name
                self._val = val

            def __str__(self):
                return "<DesignIntentTypes.{}: {}>".format(self._name, self._val)

            __repr__ = __str__

        class Types(object):
            PartDesignIntentType = IntentVal("PartDesignIntentType", 0)
            AssemblyDesignIntentType = IntentVal("AssemblyDesignIntentType", 1)
            HybridDesignIntentType = IntentVal("HybridDesignIntentType", 2)

        return Types

    def test_same_intent_identical_object(self):
        T = self._make_intent_types()
        self.assertTrue(self.di.same_intent(T.PartDesignIntentType, T.PartDesignIntentType))

    def test_same_intent_equal_but_distinct_instance(self):
        T = self._make_intent_types()
        other = type(T.PartDesignIntentType)("PartDesignIntentType", 0)
        self.assertTrue(self.di.same_intent(other, T.PartDesignIntentType))

    def test_same_intent_none_and_unknown(self):
        T = self._make_intent_types()
        self.assertFalse(self.di.same_intent(None, T.PartDesignIntentType))
        self.assertFalse(self.di.same_intent(T.PartDesignIntentType, None))
        self.assertFalse(self.di.same_intent(object(), T.PartDesignIntentType))

    # ---------- 类别判定 ----------

    def _design_with(self, intent):
        class D(object):
            pass
        d = D()
        if intent is not None:
            d.designIntent = intent
        return d

    def test_classify_part_assembly_hybrid(self):
        T = self._make_intent_types()
        self.assertEqual(self.di.classify(self._design_with(T.PartDesignIntentType), T), self.di.PART)
        self.assertEqual(self.di.classify(self._design_with(T.AssemblyDesignIntentType), T), self.di.ASSEMBLY)
        self.assertEqual(self.di.classify(self._design_with(T.HybridDesignIntentType), T), self.di.HYBRID)

    def test_classify_unknown_when_intent_unreadable(self):
        """designIntent 读不到（preview 未启用 / 属性缺失）时必须归为 UNKNOWN，不能崩"""
        T = self._make_intent_types()

        class Boom(object):
            @property
            def designIntent(self):
                raise RuntimeError("preview not enabled")

        self.assertEqual(self.di.classify(Boom(), T), self.di.UNKNOWN)
        self.assertEqual(self.di.classify(None, T), self.di.UNKNOWN)
        self.assertEqual(self.di.classify(self._design_with(None), T), self.di.UNKNOWN)

    def test_classify_unknown_when_intent_types_missing(self):
        """DesignIntentTypes 不可用时也必须安全返回 UNKNOWN"""
        T = self._make_intent_types()
        self.assertEqual(self.di.classify(self._design_with(T.PartDesignIntentType), None), self.di.UNKNOWN)

    def test_classify_unknown_for_unrecognised_value(self):
        T = self._make_intent_types()
        weird = type(T.PartDesignIntentType)("SomethingElse", 9)
        self.assertEqual(self.di.classify(self._design_with(weird), T), self.di.UNKNOWN)

    # ---------- 三级策略分派 ----------

    def test_strategy_multi_part(self):
        """装配体模式：零件设计 -> 仅定位不绑定；装配设计 -> 正常；其它 -> 正常"""
        self.assertEqual(self.di.strategy_for(self.di.PART, True), self.di.STRATEGY_PART_NO_JOINTS)
        self.assertEqual(self.di.strategy_for(self.di.ASSEMBLY, True), self.di.STRATEGY_NORMAL)
        self.assertEqual(self.di.strategy_for(self.di.HYBRID, True), self.di.STRATEGY_NORMAL)
        self.assertEqual(self.di.strategy_for(self.di.UNKNOWN, True), self.di.STRATEGY_NORMAL)

    def test_strategy_single_part(self):
        """单个零件：零件设计 -> 建在根组件；其它 -> 正常新建零部件"""
        self.assertEqual(self.di.strategy_for(self.di.PART, False), self.di.STRATEGY_PART_NO_JOINTS)
        self.assertEqual(self.di.strategy_for(self.di.ASSEMBLY, False), self.di.STRATEGY_NORMAL)
        self.assertEqual(self.di.strategy_for(self.di.UNKNOWN, False), self.di.STRATEGY_NORMAL)

    def test_describe_covers_all_known_kinds(self):
        for kind in (self.di.PART, self.di.ASSEMBLY, self.di.HYBRID, self.di.UNKNOWN):
            self.assertTrue(self.di.describe(kind))
        self.assertEqual(self.di.describe(self.di.PART), "零件设计")
        self.assertEqual(self.di.describe(self.di.ASSEMBLY), "装配设计")

    # ---------- 源码一致性：分支必须走公共策略，避免再次漏改 ----------

    def test_generator_uses_doc_intent_module(self):
        src = _read_source("WormGearGenerator.py")
        self.assertIn("import doc_intent", src)
        self.assertIn("doc_intent.classify(", src)
        # 不允许再出现写死的字符串比较（曾因分支写反而出错）
        self.assertNotIn('doc_kind == "ASSEMBLY"', src)
        self.assertNotIn('doc_kind == "PART"', src)
        self.assertIn("doc_intent.ASSEMBLY", src)

    def test_every_generation_branch_consults_strategy_for(self):
        """三条生成路径都必须先问策略，不能自己写死条件。

        真实教训：装配体分支曾绕过公共策略直接判断，导致分支写反。
        这里逐条核对「装配体」「仅蜗杆」「仅蜗轮」三个分支内都有 strategy_for 调用。
        """
        src = _read_source("WormGearGenerator.py")

        def branch_body(marker, next_markers):
            start = src.index(marker)
            end = len(src)
            for nm in next_markers:
                idx = src.find(nm, start + len(marker))
                if idx != -1:
                    end = min(end, idx)
            return src[start:end]

        assembly = branch_body('if is_multi_part:', ['elif "仅蜗杆" in target_mode:'])
        worm = branch_body('elif "仅蜗杆" in target_mode:', ['else:', '\n        except'])
        wheel = branch_body('\n            else:\n                # 仅蜗轮', ['\n        except'])

        self.assertIn("doc_intent.strategy_for(", assembly,
                      "装配体分支未调用 doc_intent.strategy_for")
        self.assertIn("doc_intent.strategy_for(", worm,
                      "仅蜗杆分支未调用 doc_intent.strategy_for")
        self.assertIn("doc_intent.strategy_for(", wheel,
                      "仅蜗轮分支未调用 doc_intent.strategy_for")

    def test_reload_submodules_arity_matches_call_sites(self):
        """reload_submodules() 的返回个数必须与每个调用点的解包个数一致

        真实事故：给它新增第 4 个返回值后，三个老调用点仍按 3 个解包，
        会抛 ValueError: too many values to unpack。
        """
        src = _read_source("WormGearGenerator.py")
        ret = re.search(r"return\s+([\w\s,]+?)\s*$", src[src.find("def reload_submodules"):], re.M)
        self.assertIsNotNone(ret, "未找到 reload_submodules 的 return 语句")
        n_returned = len([t for t in ret.group(1).split(",") if t.strip()])

        for m in re.finditer(r"([\w\s,]+)=\s*reload_submodules\(\)", src):
            targets = [t.strip() for t in m.group(1).split(",") if t.strip()]
            line = src[:m.start()].count("\n") + 1
            self.assertEqual(
                len(targets), n_returned,
                f"第 {line} 行的解包个数 {len(targets)} 与返回值个数 {n_returned} 不一致"
            )


    def test_builders_tolerate_part_design_documents(self):
        """两个建模模块内的 addNewComponent 都必须包在 try 中。

        真实事故：WormGearGenerator 已按文档类别分流，但 gear_builder 内部
        「为保持时间线干净而在临时子组件里放样」这一步同样会新建零部件，
        在零件设计文档下直接抛
            "零件设计文档只能包含一个零部件"
        导致回退路径依然崩溃。此断言确保该调用始终有兜底。
        """
        for fname in ("gear_builder.py", "worm_builder.py"):
            src = _read_source(fname)
            for m in re.finditer(r"addNewComponent\(", src):
                # 该调用所在行往前看，必须处于某个 try 块内
                head = src[:m.start()]
                line_no = head.count("\n") + 1
                line_indent = len(head.rsplit("\n", 1)[-1]) - len(head.rsplit("\n", 1)[-1].lstrip())
                # 向上找最近的、缩进更浅的语句
                guarded = False
                for line in reversed(head.splitlines()):
                    stripped = line.strip()
                    if not stripped or stripped.startswith("#"):
                        continue
                    indent = len(line) - len(line.lstrip())
                    if indent < line_indent:
                        guarded = stripped.startswith("try:")
                        break
                self.assertTrue(
                    guarded,
                    f"{fname} 第 {line_no} 行的 addNewComponent 未包在 try 中，"
                    "零件设计文档下会直接抛错",
                )


    def test_fallback_path_applies_worm_assembly_transform(self):
        """零件设计回退路径必须把装配变换补到蜗杆实体上。

        真实事故：回退路径直接 w_builder.build_worm(root_comp, ...) 建在根组件，
        而**创建时的装配变换只对 occurrence 生效**，不会作用于实体本身。
        结果蜗杆以建模姿态留在原点，轴线与蜗轮轴线重合（都沿全局 Z），
        看起来像一根竖插在轮盘正中的圆柱，而不是在 x = a 处与蜗轮 90° 交错。

        本测试从源码层面校验：
          1. _build_into_root 接收 transform 参数；
          2. 装配体分支的所有回退调用都传入了 mat_trans；
          3. 装配变换由 _make_worm_transform() 统一构造。
        """
        src = _read_source("WormGearGenerator.py")

        m = re.search(r"def _build_into_root\(([^)]*)\)", src)
        self.assertIsNotNone(m, "未找到 _build_into_root 定义")
        self.assertIn("transform", m.group(1),
                      "_build_into_root 未接收 transform 参数，回退路径无法补装配变换")

        # 装配体模式下的回退调用必须带上 mat_trans
        asm = src[src.index("if is_multi_part:"):]
        asm = asm[:asm.index('elif "仅蜗杆" in target_mode:')]
        self.assertIn('_build_into_root("WORM", mat_trans)', asm,
                      "装配体回退路径未把 mat_trans 传给蜗杆")
        self.assertNotIn('_build_into_root("WORM")\n', asm,
                         "装配体回退路径仍存在不带变换的蜗杆调用")

        # 变换必须由统一构造函数产出，保证与 occurrence 路径同源
        self.assertIn("def _make_worm_transform()", src)
        self.assertIn("_make_worm_transform()", src)

    def test_transform_is_applied_to_body_in_fallback(self):
        """回退路径必须真的对实体施加变换（而不是只接收参数不用）

        并必须通过 _insert_body 注入——参数化设计下 BRepBodies.add 需要
        targetBaseFeature，直接调用会抛 "A valid targetBaseFeature is required"。
        """
        src = _read_source("WormGearGenerator.py")
        body = src[src.index("def _build_into_root("):]
        body = body[:body.index("def _no_joint_message(")]
        self.assertIn("mgr.transform(", body,
                      "回退路径未对蜗杆实体调用 transform")
        self.assertIn("if transform is None:", body,
                      "回退路径未判断 transform 是否给出")
        self.assertIn("_insert_body(", body,
                      "回退路径未通过 _insert_body 注入实体（缺少 BaseFeature 处理）")
        # 不允许再出现裸的 bRepBodies.add（参数化设计下必失败）
        self.assertNotIn("bRepBodies.add(moved)", body,
                         "回退路径仍直接调用 bRepBodies.add，参数化设计下会失败")


    def test_generator_uses_simple_phase_formula(self):
        """生成器必须使用 worm_phase 里的简单相位公式，不得再写数值求解器。

        教训：曾用一个未经验证的"干涉最小化求解器"代替几何公式，结果仍然干涉。
        现在相位就是一条直线几何关系：螺纹中心线从局部 +X 转到全局 -X，故 ψ = π。
        """
        src = _read_source("WormGearGenerator.py")
        self.assertIn("import worm_phase", src)
        self.assertIn("worm_phase.worm_phase_rad()", src)
        self.assertIn("worm_phase.worm_basis(", src)
        # 不得再出现求解器/搜索类调用
        for banned in ("solve_worm_phase", "count_interference", "build_check_geometry"):
            self.assertNotIn(banned, src, f"生成器不应再引用 {banned}")
        self.assertNotIn("theta1_0 =", src,
                         "不应再出现把 π/z1 当相位使用的 theta1_0")


if __name__ == "__main__":
    unittest.main(verbosity=2)
