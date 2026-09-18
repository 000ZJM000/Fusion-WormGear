"""
doc_intent.py - Fusion 文档类别判定

Fusion 有两种设计文档，零部件容量不同：

    Part     零件设计 ：整个文档只允许一个零部件
    Assembly 装配设计 ：可以含多个子零部件
    Hybrid   混合设计 ：兼容两者

本插件生成「装配体」时需要新建两个子零部件（蜗轮 / 蜗杆）才能建立旋转副，
因此必须先判定文档类别，再决定走哪条路径：

    1) PART     -> 正常生成两个零件并正确定位，但声明「未建立装配绑定，仅移动了位置」
    2) ASSEMBLY -> 正常生成；若新建零部件仍失败，说明判定有误或文档状态异常，中止报错
    3) 其他     -> 按普通模式正常生成

`Design.designIntent` 在 Autodesk SDK 中被标注为 preview 特性
（"may change in future releases. Do not use it in distributed programs."），
因此这里的枚举比较做了多重兜底：读不到、或取值不认识时一律归入「其他」，
不会因为 API 变动而中断生成。

本模块不依赖 adsk，可脱离 Fusion 单独测试（见 test_math.py 的 TestDocIntent）。
"""

# 文档类别常量
PART = "PART"
ASSEMBLY = "ASSEMBLY"
HYBRID = "HYBRID"
UNKNOWN = "UNKNOWN"

# 生成策略
STRATEGY_NORMAL = "NORMAL"                      # 正常新建零部件
STRATEGY_PART_NO_JOINTS = "PART_NO_JOINTS"      # 建在根组件，不建立装配绑定
STRATEGY_ABORT = "ABORT"                        # 直接中止并报错


def same_intent(a, b) -> bool:
    """健壮地比较两个设计意图枚举值。

    SWIG 生成的枚举对象不保证 __eq__ 语义，故依次尝试：
    同一对象 -> == 比较 -> 字符串尾段比较。
    """
    if a is None or b is None:
        return False
    if a is b:
        return True
    try:
        if a == b:
            return True
    except Exception:
        pass
    ta = str(a).rsplit(".", 1)[-1].rstrip(">").split(":")[0].strip().lower()
    tb = str(b).rsplit(".", 1)[-1].rstrip(">").split(":")[0].strip().lower()
    return ta == tb


def read_doc_intent(design):
    """读取 design.designIntent；任何异常（属性缺失 / preview 未启用）都返回 None。"""
    if design is None:
        return None
    try:
        return design.designIntent
    except Exception:
        return None


def classify(design, intent_types=None) -> str:
    """判定文档类别，返回 PART / ASSEMBLY / HYBRID / UNKNOWN。

    :param design: adsk.fusion.Design 对象
    :param intent_types: adsk.fusion.DesignIntentTypes（为 None 时返回 UNKNOWN）
    """
    if intent_types is None:
        return UNKNOWN

    part = getattr(intent_types, "PartDesignIntentType", None)
    assy = getattr(intent_types, "AssemblyDesignIntentType", None)
    hyb = getattr(intent_types, "HybridDesignIntentType", None)

    intent = read_doc_intent(design)
    if intent is None:
        return UNKNOWN

    if same_intent(intent, part):
        return PART
    if same_intent(intent, assy):
        return ASSEMBLY
    if same_intent(intent, hyb):
        return HYBRID
    return UNKNOWN


def strategy_for(doc_kind: str, is_multi_part: bool) -> str:
    """给定文档类别与生成需求，返回应采取的策略。

    :param doc_kind: classify() 的结果
    :param is_multi_part: 本次是否需要多个零部件
                          （装配体模式 = True；仅蜗轮 / 仅蜗杆 = False）
    """
    if is_multi_part:
        if doc_kind == PART:
            # 零件设计放不下两个零部件：建在根组件，不建立装配绑定
            return STRATEGY_PART_NO_JOINTS
        if doc_kind == ASSEMBLY:
            # 装配设计本应可以新建零部件；若仍失败则属异常，交由调用方中止报错
            return STRATEGY_NORMAL
        return STRATEGY_NORMAL

    # 单个零件
    if doc_kind == PART:
        # 零件设计：单个零件也建不了子零部件，直接建在根组件
        return STRATEGY_PART_NO_JOINTS
    return STRATEGY_NORMAL


def describe(doc_kind: str) -> str:
    """把文档类别翻译成给用户看的中文名。"""
    return {
        PART: "零件设计",
        ASSEMBLY: "装配设计",
        HYBRID: "混合设计",
    }.get(doc_kind, "未知类别")
