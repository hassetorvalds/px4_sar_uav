"""阶段状态机的静态单测（不飞、不启 ROS）。

背景（服务器侧 2026-10-14 建议）：2026-10-14 的严重 bug 是
"ALIGN 分支里 return 之后紧跟 enter('APPROACH')"——enter 变成死代码，
于是"对准成功"时阶段永不推进（驱动卡死 11 分钟、心跳却正常）。这类错误静态检查就能拦住。

本测试对 scripts/vlm_closed_loop.py 做两项检查：

1. 死代码：同一语句块里出现 return/break/continue/raise 之后仍有可执行语句 → 失败；
2. 分支无转移：on_phase 里每个 `if self.phase ...` 分支至少要能推进状态机
   （enter / finish / fail / send('land_fast')），否则该阶段永远出不来。
"""

import ast
import os

import pytest

DRIVER = os.path.normpath(os.path.join(
    os.path.dirname(__file__),          # .../src/vlm/test
    os.pardir, os.pardir, os.pardir, os.pardir,   # → 仓库根
    'scripts', 'vlm_closed_loop.py'))


def _load():
    if not os.path.exists(DRIVER):
        pytest.skip(f'找不到 {DRIVER}')
    with open(DRIVER, encoding='utf-8') as handle:
        source = handle.read()
    return source, ast.parse(source)


def test_no_statement_after_terminator():
    """任何语句块里 return 之后都不该再写代码（Bug1 的形态）。"""
    _source, tree = _load()
    offenders = []
    terminators = (ast.Return, ast.Break, ast.Continue, ast.Raise)
    for node in ast.walk(tree):
        for _field, block in ast.iter_fields(node):
            if not isinstance(block, list) or not block:
                continue
            if not all(isinstance(item, ast.stmt) for item in block):
                continue
            for index, statement in enumerate(block[:-1]):
                if isinstance(statement, terminators):
                    dead = block[index + 1]
                    offenders.append(f'第 {getattr(dead, "lineno", "?")} 行在 return 之后仍有语句')
                    break
    assert not offenders, (
        '发现死代码（return 之后的语句）——2026-10-14 的 ALIGN 卡死就是这种形态：\n  '
        + '\n  '.join(offenders))


def test_every_phase_branch_can_transition():
    """on_phase 的每个阶段分支都必须能推进状态机（否则该阶段会永久停住）。"""
    _source, tree = _load()
    on_phase = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == 'on_phase':
            on_phase = node
            break
    assert on_phase is not None, '找不到 on_phase（函数被改名？）'

    def is_transition(call: ast.Call) -> bool:
        target = call.func
        if isinstance(target, ast.Attribute):
            if target.attr in ('enter', 'finish', 'fail'):
                return True
            if target.attr == 'send' and call.args:
                first = call.args[0]
                return (isinstance(first, ast.Constant)
                        and isinstance(first.value, str)
                        and 'land' in first.value)
        return False

    phases = []
    for statement in ast.walk(on_phase):
        if not isinstance(statement, ast.If):
            continue
        test = statement.test
        if not (isinstance(test, ast.Compare) and isinstance(test.left, ast.Attribute)
                and test.left.attr == 'phase'):
            continue
        names = []
        for comparator in test.comparators:
            if isinstance(comparator, ast.Constant) and isinstance(comparator.value, str):
                names.append(comparator.value)
            elif isinstance(comparator, (ast.Tuple, ast.List)):
                names.extend(e.value for e in comparator.elts
                             if isinstance(e, ast.Constant) and isinstance(e.value, str))
        for name in names:
            phases.append((name, statement))

    assert phases, '没找到任何 `if self.phase ...` 分支（结构变了？）'
    for name, statement in phases:
        calls = [node for node in ast.walk(statement) if isinstance(node, ast.Call)]
        assert any(is_transition(call) for call in calls), (
            f'阶段 {name!r} 的分支里没有任何 enter()/finish()/fail()/land —— '
            '该阶段一旦进入就永远出不来（2026-10-14 ALIGN 的教训）')
