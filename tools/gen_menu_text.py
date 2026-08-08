#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_menu_text.py — 中文菜单字符串 → App_Menu.c 字库索引数组 生成器

把 "惯导设置" 这类中文串, 翻译成可直接粘贴进 App_Menu.c 的索引数组定义:

    static const menu_chn_idx_enum TXT_GUANDAO_SHEZHI[] = { MENU_CHN_GUAN, MENU_CHN_DAO, MENU_CHN_SHE, MENU_CHN_ZHI3 };  // 惯导设置

顺带校验每个字是否在 menu_font.h 字库里; 缺字会明确报错并给出补字步骤。

生成结果会同时写到 tools/menu_text_out.txt (UTF-8) —— 从该文件复制粘贴,
避免 Windows 控制台 GBK 编码把中文搞乱。

用法:
    python tools/gen_menu_text.py "惯导设置" "无人机控制"
    python tools/gen_menu_text.py --list            # 列出字库全部汉字
    不带参数 → 交互模式: 逐行输入中文串, 空行结束

可选: pip install pypinyin  后可自动生成 TXT_GUANDAO_SHEZHI 这类拼音变量名,
      不装也能用 (退回 TXT_0 / TXT_1 ...)。
"""

import sys
import re
from datetime import datetime
from pathlib import Path

# Windows 控制台多为 GBK, 统一按 UTF-8 输出 (乱码无所谓, 从文件复制)
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

# 字库头文件 (相对本脚本位置: tools/ → ../project/code/menu_font.h)
FONT_H = Path(__file__).resolve().parent.parent / "project" / "code" / "menu_font.h"
OUT_TXT = Path(__file__).resolve().parent / "menu_text_out.txt"

# 形如:  MENU_CHN_MAI    = 0,    // 麦
_ENUM_RE = re.compile(r"MENU_CHN_(\w+)\s*=\s*(\d+)\s*,\s*//\s*(.+)")


def load_font_table():
    """解析 menu_font.h 的枚举注释, 返回 { 汉字: (完整枚举名, 索引) }"""
    if not FONT_H.exists():
        sys.exit(f"[错误] 找不到字库头文件: {FONT_H}")
    table = {}
    for lineno, line in enumerate(FONT_H.read_text(encoding="utf-8").splitlines(), 1):
        m = _ENUM_RE.search(line)
        if not m:
            continue
        suffix, index, comment = m.group(1), int(m.group(2)), m.group(3)
        char = comment[0]          # 取注释里第一个字符 (汉字或全角逗号)
        name = f"MENU_CHN_{suffix}"
        if char in table:
            print(f"[警告] {FONT_H.name}:{lineno} 字符 {char!r} 重复 (已有 {table[char][0]}), 取后者")
        table[char] = (name, index)
    return table


def pinyin_name(text):
    """惯导设置 → GUANDAO_SHEZHI; 无 pypinyin 时返回 None (退回 TXT_<n>)"""
    try:
        from pypinyin import lazy_pinyin
        return "".join(p.upper() for p in lazy_pinyin(text))
    except ImportError:
        return None


def gen_one(text, seq, table):
    """生成一个字符串的索引数组定义, 返回可粘贴文本 (缺字时返回错误说明)"""
    missing = [c for c in text if c not in table]
    if missing:
        miss = " ".join(missing)
        return ("[缺字] %r 缺 %d 个字: %s\n"
                "       补字步骤: 1) menu_font.c 末尾追加该字 32 字节字形  "
                "2) menu_font.h 枚举末尾加 MENU_CHN_XX  3) 重跑本脚本"
                % (text, len(missing), miss))

    name = pinyin_name(text) or f"TXT_{seq}"
    elems = ", ".join(table[c][0] for c in text)
    return ("/* %s */\n"
            "static const menu_chn_idx_enum %s[] = { %s };  // %s\n"
            "// 菜单项用法(贴进菜单项, 跳二级就把 STATE_MAIN 换成对应目标): "
            "{ { %s, %d }, STATE_MAIN, NULL },   // %s"
            % (text, name, elems, text, name, len(text), text))


def main():
    args = sys.argv[1:]

    if "--list" in args:
        table = load_font_table()
        print(f"字库现有汉字 ({len(table)} 个):")
        for c in sorted(table):
            print(f"  {c}  {table[c][0]}({table[c][1]})")
        return

    table = load_font_table()

    if args:
        strings = args
    else:
        print("交互模式: 逐行输入中文串, 空行结束。示例: 惯导设置")
        strings = []
        while True:
            try:
                s = input("> ").strip()
            except EOFError:
                break
            if not s:
                break
            strings.append(s)

    if not strings:
        return

    blocks = []
    for i, s in enumerate(strings):
        blocks.append(gen_one(s, i, table))

    # 打印到控制台
    print("\n\n".join(blocks))
    print()

    # 同时写入 UTF-8 文件, 从文件复制粘贴 (避免控制台编码问题)
    header = (f"// 由 gen_menu_text.py 生成 @ {datetime.now():%Y-%m-%d %H:%M:%S}\n"
              f"// 输入: {' '.join(strings)}\n\n")
    OUT_TXT.write_text(header + "\n\n".join(blocks) + "\n", encoding="utf-8")
    print(f"[已写入] {OUT_TXT}  (UTF-8, 从该文件复制粘贴到 App_Menu.c)")


if __name__ == "__main__":
    main()
