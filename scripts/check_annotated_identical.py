#!/usr/bin/env python3
"""주석본이 원본과 '코드상' 동일한지 검증: 주석·공백을 무시하고 AST(구문 트리)를 비교.
  python3 check_annotated_identical.py [원본] [주석본]
docstring 은 문자열 노드라 AST 에 포함됨 → 원본 docstring 은 주석본에서도 그대로 유지해야 통과.
  --ignore-docstrings: docstring 을 빼고 비교 (make_lean.py 가 docstring 을 첫 줄로 줄인 실행본 검증용)"""
import ast, sys

args = [x for x in sys.argv[1:] if not x.startswith('--')]
IGNORE_DOC = '--ignore-docstrings' in sys.argv


def tree(path):
    t = ast.parse(open(path, encoding='utf-8').read())
    if IGNORE_DOC:
        for n in ast.walk(t):
            if isinstance(n, (ast.Module, ast.ClassDef, ast.FunctionDef)) and n.body and isinstance(n.body[0], ast.Expr) \
                    and isinstance(n.body[0].value, ast.Constant) and isinstance(n.body[0].value.value, str):
                n.body = n.body[1:] or [ast.Pass()]
    return ast.dump(t)


a = args[0] if len(args) > 0 else 'scripts/follow_object_node_v3.py'
b = args[1] if len(args) > 1 else 'scripts/follow_object_node_v3_annotated.py'
da, db = tree(a), tree(b)
print('IDENTICAL (comments only)' if da == db else 'DIFFERENT CODE')
sys.exit(0 if da == db else 1)
