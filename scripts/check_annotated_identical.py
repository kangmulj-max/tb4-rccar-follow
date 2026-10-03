#!/usr/bin/env python3
"""주석본이 원본과 '코드상' 동일한지 검증: 주석·공백을 무시하고 AST(구문 트리)를 비교.
  python3 check_annotated_identical.py [원본] [주석본]
docstring 은 문자열 노드라 AST 에 포함됨 → 원본 docstring 은 주석본에서도 그대로 유지해야 통과."""
import ast, sys

a = sys.argv[1] if len(sys.argv) > 1 else 'scripts/follow_object_node_v3.py'
b = sys.argv[2] if len(sys.argv) > 2 else 'scripts/follow_object_node_v3_annotated.py'
da = ast.dump(ast.parse(open(a, encoding='utf-8').read()))
db = ast.dump(ast.parse(open(b, encoding='utf-8').read()))
print('IDENTICAL (comments only)' if da == db else 'DIFFERENT CODE')
sys.exit(0 if da == db else 1)
