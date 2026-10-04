#!/usr/bin/env python3
"""주석본 → 실행본 생성: 주석을 지우고 docstring 은 첫 줄만 남긴다 (코드는 한 글자도 바꾸지 않음).
  python3 make_lean.py [주석본] [실행본]
  기본: scripts/follow_object_node_v3_safe_annotated.py → scripts/follow_object_node_v3_safe.py
검증: python3 scripts/check_annotated_identical.py --ignore-docstrings <실행본> <주석본>"""
import ast, io, sys, tokenize

src_path = sys.argv[1] if len(sys.argv) > 1 else 'scripts/follow_object_node_v3_safe_annotated.py'
dst_path = sys.argv[2] if len(sys.argv) > 2 else 'scripts/follow_object_node_v3_safe.py'
src = open(src_path, encoding='utf-8').read()
lines = src.splitlines()

# 1) 주석 제거: 줄 끝 주석은 잘라내고, 주석만 있던 줄은 삭제 표시
drop = set()
for tok in tokenize.generate_tokens(io.StringIO(src).readline):
    if tok.type == tokenize.COMMENT:
        r, c = tok.start[0] - 1, tok.start[1]
        if lines[r][:c].strip():
            lines[r] = lines[r][:c].rstrip()
        else:
            drop.add(r)

# 2) docstring 은 첫 줄만 (모듈·클래스·함수 본문의 첫 문자열)
for node in ast.walk(ast.parse(src)):
    if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef)) and node.body:
        d = node.body[0]
        if isinstance(d, ast.Expr) and isinstance(d.value, ast.Constant) and isinstance(d.value.value, str):
            first = d.value.value.strip().splitlines()[0].strip()
            if first.endswith((',', '(', ':')):          # 문장이 다음 줄로 이어지면 끊긴 표시
                first = first.rstrip(',(:').rstrip() + ' …'
            indent = lines[d.lineno - 1][:len(lines[d.lineno - 1]) - len(lines[d.lineno - 1].lstrip())]
            lines[d.lineno - 1] = f'{indent}"""{first}"""'
            drop.update(range(d.lineno, d.end_lineno))

# 3) 빈 줄 정리: 연속 빈 줄은 최상위 정의 앞 2 줄, 그 밖에는 1 줄까지
out = []
for i, line in enumerate(lines):
    if i in drop:
        continue
    if not line.strip():
        if out and not out[-1].strip() and (len(out) < 2 or out[-2].strip()):
            out.append('')          # 두 번째 빈 줄은 일단 두고 아래에서 정리
            continue
        if out and not out[-1].strip():
            continue
    out.append(line)
res = []
for i, line in enumerate(out):
    if not line.strip() and res and not res[-1].strip():
        nxt = next((l for l in out[i + 1:] if l.strip()), '')
        if not (nxt.startswith(('def ', 'class ', 'if __name__')) and not nxt.startswith(' ')):
            continue
    res.append(line)
while res and not res[0].strip():
    res.pop(0)

header = ['#!/usr/bin/env python3',
          f'# follow_object_node_v3_safe — 실행본 (자동 생성: scripts/make_lean.py, 원본 주석본: {src_path.split("/")[-1]})',
          '# 이 파일을 직접 고치지 말고 주석본을 고친 뒤 make_lean.py 로 다시 만들 것. 설명·근거는 주석본과 docs/ 참고.']
body = [l for l in res if not l.startswith('#!')]
open(dst_path, 'w', encoding='utf-8').write('\n'.join(header + [''] + body).rstrip() + '\n')
print(f'{src_path} ({len(src.splitlines())} 줄) → {dst_path} ({len(header) + 1 + len(body)} 줄)')
