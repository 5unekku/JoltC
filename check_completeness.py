#!/usr/bin/env python3
"""verify every JPC_API declaration in Functions.h has an implementation in JoltC.cpp"""

import re, sys

FN_RE = re.compile(r'\b(JPC_[A-Za-z0-9_]+)\s*\(')


def extract_declared(path):
    names = set()
    with open(path) as f:
        for line in f:
            stripped = line.strip()
            if not stripped.startswith('JPC_API'):
                continue
            m = FN_RE.search(stripped)
            if m:
                names.add(m.group(1))
    return names


def extract_defined(path):
    with open(path) as f:
        content = f.read()
    names = set()
    # explicit definitions: JPC_API ... JPC_Name( ... ) {
    for m in re.finditer(r'JPC_API\b[^;{]*?(JPC_[A-Za-z0-9_]+)\s*\(', content):
        fname = m.group(1)
        pos = m.end()
        rest = content[pos:]
        brace = rest.find('{')
        semi = rest.find(';')
        if brace != -1 and (semi == -1 or brace < semi):
            names.add(fname)
    # DESTRUCTOR(type) macro generates type_delete
    for m in re.finditer(r'\bDESTRUCTOR\s*\(\s*(JPC_[A-Za-z0-9_]+)\s*\)', content):
        names.add(m.group(1) + '_delete')
    return names


declared = extract_declared('JoltC/Functions.h')
defined = extract_defined('JoltCImpl/JoltC.cpp')

missing = sorted(declared - defined)
if missing:
    print(f'ERROR: {len(missing)} declared function(s) have no implementation:')
    for name in missing:
        print(f'  {name}')
    sys.exit(1)

print(f'OK: all {len(declared)} declared functions are implemented')
