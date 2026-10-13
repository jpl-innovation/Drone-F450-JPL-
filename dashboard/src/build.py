#!/usr/bin/env python3
"""Embeds index.html into server.py and writes ../drone_dashboard.py (the single file you copy to the Pi)."""
import os
here = os.path.dirname(os.path.abspath(__file__))
html = open(os.path.join(here, 'index.html'), encoding='utf-8').read()
assert "'''" not in html, "index.html must not contain triple single quotes"
src = open(os.path.join(here, 'server.py'), encoding='utf-8').read()
assert '__INDEX_HTML__' in src
out = os.path.join(here, '..', 'drone_dashboard.py')
open(out, 'w', encoding='utf-8').write(src.replace('__INDEX_HTML__', html))
os.chmod(out, 0o755)
print('built', os.path.normpath(out), os.path.getsize(out), 'bytes')
