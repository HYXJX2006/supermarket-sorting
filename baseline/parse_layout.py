import re

s = open('/tmp/layout.xml', encoding='utf-8', errors='replace').read()
bodies = re.findall(r'<body name="(product_\d+)"[^>]*?pos="([^"]+)"', s)
print('商品数:', len(bodies))
zs = {}
for name, pos in bodies:
    x, y, z = [float(v) for v in pos.split()]
    zs.setdefault(round(z, 3), []).append(name)
print('=== z 层分布 ===')
for z in sorted(zs):
    print(f'z={z}: {len(zs[z])} 个 -> {zs[z][:4]}...')
print()
for name, pos in bodies:
    if name in ('product_028', 'product_030', 'product_034'):
        print('重点:', name, 'pos=', pos)
