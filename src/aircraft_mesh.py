"""AC3D asset reader and GPU-ready F-22 mesh preparation (Python 3.7)."""
import io
import math
from functools import lru_cache
from pathlib import Path
import shlex
import sys


def resource_root():
    return Path(getattr(sys, '_MEIPASS', Path(__file__).resolve().parents[1]))


def parse_ac3d(data):
    stream = io.BytesIO(data)

    def line():
        value = stream.readline()
        if not value:
            raise ValueError('Unexpected end of AC3D asset')
        return value.decode('utf-8').strip()

    if line() != 'AC3Db':
        raise ValueError('Unsupported AC3D header')
    materials = []
    current = line()
    while current.startswith('MATERIAL '):
        fields = shlex.split(current)
        def values(key, count):
            start = fields.index(key) + 1
            return tuple(float(value) for value in fields[start:start + count])
        materials.append({'name': fields[1], 'rgb': values('rgb', 3),
                          'spec': values('spec', 3), 'shininess': values('shi', 1)[0]})
        current = line()

    def node(header):
        if not header.startswith('OBJECT '):
            raise ValueError('Expected AC3D object')
        result = {'name': '', 'vertices': [], 'faces': [], 'texture': None,
                  'loc': (0., 0., 0.), 'rot': ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)), 'children': []}
        while True:
            fields = shlex.split(line())
            if not fields:
                continue
            key = fields[0]
            if key == 'name':
                result['name'] = fields[1]
            elif key == 'texture':
                result['texture'] = Path(fields[1]).name
            elif key == 'loc':
                result['loc'] = tuple(map(float, fields[1:4]))
            elif key == 'rot':
                flat = list(map(float, fields[1:10]))
                result['rot'] = tuple(tuple(flat[k:k + 3]) for k in (0, 3, 6))
            elif key == 'data':
                size = int(fields[1])
                if len(stream.read(size)) != size:
                    raise ValueError('Truncated AC3D data')
                terminator = stream.read(1)
                if terminator == b'\r':
                    terminator = stream.read(1)
                if terminator != b'\n':
                    raise ValueError('Invalid AC3D data terminator')
            elif key == 'numvert':
                result['vertices'] = [tuple(map(float, line().split())) for _ in range(int(fields[1]))]
            elif key == 'numsurf':
                for _ in range(int(fields[1])):
                    surface = shlex.split(line())
                    if surface[0] != 'SURF':
                        raise ValueError('Expected AC3D surface')
                    flags = int(surface[1], 0)
                    material_line = shlex.split(line())
                    if material_line[0] != 'mat':
                        raise ValueError('Expected surface material')
                    material = int(material_line[1])
                    refs_line = shlex.split(line())
                    if refs_line[0] != 'refs':
                        raise ValueError('Expected surface references')
                    refs = [line().split() for _ in range(int(refs_line[1]))]
                    indices = [int(ref[0]) for ref in refs]
                    if not 0 <= material < len(materials) or any(not 0 <= i < len(result['vertices']) for i in indices):
                        raise ValueError('AC3D material/vertex index out of range')
                    if flags & 15 == 0 and len(indices) >= 3:
                        result['faces'].append({'indices': indices, 'uv': [tuple(map(float, ref[1:3])) for ref in refs],
                                                'material': material, 'smooth': bool(flags & 16)})
            elif key == 'kids':
                result['children'] = [node(line()) for _ in range(int(fields[1]))]
                return result
            elif key not in ('crease', 'texrep', 'texoff', 'url'):
                raise ValueError('Unsupported AC3D property: ' + key)

    tree = node(current)
    objects = []
    identity = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
    def walk(item, rotation, translation, ancestry):
        local = item['rot']
        combined = tuple(tuple(sum(rotation[i][k] * local[k][j] for k in range(3)) for j in range(3)) for i in range(3))
        offset = tuple(translation[i] + sum(rotation[i][k] * item['loc'][k] for k in range(3)) for i in range(3))
        names = ancestry + [item['name']]
        if item['vertices']:
            vertices = [tuple(offset[i] + sum(combined[i][k] * point[k] for k in range(3)) for i in range(3))
                        for point in item['vertices']]
            if not all(len(point) == 3 and all(math.isfinite(value) for value in point) for point in vertices):
                raise ValueError('Invalid AC3D vertex')
            objects.append(dict(item, vertices=vertices, ancestry=names))
        for child in item['children']:
            walk(child, combined, offset, names)
    walk(tree, identity, (0., 0., 0.), [])
    return {'materials': materials, 'objects': objects}


@lru_cache(maxsize=2)
def load_f22(path=None):
    import numpy as np
    path = Path(path) if path else resource_root() / 'assets/f22/Models/F-22.ac'
    parsed = parse_ac3d(path.read_bytes())
    hidden = ('gear', 'wheel', 'bay', 'oiltank', 'box', 'cylinder', 'line0', 'pilot_', 'display')
    objects = [item for item in parsed['objects'] if not any(
        token in '/'.join(item['ancestry']).lower() for token in hidden)]
    all_vertices = np.concatenate([np.asarray(item['vertices'], dtype=float) for item in objects])
    xmin, xmax = all_vertices[:, 0].min(), all_vertices[:, 0].max()
    origin = np.array([(xmin + xmax) / 2., -.85, 0.])
    scale = 4.1 / (xmax - xmin)
    batches, parts = [], []
    for item in objects:
        source = (np.asarray(item['vertices'], dtype=float) - origin) * scale
        # AC3D asset: nose=-X, up=+Y, left=+Z -> body: forward=+X, right=+Y, down=+Z.
        vertices = np.column_stack((-source[:, 0], -source[:, 2], -source[:, 1]))
        triangles = []
        for face in item['faces']:
            for k in range(1, len(face['indices']) - 1):
                corners = (0, k, k + 1)
                indices = [face['indices'][i] for i in corners]
                points = vertices[indices]
                normal = np.cross(points[1] - points[0], points[2] - points[0])
                length = np.linalg.norm(normal)
                if length > 1e-10:
                    triangles.append((indices, [face['uv'][i] for i in corners], face, normal / length))
        smooth = np.zeros_like(vertices)
        for indices, _, _, normal in triangles:
            smooth[indices] += normal
        lengths = np.linalg.norm(smooth, axis=1)
        smooth /= np.maximum(lengths[:, None], 1e-12)
        grouped = {}
        for indices, uv, face, normal in triangles:
            target = grouped.setdefault(face['material'], [])
            for index, coord in zip(indices, uv):
                n = smooth[index] if face['smooth'] and lengths[index] > 1e-10 else normal
                target.append(tuple(vertices[index]) + tuple(n) + tuple(coord))
        for material, rows in grouped.items():
            mat = parsed['materials'][material]
            canopy = item['name'].lower() == 'canopy'
            texture = path.parent / item['texture'] if item['texture'] else None
            if texture and not texture.is_file():
                raise ValueError('Missing model texture: ' + str(texture))
            batches.append({'array': np.asarray(rows, dtype=np.float32), 'texture': str(texture) if texture else None,
                            'color': (.69, .53, .24) if canopy else mat['rgb'],
                            'alpha': .62 if canopy else 1., 'shininess': 100. if canopy else min(48., mat['shininess']),
                            'name': item['name']})
        parts.append({'name': item['name'], 'center': tuple(vertices.mean(axis=0))})
    body = np.concatenate([batch['array'][:, :3] for batch in batches])
    return {'batches': batches, 'triangles': sum(len(batch['array']) // 3 for batch in batches),
            'bounds': (tuple(body.min(axis=0)), tuple(body.max(axis=0))), 'parts': parts}
