import re

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter

# Settings
N = 513
SIZE_X = 60.0
SIZE_Y = 60.0
HEIGHT = 5
SEED = 42

PNG = "src/fleet_simulation/terrain/forest_ground_heightmap.png"
OBJ = "src/fleet_simulation/terrain/forest_ground.obj"

rng = np.random.default_rng(SEED)

# Generate smooth hills
large = gaussian_filter(rng.normal(size=(N, N)), sigma=40)
medium = gaussian_filter(rng.normal(size=(N, N)), sigma=20)

Z = large + 0.25 * medium
Z -= Z.min()
Z /= Z.max()

# Override the previously generated terrain with a flat terrain at height 0.
# Z = np.zeros((N, N))

# Vertical offset applied to every generated tree base (meters).
# Use 0.0 for exact terrain-surface placement.
TREE_BASE_OFFSET = 0.0


def terrain_height_at(x, y, field=Z):
    x_norm = (x + SIZE_X / 2.0) / SIZE_X
    y_norm = (y + SIZE_Y / 2.0) / SIZE_Y

    col = np.clip(x_norm * (field.shape[1] - 1), 0, field.shape[1] - 1)
    row = np.clip(y_norm * (field.shape[0] - 1), 0, field.shape[0] - 1)

    x0 = int(np.floor(col))
    x1 = int(np.ceil(col))
    y0 = int(np.floor(row))
    y1 = int(np.ceil(row))

    tx = col - x0
    ty = row - y0

    z00 = field[y0, x0]
    z10 = field[y0, x1]
    z01 = field[y1, x0]
    z11 = field[y1, x1]

    return float(
        z00 * (1 - tx) * (1 - ty)
        + z10 * tx * (1 - ty)
        + z01 * (1 - tx) * ty
        + z11 * tx * ty
    )


def tree_base_z(x, y, field=Z):
    """Return terrain-following z with a tunable lift offset."""
    return terrain_height_at(x, y, field) * HEIGHT+ TREE_BASE_OFFSET


def tree_pose(x, y, model_uri, name, field=Z):
    z = tree_base_z(x, y, field)
    return f'      <include><name>{name}</name><uri>{model_uri}</uri><pose>{x:.6f} {y:.6f} {z:.6f} 0 0 0.000000</pose></include>'


def generate_tree_config(tree_positions, model_uri, prefix, field=Z):
    lines = []
    for i, (x, y) in enumerate(tree_positions, start=1):
        name = f"{prefix}{i:02d}"
        lines.append(tree_pose(x, y, model_uri, name, field))
    return "\n".join(lines)


def update_forest_world_tree_heights(world_file="src/fleet_simulation/worlds/forest.sdf.in", field=Z):
    with open(world_file, "r") as f:
        text = f.read()

    pattern = re.compile(
        r"(<include><name>.*?</name><uri>model://(?:pine|oak)_tree</uri><pose>)(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\s+0 0 0\.000000</pose></include>"
    )

    def repl(match):
        prefix = match.group(1)
        x = float(match.group(2))
        y = float(match.group(3))
        z = tree_base_z(x, y, field)
        return f"{prefix}{x:.6f} {y:.6f} {z:.6f} 0 0 0.000000</pose></include>"

    new_text = pattern.sub(repl, text)

    if new_text != text:
        with open(world_file, "w") as f:
            f.write(new_text)


# Save PNG
Image.fromarray((Z * 255).astype(np.uint8), mode="L").save(PNG)

# Make a lighter OBJ: 129x129 instead of 513x513
Zmesh = Z[::4, ::4]
rows, cols = Zmesh.shape

# Calculate normals
dy, dx = np.gradient(
    Zmesh * HEIGHT,
    SIZE_Y / (rows - 1),
    SIZE_X / (cols - 1),
)

normals = np.dstack((-dx, -dy, np.ones_like(Zmesh)))
normals /= np.linalg.norm(normals, axis=2, keepdims=True)

with open(OBJ, "w") as f:
    f.write("# Forest terrain\n")

    # Vertices
    for y in range(rows):
        py = -SIZE_Y / 2 + SIZE_Y * y / (rows - 1)

        for x in range(cols):
            px = -SIZE_X / 2 + SIZE_X * x / (cols - 1)
            pz = Zmesh[y, x] * HEIGHT

            f.write(f"v {px:.6f} {py:.6f} {pz:.6f}\n")

    # Normals
    for y in range(rows):
        for x in range(cols):
            nx, ny, nz = normals[y, x]
            f.write(f"vn {nx:.6f} {ny:.6f} {nz:.6f}\n")

    # Faces
    for y in range(rows - 1):
        for x in range(cols - 1):
            a = y * cols + x + 1
            b = a + 1
            c = a + cols
            d = c + 1

            f.write(f"f {a}//{a} {b}//{b} {d}//{d}\n")
            f.write(f"f {a}//{a} {d}//{d} {c}//{c}\n")

# Global forest layout for the world. Tree z-values are computed from the same
# terrain map used to generate the ground mesh, so when the hill field changes the
# tree bases move with it.
TREE_POSITIONS = [
    (9.0, 3.0),
    (6.0, 8.0),
    (-3.0, 10.0),
    (-9.0, 5.0),
    (-10.0, -2.0),
    (-7.0, -8.0),
    (1.0, -11.0),
    (8.0, -7.0),
    (12.0, 0.0),
    (11.0, 7.0),
    (2.0, 13.0),
    (-12.0, -6.0),
    (-4.0, -13.0),
    (10.0, -10.0),
    (-8.0, 9.0),
    (-13.0, 1.0),
    (0.0, -14.0),
    (13.0, -4.0),
    (8.0, 11.0),
    (-11.0, -10.0),
]

# This is the actual terrain-driven tree generator used by the forest world.
# Formula: z_tree = terrain_height(x, y) + TREE_BASE_OFFSET
# Set TREE_BASE_OFFSET to 0.0 if you want the base to sit exactly on terrain.
FOREST_TREE_BLOCK = generate_tree_config(TREE_POSITIONS, "model://pine_tree", "pine_c", Z)
update_forest_world_tree_heights()

# Keep the mesh generation output quiet, but leave the terrain-aware tree logic in
# the file so the world SDF can be generated from the mapping function directly.
#
# Important: tree XY layout in forest.sdf.in is deterministic and static. Running
# this script updates tree Z values from terrain; it does not randomize per run.

