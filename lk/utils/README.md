Should just be pure python utils, no internal dependencies


We keep it very flat. Even though it may be hard to find when you navigate it makes easier to import!

- As we may not be installing the libraries for these utils I want to avoid alot of rexporting into common packages.
- This allows good selectivity
---


### Naming Guidelines


## 2. Simple naming rules that work well

### Rule A – Never use bare library names at the top level

Avoid `torch.py`, `json.py`, `typing.py`, `random.py` etc **in the repo root**.
That’s where the real collisions happen.

### Rule B – Under a package, prefer “X_utils” / “X_ext” for library-specific helpers

Inside `bam.utils`, I’d do:

* `torch_utils.py` (or `torch_ext.py`)
* `pinocchio_utils.py` / `pin_utils.py`
* `numpy_utils.py`
* `ros2_utils.py`

This makes it obvious these are wrappers around a library, not the library itself.

### Rule C – For conceptual utilities, use domain names, not library names

These are great and safe:

* `pointcloud.py`
* `geometry.py`
* `fs.py` / `path_utils.py`
* `logging_utils.py`
* `time_utils.py`

Just avoid exact stdlib names when you can (`json`, `pathlib`, `logging`).

---

## 3. How to keep imports short *and* safe

### Pattern 1 – “Verbose file names, short re-exports”

Give modules clear names, then re-export symbols at package level:

```bash
bam/
  utils/
    __init__.py
    pointcloud_ops.py
    torch_utils.py
```

`pointcloud_ops.py`:

```python
# bam/utils/pointcloud_ops.py
def voxel_downsample(...):
    ...
def estimate_normals(...):
    ...
```

`torch_utils.py`:

```python
# bam/utils/torch_utils.py
def to_device(...): ...
def detach_cpu(...): ...
```

`__init__.py`:

```python
from .pointcloud_ops import voxel_downsample, estimate_normals
from .torch_utils import to_device, detach_cpu

__all__ = [
    "voxel_downsample",
    "estimate_normals",
    "to_device",
    "detach_cpu",
]
```

Then you get super clean imports:

```python
from bam.utils import voxel_downsample, to_device
```

Short to type, no weird collisions, filenames stay descriptive.

---

### Pattern 2 – Subpackages for bigger themes

If a util area grows:

```bash
bam/
  pointcloud/
    __init__.py
    ops.py
    io.py
```

Then:

```python
from bam.pointcloud import ops as pc_ops
```

Still short, but better structured than one giant `utils.py`.

---
