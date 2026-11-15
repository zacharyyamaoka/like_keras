"""A second, independent check of the golden bytes: pyarrow rebuilds each frame and runs Arrow's full validator.

WHY: kpi/golden/lkf.py and like_keras/codec.py could agree on a layout that is not valid Arrow. pyarrow is the
reference implementation, so a frame it rebuilds from the raw buffers (`pa.Array.from_buffers`, no copy) and
validates in full is Arrow by construction, and its `to_pylist()` must equal the instance.
"""

from __future__ import annotations

from typing import Any

import pyarrow as pa

from kpi.golden import lkf


def arrow_type(field: lkf.Field) -> pa.DataType:
    kind = field.kind
    if kind == "f64":
        return pa.float64()
    if kind == "i64":
        return pa.int64()
    if kind == "bool":
        return pa.bool_()
    if kind == "utf8":
        return pa.utf8()
    if kind in ("strs", "f64s", "i64s"):
        item = lkf._children(field)[0]
        return pa.list_(pa.field("item", arrow_type(item), nullable=item.nullable))
    if kind == "struct":
        return pa.struct([pa.field(child.name, arrow_type(child), nullable=child.nullable) for child in lkf.SCHEMAS[field.of]])
    if kind == "map_str":
        return pa.map_(pa.utf8(), pa.utf8())
    if kind == "map_f64s":
        return pa.map_(pa.utf8(), pa.list_(pa.field("item", pa.float64(), nullable=False)))
    raise ValueError(kind)


def _build(data_type: pa.DataType, length: int, buffers: list[bytes], cursor: list[int]) -> pa.Array:
    def take(is_validity: bool = False) -> pa.Buffer | None:
        raw = buffers[cursor[0]]
        cursor[0] += 1
        # An empty validity buffer is the C interface's null pointer (no nulls); an empty data buffer is just empty.
        return None if is_validity and not raw else pa.py_buffer(raw)

    validity = take(is_validity=True)
    null_count = -1 if validity is not None else 0
    if pa.types.is_struct(data_type):
        children = [_build(data_type.field(i).type, length, buffers, cursor) for i in range(data_type.num_fields)]
        return pa.Array.from_buffers(data_type, length, [validity], null_count, children=children)
    if pa.types.is_map(data_type):
        offsets = take()
        total = int.from_bytes(offsets.to_pybytes()[-4:], "little", signed=True) if length else 0
        if take(is_validity=True) is not None:
            raise ValueError("map entries carry a validity buffer")
        keys = _build(data_type.key_type, total, buffers, cursor)
        items = _build(data_type.item_type, total, buffers, cursor)
        entries = pa.StructArray.from_arrays([keys, items], fields=[data_type.key_field, data_type.item_field])
        return pa.Array.from_buffers(data_type, length, [validity, offsets], null_count, children=[entries])
    if pa.types.is_list(data_type):
        offsets = take()
        total = int.from_bytes(offsets.to_pybytes()[-4:], "little", signed=True) if length else 0
        child = _build(data_type.value_type, total, buffers, cursor)
        return pa.Array.from_buffers(data_type, length, [validity, offsets], null_count, children=[child])
    if pa.types.is_string(data_type):
        return pa.Array.from_buffers(data_type, length, [validity, take(), take()], null_count)
    return pa.Array.from_buffers(data_type, length, [validity, take()], null_count)


def rebuild(type_name: str, frame: bytes) -> pa.StructArray:
    """The frame as a pyarrow struct array over the frame's own buffers, fully validated."""
    _, length, buffers = lkf.split(frame)
    cursor = [0]
    array = _build(arrow_type(lkf.Field("", "struct", of=type_name)), length, buffers, cursor)
    if cursor[0] != len(buffers):
        raise ValueError(f"{len(buffers) - cursor[0]} buffers left over")
    array.validate(full=True)
    return array


def to_message(type_name: str, array: pa.StructArray) -> dict[str, Any] | None:
    """Row 0 of a rebuilt array back in kpi/golden/lkf.py's dict form (numpy arrays keep every bit)."""

    def convert(field: lkf.Field, column: pa.Array, index: int) -> Any:
        if not column[index].is_valid:
            return None
        if field.kind in ("f64s", "i64s"):
            start, stop = column.offsets[index].as_py(), column.offsets[index + 1].as_py()
            return column.values.slice(start, stop - start).to_numpy(zero_copy_only=False)
        if field.kind == "strs":
            return tuple(column[index].as_py())
        if field.kind == "struct":
            return {child.name: convert(child, column.field(child.name), index) for child in lkf.SCHEMAS[field.of]}
        if field.kind in ("map_str", "map_f64s"):
            start, stop = column.offsets[index].as_py(), column.offsets[index + 1].as_py()
            keys, items = column.keys, column.items
            value_field = lkf.Field("value", "utf8" if field.kind == "map_str" else "f64s")
            return {keys[i].as_py(): convert(value_field, items, i) for i in range(start, stop)}
        if field.kind == "f64":
            return column.slice(index, 1).to_numpy(zero_copy_only=False)[0]
        return column[index].as_py()

    if len(array) == 0:
        return None
    return convert(lkf.Field("", "struct", of=type_name), array, 0)
