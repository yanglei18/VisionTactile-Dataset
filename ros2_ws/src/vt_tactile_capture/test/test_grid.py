"""Only the native row44 state schema is accepted."""
from types import SimpleNamespace
import pytest
from vt_tactile_capture.grid import CELL_ROW_INDICES, LAYOUT_ID, normalize_state


def test_native_state_is_returned_unchanged_and_rows_map_to_cells():
    state = SimpleNamespace(schema_version=3, rows=20, cols=20, layout_id=LAYOUT_ID)
    assert normalize_state(state) is state
    assert CELL_ROW_INDICES == tuple(i // 20 for i in range(400))


@pytest.mark.parametrize("schema,rows,cols,layout", [
    (1,80,80,"legacy"), (2,20,20,"row20_col_group_stride8_v2"),
    (3,20,20,"wrong"), (True,20,20,LAYOUT_ID), (3,True,20,LAYOUT_ID),
    (3,20.0,20,LAYOUT_ID), (3,20,19,LAYOUT_ID),
])
def test_incompatible_or_invalid_states_are_rejected(schema, rows, cols, layout):
    with pytest.raises(ValueError):
        normalize_state(SimpleNamespace(schema_version=schema, rows=rows, cols=cols, layout_id=layout))
