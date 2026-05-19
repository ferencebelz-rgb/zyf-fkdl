from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8", errors="ignore")


def test_camera_exposes_structured_junction_status():
    header = read("code/camera.h")

    assert "JUNCTION_SIDE_LEFT_T" in header
    assert "JUNCTION_SIDE_RIGHT_T" in header
    assert "JUNCTION_STANDARD_T" in header
    assert "Camera_GetJunctionError" in header
    assert "Camera_GetJunctionKind" in header
    assert "Camera_GetRouteDecision" in header


def test_route_sequence_is_not_t_junction_specific():
    source = read("code/task2.c")
    header = read("code/task2.h")

    assert "route_sequence" in source
    assert "Task2_GetRouteDecision" in header
    assert "tjun_sequence" not in source


def test_camera_uses_exit_flags_and_three_frame_unlock():
    source = read("code/camera.c")
    header = read("code/camera.h")

    assert "front_exists" in source
    assert "left_exists" in source
    assert "right_exists" in source
    assert "JUNCTION_LEFT_CORNER" in header
    assert "JUNCTION_RIGHT_CORNER" in header
    assert "JUNCTION_LEFT_CORNER" in source
    assert "JUNCTION_RIGHT_CORNER" in source
    assert "EXIT_ROI_LEFT" in source
    assert "EXIT_ROI_RIGHT" in source
    assert "EXIT_ROI_TOP" in source
    assert "EXIT_ROI_BOTTOM" in source
    assert "FRONT_TOP_ROWS" in source
    assert "entry_center" in source
    assert "JUNCTION_MISSING_UNLOCK_FRAMES" in source
    assert "#define JUNCTION_MISSING_UNLOCK_FRAMES 3" in source


def test_old_edge_loss_t_junction_classifier_is_removed():
    source = read("code/camera.c")

    assert "b_flag && t_flag && !l_flag && !r_flag" not in source
    assert "b_flag && t_flag && l_flag && !r_flag" not in source
    assert "b_flag && t_flag && !l_flag && r_flag" not in source
    assert "detect_boundary_sharp_turn(boundary_left, boundary_right)" not in source


def test_route_fill_uses_dynamic_aim_rows():
    source = read("code/camera.c")

    assert "ROUTE_AIM_ADVANCE_ROWS" in source
    assert "left_exit_row" in source
    assert "right_exit_row" in source
    assert "draw_route_line(locked_decision, locked_kind" in source
    assert "end_y = h / 3" not in source
    assert "end_y -= 10" not in source
    assert "entry_center, row_start" in source
