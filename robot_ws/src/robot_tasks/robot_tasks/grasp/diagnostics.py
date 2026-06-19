"""Formatting helpers for open-loop grasp diagnostics."""


def fmt_xyz(xyz) -> str:
    if xyz is None:
        return 'None'
    return f'({float(xyz[0]):.3f}, {float(xyz[1]):.3f}, {float(xyz[2]):.3f})'


def build_workspace_rejection_message(
    *,
    reason: str,
    target_base=None,
    attempted_pre_grasp=None,
    attempted_grasp=None,
    current_end_pose=None,
    workspace_limits=None,
    approach_mode=None,
    front_approach_x_offset=None,
) -> str:
    return (
        f'workspace planning rejected: reason={reason}; '
        f'approach_mode={approach_mode}; '
        f'target_base={fmt_xyz(target_base)}; '
        f'attempted_pre_grasp={fmt_xyz(attempted_pre_grasp)}; '
        f'attempted_grasp={fmt_xyz(attempted_grasp)}; '
        f'current_end_pose={fmt_xyz(current_end_pose)}; '
        f'workspace_limits={workspace_limits}; '
        f'front_approach_x_offset={front_approach_x_offset}'
    )


def format_status_summary(
    *,
    state: str,
    executor: str,
    mode=None,
    target=None,
    end=None,
    goal=None,
) -> str:
    return (
        f'[GRASP] state={state}, executor={executor}, mode={mode}, '
        f'target={fmt_xyz(target)}, end={fmt_xyz(end)}, goal={fmt_xyz(goal)}'
    )
