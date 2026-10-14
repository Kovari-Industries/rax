"""Validate complete motion requests before any simulator RPC can advance physics."""
import numpy as np

from .kinematics import validate_eef_target
from .trajectory import MAX_TRAJECTORY_ACTIONS


class ActionValidationError(ValueError):
    """An agent-correctable rejection with a guarantee that no action was sent."""


def validate_motion_request(name, arguments):
    if name not in {'robodojo_step', 'robodojo_step_eef',
                    'robodojo_free_space_move', 'robodojo_execute_motion_plan'}:
        return
    if not isinstance(arguments, dict):
        raise ActionValidationError('Motion arguments must be an object; no action executed.')
    try:
        if name in {'robodojo_step', 'robodojo_step_eef'}:
            field = 'actions' if name == 'robodojo_step' else 'targets'
            if set(arguments) != {field}:
                raise ValueError(f'{name} requires exactly the {field} field')
            rows = arguments[field]
            if not isinstance(rows, list) or not 1 <= len(rows) <= MAX_TRAJECTORY_ACTIONS:
                count = len(rows) if isinstance(rows, list) else 'a non-list value'
                raise ValueError(f'{field} must contain 1..{MAX_TRAJECTORY_ACTIONS} rows; received {count}. Split longer sequences into calls')
            if field == 'actions':
                matrix = np.asarray(rows, dtype=np.float32)
                if matrix.shape != (len(rows), 14) or not np.isfinite(matrix).all():
                    raise ValueError('Each action must be a finite 14-D vector')
                if np.any((matrix[:, (6, 13)] < 0) | (matrix[:, (6, 13)] > 1)):
                    raise ValueError('Gripper openings must be in [0, 1]')
            else:
                for index, target in enumerate(rows):
                    if not isinstance(target, dict) or set(target) != {'left', 'right'}:
                        raise ValueError(f'targets[{index}] must explicitly contain left and right')
                    for arm in ('left', 'right'):
                        try:
                            validate_eef_target(target[arm], require_gripper=True)
                        except (ValueError, TypeError) as exc:
                            raise ValueError(f'targets[{index}].{arm}: {exc}') from exc
        elif name == 'robodojo_free_space_move':
            if set(arguments) - {'arm', 'target', 'preview_only', 'include_trajectory'}:
                raise ValueError('Unknown free_space_move fields; use arm, target, preview_only and include_trajectory only')
            if arguments.get('arm') not in ('left', 'right'):
                raise ValueError('arm must be left or right')
            if any(type(arguments.get(key, False)) is not bool for key in ('preview_only', 'include_trajectory')):
                raise ValueError('preview_only and include_trajectory must be boolean')
            validate_eef_target(arguments.get('target'))
        elif set(arguments) != {'motion_plan_id'} or not isinstance(arguments.get('motion_plan_id'), str) or not arguments['motion_plan_id']:
            raise ValueError('Provide exactly one nonempty string motion_plan_id')
    except (ValueError, TypeError, OverflowError) as exc:
        raise ActionValidationError(f'{exc}; no action executed.') from exc
