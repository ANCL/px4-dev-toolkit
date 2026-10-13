"""Expose SE(3) comparison metrics, reports, and plots."""

from .model import (
    _AXIS_NAMES,
    _COMPONENTS,
    ComparisonRun,
    build_run,
    trajectory_segments,
)

from .metrics import (
    _ordered_component,
    _interp_component,
    _first_crossing,
    _settling_time,
    _integral_absolute_error,
    _step_axis,
    _step_response_end,
    step_response_metrics,
    yaw_segment_metrics,
    figure_eight_metrics,
    summarize_run,
)

from .reporting import (
    _format_float,
    single_run_summary,
    comparison_summary,
)

from .plotting import (
    _segment_markers,
    _save_scalar_overlay,
    write_comparison_plots,
)
