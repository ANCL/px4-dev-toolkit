"""Smoke-test stable analysis entry points after module boundaries changed.

These checks verify imports and exported interfaces, not numerical
equivalence. Separate tests cover selected comparison boundaries and laws.
"""

import unittest

from analysis import pipeline
from analysis.profiles import get_profile
from analysis.profiles.se3 import (
    AnalysisResult,
    PROFILE_NAME,
    analyze,
    enrich_summary,
    write_plots,
)
from analysis.profiles.se3.comparison import (
    ComparisonRun,
    build_run,
    comparison_summary,
    figure_eight_metrics,
    single_run_summary,
    step_response_metrics,
    trajectory_segments,
    write_comparison_plots,
)


class AnalysisInterfaceTests(unittest.TestCase):
    def test_profile_registry(self):
        self.assertEqual(PROFILE_NAME, "se3")
        self.assertIs(get_profile("se3").analyze, analyze)
        self.assertTrue(callable(enrich_summary))
        self.assertTrue(callable(write_plots))
        self.assertTrue(hasattr(AnalysisResult, "__dataclass_fields__"))

    def test_pipeline_entry_points(self):
        for name in (
            "CONTROL_PIPELINE_TOPICS", "analyze_control_pipeline",
            "control_pipeline_summary", "write_control_pipeline_plots",
            "_series3", "_attitude_series", "_aligned_error_stats",
        ):
            self.assertTrue(hasattr(pipeline, name), name)

    def test_comparison_entry_points(self):
        self.assertTrue(hasattr(ComparisonRun, "__dataclass_fields__"))
        for method in (
            build_run, trajectory_segments, step_response_metrics,
            figure_eight_metrics, single_run_summary, comparison_summary,
            write_comparison_plots,
        ):
            self.assertTrue(callable(method))

if __name__ == "__main__":
    unittest.main()
