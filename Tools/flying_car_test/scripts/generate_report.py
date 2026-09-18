#!/usr/bin/env python3
"""Generate a flying-car experiment report from a trial result directory.

Usage::

    # Single-trial report (writes report.md, plots/, summary.json).
    python3 generate_report.py \
        --result-dir <trial> \
        --config Tools/flying_car_test/config/experiment.yaml \
        --log-rules Tools/flying_car_test/config/log_rules.yaml \
        --output-dir <trial>

    # Aggregate report across three trials (writes aggregate_report.md,
    # aggregate_summary.json).
    python3 generate_report.py \
        --result-dir <trial_a> \
        --result-dir <trial_b> \
        --result-dir <trial_c> \
        --config Tools/flying_car_test/config/experiment.yaml \
        --log-rules Tools/flying_car_test/config/log_rules.yaml \
        --output-dir aggregate

Exit code 0 is returned only when the verdict is PASS.
"""

from flying_car_test.reporting import main


if __name__ == "__main__":
    raise SystemExit(main())