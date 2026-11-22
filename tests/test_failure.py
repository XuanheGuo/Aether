"""Failure preservation through the CLI entry point."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np


def test_cli_failure_checkpoint():
    with tempfile.TemporaryDirectory() as d:
        out = Path(d)/'out'
        code = '''
from unittest.mock import patch
from aether_sil.cli import main
from aether_sil.wrenches import ZeroWrench
original = ZeroWrench.evaluate
def injected(self, t, state):
    if t >= .034:
        raise ValueError("injected solver failure")
    return original(self, t, state)
with patch.object(ZeroWrench, "evaluate", injected):
    main(["run", "scenarios/free_rotation.yaml", "--output", OUTPUT])
'''.replace('OUTPUT', repr(str(out)))
        result = subprocess.run([sys.executable, '-c', code], env=os.environ.copy(),
                                capture_output=True, text=True)
        assert result.returncode != 0
        status = json.loads((out/'validation.json').read_text())
        assert status['run_status'] == 'FAIL'
        assert status['exception_type'] == 'ValueError'
        assert status['exception_message'] == 'injected solver failure'
        assert status['last_successful_step'] == 3
        assert status['last_successful_time_s'] == .03
        assert status['failure_step_start_s'] == .03
        assert abs(status["failure_time_s"] - .035) < 1e-14
        with np.load(out/'partial_trajectory.npz') as trace:
            assert len(trace['t_s']) == 4
            assert trace['t_s'][-1] == .03
        assert not (out/'metrics.json').exists()
