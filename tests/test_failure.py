"""Failure injection through the command entry point."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np
import pytest


@pytest.mark.parametrize(('mode', 'phase', 'last_step'), [
    ('integration', 'integration', 3),
    ('reference_solver', 'reference_solver', 200),
    ('postprocessing', 'postprocessing', 200),
    ('trajectory', 'artifact_write', 200),
    ('metrics', 'artifact_write', 200),
    ('checkpoint', 'integration', 3),
])
def test_cli_failure_checkpoint(mode, phase, last_step):
    with tempfile.TemporaryDirectory() as d:
        out = Path(d)/'out'
        code = '''
from unittest.mock import patch
import numpy as np
from pathlib import Path
from contextlib import ExitStack
from aether_sil.cli import main
from aether_sil.wrenches import ZeroWrench
mode = MODE
original_evaluate = ZeroWrench.evaluate
original_savez = np.savez
original_write = Path.write_text

def injected_evaluate(self, t, state):
    if t >= .034:
        raise ValueError("injected integration")
    return original_evaluate(self, t, state)

def injected_savez(path, *args, **kwargs):
    if (mode == 'trajectory' and Path(path).name == 'trajectory.npz') or (
       mode == 'checkpoint' and Path(path).name == 'partial_trajectory.npz'):
        raise OSError("injected " + mode)
    return original_savez(path, *args, **kwargs)

def injected_write(self, *args, **kwargs):
    if mode == 'metrics' and self.name == 'metrics.json':
        raise OSError("injected metrics")
    return original_write(self, *args, **kwargs)
with ExitStack() as stack:
    if mode in ('integration', 'checkpoint'):
        stack.enter_context(patch.object(ZeroWrench, 'evaluate', injected_evaluate))
    if mode == 'reference_solver':
        stack.enter_context(patch('aether_sil.cli.solve_reference', side_effect=ValueError('injected reference_solver')))
    if mode == 'postprocessing':
        stack.enter_context(patch('aether_sil.cli.rotation_matrix', side_effect=ValueError('injected postprocessing')))
    stack.enter_context(patch('aether_sil.cli.np.savez', injected_savez))
    stack.enter_context(patch.object(Path, 'write_text', injected_write))
    main(['run', 'scenarios/free_rotation.yaml', '--output', OUTPUT])
'''.replace('OUTPUT', repr(str(out))).replace('MODE', repr(mode))
        result = subprocess.run([sys.executable, '-c', code], env=os.environ.copy(),
                                capture_output=True, text=True)
        assert result.returncode != 0
        status = json.loads((out/'validation.json').read_text())
        assert status['run_status'] == 'FAIL'
        assert status['gate_s0'] == 'NOT EVALUATED'
        assert status['failure_phase'] == phase
        assert status['last_successful_step'] == last_step
        assert status['last_successful_time_s'] == pytest.approx(last_step*.01)
        expected = 'integration' if mode == 'checkpoint' else mode
        assert status['exception_message'] == 'injected ' + expected
        assert 'injected ' + expected in result.stderr
        if phase == 'integration':
            assert status['failure_time_s'] == pytest.approx(.035)
        else:
            assert status['failure_time_s'] is None
            assert status['failure_time_note']
        if mode == 'checkpoint':
            assert status['partial_trajectory'] is None
            assert 'injected checkpoint' in status['checkpoint_error']
            assert 'injected checkpoint' in result.stderr
        else:
            with np.load(out/'partial_trajectory.npz') as trace:
                assert len(trace['t_s']) == last_step+1
                assert trace['t_s'][-1] == pytest.approx(last_step*.01)
                assert trace['p_W_m'].shape == (last_step+1, 3)
