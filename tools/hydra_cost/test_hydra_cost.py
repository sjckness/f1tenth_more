"""hydra_cost runs and writes its CSV and summary (no ROS, no Hydra needed).

    python3 -m pytest tools/hydra_cost/test_hydra_cost.py -q
"""

import csv
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def test_sampler_writes_csv_and_summary(tmp_path):
    out = tmp_path / 'run'
    # a child that matches --match and burns a little CPU, so a proc row must appear
    burner = subprocess.Popen([sys.executable, '-c',
                               'import time\nt=time.time()\nwhile time.time()-t<6: pass',
                               'hydra_cost_test_burner'])
    try:
        r = subprocess.run([sys.executable, os.path.join(HERE, 'hydra_cost.py'), '--no-ros',
                            '--duration', '3', '--out-dir', str(out),
                            '--match', 'hydra_cost_test_burner'],
                           capture_output=True, text=True, timeout=60)
    finally:
        burner.kill()
    assert r.returncode == 0, r.stderr
    with open(out / 'samples.csv', newline='') as f:
        rows = list(csv.DictReader(f))
    kinds = {row['kind'] for row in rows}
    assert {'system', 'proc'} <= kinds, kinds
    burn = [float(row['value']) for row in rows if row['kind'] == 'proc'
            and 'hydra_cost_test_burner' in row['entity'] and row['metric'] == 'cpu_pct']
    assert burn and max(burn) > 20.0, burn          # a busy loop is most of one core
    assert max(float(row['t_rel']) for row in rows) >= 3.0
    md = (out / 'summary.md').read_text()
    assert '# hydra_cost summary' in md and 'hydra_cost_test_burner' in md
    assert os.path.islink(tmp_path / 'latest')
