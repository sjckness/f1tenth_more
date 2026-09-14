"""
Per-control-step CSV for offline model validation.

The consumer is tools/mpc_model_check.py, run by hand on a saved log; see
docs/DIAGNOSTICS.md. Nothing in this module is read back by the controller.

Format, fixed by that script: one header row, then one row per solved
control step::

    t,x,y,psi,v,steer_cmd,accel_cmd

What each column means is decided and documented by the caller (MPC_corr.py,
where it constructs the writer). This module only enforces the rules whose
violation would corrupt the analysis silently:

* A row whose t does not advance past the previous row's is SKIPPED. mpc_corr's
  control loop is a timer that solves from the LATEST odometry message; if
  odometry stalls for a tick, the next step solves from the very same message
  and stamp. That row would put dt = 0 into the analysis, which divides by dt.
  Skipping it loses nothing: the interval reappears as a doubled dt on the next
  row, which the script's dt spread still reports.
* Every row is flushed at once, so a crash or Ctrl-C loses nothing.
* A path that cannot be opened, or a write that fails, disables the log
  instead of raising. Logging must never take the controller down with it.
"""

import csv


class ModelLogWriter:
    """CSV writer for the t,x,y,psi,v,steer_cmd,accel_cmd model log."""

    HEADER = ('t', 'x', 'y', 'psi', 'v', 'steer_cmd', 'accel_cmd')

    def __init__(self, path):
        """Open `path` and write the header; an empty path disables logging."""
        self.path = path
        self.skipped = 0
        self.error = None
        self._file = None
        self._writer = None
        self._last_t = None
        if not path:
            return
        try:
            self._file = open(path, 'w', newline='')
            self._writer = csv.writer(self._file)
            self._writer.writerow(self.HEADER)
            self._file.flush()
        except OSError as exc:
            self._fail(exc)

    @property
    def enabled(self):
        """Return True while rows are being written."""
        return self._file is not None

    def write(self, t, x, y, psi, v, steer_cmd, accel_cmd):
        """Append and flush one row. Returns False when nothing was written."""
        if self._file is None:
            return False
        if self._last_t is not None and t <= self._last_t:
            self.skipped += 1
            return False
        try:
            self._writer.writerow(
                [repr(float(value)) for value in (t, x, y, psi, v, steer_cmd, accel_cmd)])
            self._file.flush()
        except OSError as exc:
            self._fail(exc)
            return False
        self._last_t = t
        return True

    def close(self):
        """Close the file; later writes are no-ops."""
        if self._file is not None:
            try:
                self._file.close()
            except OSError:
                pass
            self._file = None

    def _fail(self, exc):
        self.error = exc
        self.close()
