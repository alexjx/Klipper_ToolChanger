"""Deprecated Klipper configuration entry point for ``[ktcclog]``.

The custom logger was removed.  This adapter deliberately has no runtime
behavior; it only consumes deployed options so existing printer configuration
can be used while switching between old and refactored revisions.
"""

import logging


class KtccLogConfigCompatibility:
    """Accept the old section without restoring the old logging subsystem."""

    def __init__(self, config):
        self.log_level = config.getint('log_level', 1, minval=0, maxval=3)
        self.logfile_level = config.getint(
            'logfile_level', 3, minval=-1, maxval=4
        )
        self.log_statistics = config.getint(
            'log_statistics', 0, minval=0, maxval=1
        )
        self.log_visual = config.getint('log_visual', 1, minval=0, maxval=2)
        logging.getLogger('ktcc').info(
            '[ktcclog] is accepted for configuration compatibility; '
            'its custom logging options are inactive.'
        )


def load_config(config):
    return KtccLogConfigCompatibility(config)
