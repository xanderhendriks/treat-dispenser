"""idf.py extension: flash over J-Link JTAG via OpenOCD (no BOOT/RESET buttons needed).

Loaded automatically by idf.py because this file sits in the project directory.

Usage:
    idf.py jtag-flash                 # bootloader + partition table + app
    idf.py jtag-flash --app-only      # just the app binary (faster)
    idf.py jtag-flash --jtag-speed 8000
"""

import json
import os
import subprocess

try:
    from idf_py_actions.errors import FatalError
except ImportError:
    FatalError = SystemExit


def action_extensions(base_actions, project_path):
    def jtag_flash(action, ctx, args, jtag_speed, app_only, **kwargs):
        build_dir = args.build_dir
        flasher_args_path = os.path.join(build_dir, 'flasher_args.json')
        if not os.path.exists(flasher_args_path):
            raise FatalError('{} not found; run idf.py build first'.format(flasher_args_path))

        if app_only:
            with open(flasher_args_path) as f:
                flasher_args = json.load(f)
            app = flasher_args['app']
            program_cmd = 'program_esp {} {} verify reset exit'.format(app['file'], app['offset'])
        else:
            program_cmd = 'program_esp_bins . flasher_args.json verify reset exit'

        cmd = [
            'openocd',
            '-f', 'interface/jlink.cfg',
            '-f', 'target/esp32s3.cfg',
            '-c', 'adapter speed {}'.format(jtag_speed),
            '-c', program_cmd,
        ]
        print('Executing: {}'.format(' '.join(cmd)))
        result = subprocess.run(cmd, cwd=build_dir)
        if result.returncode != 0:
            raise FatalError('OpenOCD JTAG flash failed (exit code {})'.format(result.returncode))

    return {
        'actions': {
            'jtag-flash': {
                'callback': jtag_flash,
                'help': 'Flash the project over JTAG (J-Link) using OpenOCD; no BOOT/RESET buttons required.',
                'dependencies': ['all'],
                'options': [
                    {
                        'names': ['--jtag-speed'],
                        'help': 'JTAG adapter speed in kHz (default 4000).',
                        'type': int,
                        'default': 4000,
                    },
                    {
                        'names': ['--app-only'],
                        'help': 'Flash only the app binary, skipping bootloader and partition table.',
                        'is_flag': True,
                        'default': False,
                    },
                ],
            },
        },
    }
