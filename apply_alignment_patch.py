#!/usr/bin/env python3
"""Apply the L90/R90 trailing-edge alignment patch to the pinned main_test.py.

Place this file, turn_alignment.py, and measure_55mm_interactive.py in the
ArmRobotics_2026 repository root; run `python3 apply_alignment_patch.py`.
Creates a backup and refuses unexpected source versions, without running motors.
"""
from pathlib import Path
import ast
import shutil

MAIN = Path(__file__).with_name('main_test.py')


def replace_exact(source, old, new, label, count=1):
    found=source.count(old)
    if found!=count:
        raise RuntimeError(f'{label}: expected {count} matches, found {found}; source unchanged')
    return source.replace(old,new)


def apply(source):
    source=replace_exact(source,
        'from navigation_core import FreshFrames\n',
        'from navigation_core import FreshFrames\nfrom turn_alignment import align_to_crossbar_center\n',
        'import alignment helper')
    source=replace_exact(source,
        '''    if not follow_for(
        calibration,
        TURN_90_FORWARD_TIME,
        speed=TURN_90_FORWARD_SPEED,
        max_speed=TURN_90_FORWARD_MAX_SPEED,
    ):
        return False
''',
        '''    if not align_to_crossbar_center(calibration):
        return False
''',
        'button first L90')
    source=replace_exact(source,
        '''        if not follow_for(
            calibration,
            TURN_90_FORWARD_TIME,
            speed=TURN_90_FORWARD_SPEED,
            max_speed=TURN_90_FORWARD_MAX_SPEED,
        ):
            return False
''',
        '''        if not align_to_crossbar_center(calibration):
            return False
''',
        'cube first custom 90')
    source=replace_exact(source,
        '''        ok = follow_for(
            calibration,
            TURN_90_FORWARD_TIME,
            speed=TURN_90_FORWARD_SPEED,
            max_speed=TURN_90_FORWARD_MAX_SPEED,
        )
''',
        '''        ok = align_to_crossbar_center(calibration)
''',
        'color R90')
    source=replace_exact(source,
        '''                        ok = follow_for(
                            calibration, TURN_90_FORWARD_TIME,
                            speed=TURN_90_FORWARD_SPEED,
                            max_speed=TURN_90_FORWARD_MAX_SPEED,
                        )
''',
        '''                        ok = align_to_crossbar_center(calibration)
''',
        'route L90/R90')
    source=replace_exact(source,
        '''    if not follow_for(calibration, RETURN_FORWARD_TIME):
        return False
''',
        '''    if not align_to_crossbar_center(calibration):
        return False
''',
        'button reverse R90')
    source=replace_exact(source,
        "        if not follow_for(calibration, RETURN_FORWARD_TIME):\n            return False\n",
        "        if not align_to_crossbar_center(calibration):\n            return False\n",
        'FIRE and CUBE1 reverse R90', count=2)
    return source


def main():
    if not MAIN.exists():
        raise SystemExit('main_test.py not found. Put patch files in repository root.')
    original=MAIN.read_text(encoding='utf-8')
    if 'from turn_alignment import align_to_crossbar_center' in original:
        raise SystemExit('Patch already appears to be installed; no files changed.')
    updated=apply(original)
    ast.parse(updated,filename='main_test.py')
    backup=MAIN.with_name('main_test.py.before_alignment.bak')
    if backup.exists():
        raise SystemExit('Backup already exists; refusing to overwrite.')
    shutil.copy2(MAIN,backup)
    MAIN.write_text(updated,encoding='utf-8')
    print('Patched main_test.py; backup:',backup)
    print('Calibrate at PWM 80 using measure_55mm_interactive.py at two distinct times before L90/R90.')

if __name__=='__main__':
    main()
