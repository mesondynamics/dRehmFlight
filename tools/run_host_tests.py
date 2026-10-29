#!/usr/bin/env python3
"""Build and run every host C++ regression test without writing into the repo."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sanitize', action='store_true', help='enable address and undefined-behavior sanitizers')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    tests = sorted((root / 'tests').glob('*_test.cpp'))
    compiler = shlex.split(os.environ.get('CXX', 'c++'))
    flags = ['-std=c++11', '-Wall', '-Wextra', '-Werror']
    if args.sanitize:
        flags += ['-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer']
    with tempfile.TemporaryDirectory(prefix='drehmflight-host-') as build:
        for test in tests:
            target = str(Path(build) / test.stem)
            print(f'Running {test.name}', flush=True)
            subprocess.run(compiler + flags + [str(test), '-o', target], check=True)
            subprocess.run([target], check=True)
    print(f'All {len(tests)} host test programs passed', flush=True)


if __name__ == '__main__':
    main()
