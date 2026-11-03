#!/usr/bin/env python3
"""Validate/import Cyclone RemoteMCU UI profiles; never contact the gateway/MCU."""
import argparse
import json
from pathlib import Path
import re
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[2]
FILES = [ROOT / 'docs/remote-mcu' / name for name in
         ('cyclone-rotor-flight.json', 'cyclone-rotor-device.json')]
PARAMETER_IDS = set(range(1, 19)) | set(range(20, 29)) | set(range(30, 57)) | set(range(60, 100))


def check(condition, message):
    if not condition:
        raise ValueError(message)


def label(value, maximum=128):
    return isinstance(value, str) and 0 < len(value.encode('utf-8')) <= maximum and \
        not any(ord(c) < 32 or 127 <= ord(c) <= 159 for c in value)


def validate(profiles):
    app, device = profiles
    for p in profiles:
        check(set(p) == {'format_version', 'id', 'revision', 'name', 'data'}, 'Unknown profile fields')
        check(p['format_version'] == 1 and re.fullmatch(r'[A-Za-z0-9_-]{1,64}', p['id']), 'Profile identity')
        check(label(p['name']), 'Profile name exceeds Agent UTF-8 limit')
        check(len((json.dumps(p, ensure_ascii=False, indent=2) + '\n').encode()) <= 65536, 'Profile >64KiB')
    a, d = app['data'], device['data']
    check(a['kind'] == 'application' and a['schema_id'] == 1, 'Unsupported application schema')
    check(set(a) == {'kind', 'target_profile', 'schema_id', 'window_seconds', 'wave_layout',
                     'wave_windows', 'channels', 'cards', 'parameters', 'parameter_mode'}, 'Application fields')
    check(a['target_profile'] == 'teensy40-usb', 'Target profile mismatch')
    check(1 <= a['window_seconds'] <= 30 and a['wave_layout'] in ('stack', 'columns'), 'Wave layout')
    for key, ids in [('channels', set(range(32))), ('cards', set(range(1, 47))), ('parameters', PARAMETER_IDS)]:
        items = a[key]
        check(len(items) == len(ids) and {v['id'] for v in items} == ids, f'{key}: IDs/count mismatch')
        check(all(label(v['name']) for v in items), f'{key}: UTF-8 name limit')
    for c in a['channels']:
        check(set(c) == {'id', 'name', 'unit', 'precision', 'color', 'visible', 'scale'}, 'Channel fields')
        check(re.fullmatch(r'#[0-9a-fA-F]{6}', c['color']) and 0 <= c['precision'] <= 9, 'Channel format')
        check(1e-6 <= c['scale'] <= 1e6 and len(c['unit'].encode()) <= 24, 'Channel scale/unit')
    for c in a['cards']:
        check(set(c) <= {'id', 'name', 'unit', 'precision', 'warning_min', 'warning_max', 'enabled'}, 'Card fields')
        check(0 <= c['precision'] <= 9 and len(c['unit'].encode()) <= 24, 'Card format')
        if c.get('warning_min') is not None and c.get('warning_max') is not None:
            check(c['warning_min'] <= c['warning_max'], 'Card warning interval')
    for p in a['parameters']:
        check(set(p) <= {'id', 'name', 'enabled'}, 'Parameter label fields')
    windows = a['wave_windows']
    check(len(windows) == 8 and len({w['id'] for w in windows}) == 8, 'Wave windows limit/identity')
    for w in windows:
        check(set(w) == {'id', 'name', 'channels', 'height', 'window_seconds'}, 'Window fields')
        check(re.fullmatch(r'[A-Za-z0-9_-]{1,64}', w['id']) and label(w['name']), 'Window label')
        check(240 <= w['height'] <= 640 and 1 <= w['window_seconds'] <= 30, 'Window dimensions')
        check(len(w['channels']) <= 32 and len(set(w['channels'])) == len(w['channels']) and
              set(w['channels']) <= set(range(32)), 'Unknown/duplicate waveform channel')
    check(set(d) == {'kind', 'target_profile', 'application_profile', 'gateway_ip'}, 'Device fields')
    check(d == {'kind': 'device', 'target_profile': a['target_profile'],
                'application_profile': app['id'], 'gateway_ip': '192.168.0.62'}, 'Device references')


def exchange(agent, path, payload=None):
    check(path in ('/api/profiles', '/api/profiles/select'), 'Profile-only API')
    body = None if payload is None else json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
    request = Request(agent + path, data=body, headers={
        'Content-Type': 'application/json', 'x-remotemcu-intent': 'write'})
    with urlopen(request, timeout=5) as response:
        return json.load(response)


def same_content(a, b):
    return {k: v for k, v in a.items() if k != 'revision'} == \
        {k: v for k, v in b.items() if k != 'revision'}


def install(agent, profiles, select=False):
    address = urlsplit(agent)
    check(address.scheme == 'http' and address.hostname in ('127.0.0.1', 'localhost', '::1') and
          address.port is not None and not address.username and not address.password and
          address.path in ('', '/') and not address.query and not address.fragment, 'Loopback Agent URL required')
    agent = agent.rstrip('/')
    catalog = exchange(agent, '/api/profiles')
    original_selection = catalog['selection']
    for profile in profiles:  # Application first, then the device referencing it.
        old = next((p for p in catalog['profiles'] if p['id'] == profile['id']), None)
        if old is not None and same_content(old, profile):
            continue
        result = exchange(agent, '/api/profiles', {'profile': profile,
                          'expected_revision': old['revision'] if old else 0})
        check(same_content(result, profile), 'Agent profile readback differs')
    catalog = exchange(agent, '/api/profiles')
    for profile in profiles:
        found = next((p for p in catalog['profiles'] if p['id'] == profile['id']), None)
        check(found is not None and same_content(found, profile), 'Catalog readback differs')
    if select:
        selection = {'application': profiles[0]['id'], 'device': profiles[1]['id']}
        check(exchange(agent, '/api/profiles/select', selection) == selection, 'Selection readback differs')
    else:
        check(catalog['selection'] == original_selection, 'Selection changed concurrently; inspect Web')
    return {'installed': [p['id'] for p in profiles], 'selected': select, 'gateway_contacted': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--agent', default='http://127.0.0.1:47301')
    parser.add_argument('--install', action='store_true', help='Import into local Agent; preserve current selection')
    parser.add_argument('--select', action='store_true', help='Also select new UI AFTER matching firmware is installed')
    args = parser.parse_args()
    try:
        check(not args.select or args.install, '--select requires --install')
        profiles = [json.loads(p.read_text()) for p in FILES]
        validate(profiles)
        result = install(args.agent, profiles, args.select) if args.install else {
            'validation': 'passed', 'channels': 32, 'states': 46, 'parameters': 94,
            'windows': 8, 'agent_contacted': False}
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except (ValueError, KeyError, OSError, HTTPError, URLError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
