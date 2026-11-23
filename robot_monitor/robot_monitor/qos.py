# Copyright 2026 PhysicalFleet
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
QoS profiles as plain data, and publisher/subscriber compatibility checks.

Incompatible QoS is the most common "my subscriber gets nothing" bug in ROS 2,
and ROS only reports it as a one-line warning in the subscriber's log (if at
all). We check every publisher/subscriber pair on every topic.
"""

INFINITE_NS = 0x7FFFFFFFFFFFFFFF


def _enum_name(v):
    name = getattr(v, 'name', None)
    if name:
        return name.lower()
    return str(v).split('.')[-1].lower()


def _dur_ns(d):
    if d is None:
        return None
    ns = getattr(d, 'nanoseconds', None)
    if ns is None:
        try:
            ns = int(d.sec) * 1000000000 + int(d.nsec)
        except AttributeError:
            return None
    # rmw uses several spellings of "infinite"/"default"; normalize to None.
    if ns <= 0 or ns >= INFINITE_NS or ns >= 0x7FFFFFFF * 1000000000:
        return None
    return int(ns)


def qos_to_dict(q):
    """Convert a QoSProfile to a small JSON-friendly dict."""
    if q is None:
        return None
    return {
        'reliability': _enum_name(q.reliability),
        'durability': _enum_name(q.durability),
        'history': _enum_name(q.history),
        'depth': int(getattr(q, 'depth', 0) or 0),
        'deadline_ns': _dur_ns(getattr(q, 'deadline', None)),
        'lifespan_ns': _dur_ns(getattr(q, 'lifespan', None)),
        'liveliness': _enum_name(getattr(q, 'liveliness', 'automatic')),
        'lease_ns': _dur_ns(getattr(q, 'liveliness_lease_duration', None)),
    }


def short(qd):
    """One-word-ish summary of a QoS dict, e.g. 'reliable/volatile/keep_last(10)'."""
    if not qd:
        return '?'
    hist = qd['history']
    if hist == 'keep_last':
        hist = 'keep_last(%d)' % qd['depth']
    elif hist in ('unknown', 'system_default'):
        # Discovered remote endpoints don't report their history setting.
        return '%s/%s' % (qd['reliability'], qd['durability'])
    return '%s/%s/%s' % (qd['reliability'], qd['durability'], hist)


def check(pub, sub):
    """
    Compare two QoS dicts. Returns (level, reason) with level 'ok'|'warn'|'error'.

    Mirrors rmw_qos_profile_check_compatible (request/offered semantics of DDS).
    """
    if not pub or not sub:
        return 'ok', ''
    problems, warnings = [], []
    pr, sr = pub['reliability'], sub['reliability']
    if pr == 'best_effort' and sr == 'reliable':
        problems.append('the subscriber requires RELIABLE delivery but the publisher only offers '
                        'BEST_EFFORT')
    pd, sd = pub['durability'], sub['durability']
    if pd == 'volatile' and sd == 'transient_local':
        problems.append('the subscriber requires TRANSIENT_LOCAL (latched) durability but the '
                        'publisher is VOLATILE')
    pdl, sdl = pub['deadline_ns'], sub['deadline_ns']
    if sdl is not None and (pdl is None or pdl > sdl):
        problems.append('the subscriber requests a deadline of %s but the publisher offers %s'
                        % (_fmt_ns(sdl), _fmt_ns(pdl) if pdl else 'none'))
    pl, sl = pub['liveliness'], sub['liveliness']
    if pl == 'automatic' and sl == 'manual_by_topic':
        problems.append('the subscriber requires MANUAL_BY_TOPIC liveliness but the publisher is '
                        'AUTOMATIC')
    pls, sls = pub['lease_ns'], sub['lease_ns']
    if sls is not None and (pls is None or pls > sls):
        problems.append('the subscriber requests a liveliness lease of %s but the publisher '
                        'offers %s' % (_fmt_ns(sls), _fmt_ns(pls) if pls else 'infinite'))
    for k in ('reliability', 'durability', 'liveliness'):
        if 'unknown' in (pub[k], sub[k]) or 'system_default' in (pub[k], sub[k]):
            warnings.append('%s is %s/%s, so compatibility depends on the middleware default'
                            % (k, pub[k], sub[k]))
    if problems:
        return 'error', '; '.join(problems)
    if warnings:
        return 'warn', '; '.join(warnings)
    return 'ok', ''


def fix_hint(pub, sub, reason):
    if 'RELIABLE' in reason:
        return ('Make the subscriber best-effort (e.g. rclcpp::SensorDataQoS() / '
                'qos_profile_sensor_data), or make the publisher reliable.')
    if 'TRANSIENT_LOCAL' in reason:
        return ('Make the subscriber VOLATILE, or publish with TRANSIENT_LOCAL durability '
                '(QoS(depth=1, durability=TRANSIENT_LOCAL)) if late joiners need the last '
                'message.')
    if 'deadline' in reason:
        return ('Relax the subscriber deadline or set an equal or shorter deadline on the '
                'publisher.')
    if 'liveliness' in reason or 'lease' in reason:
        return 'Align liveliness kind and lease duration between publisher and subscriber.'
    return 'Align the QoS settings of the publisher and the subscriber.'


def _fmt_ns(ns):
    if ns is None:
        return 'none'
    if ns >= 1000000000:
        return '%gs' % (ns / 1e9)
    return '%gms' % (ns / 1e6)
