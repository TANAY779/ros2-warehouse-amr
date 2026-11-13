# Copyright 2026 Tanay Baisware
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

r"""
Measure what a simulation run costs this computer, and how fast Gazebo runs.

Usage::

    ros2 run warehouse_system_tests measure_load [--world warehouse] [--duration 60]
        [--interval 2] [--out DIR] [--quiet]

Start it once the simulation is up, in a second terminal set up like the simulation's (the same
ROS_DOMAIN_ID and GZ_PARTITION). It is not a ROS node: a node on sim time receives every
/clock message, 1000 a second at the 1 ms physics step, and the measurement would pay for
that itself.

Every --interval seconds (default 2) it reads:

1. **CPU per process group**, in cores (CPU seconds per second: 1.0 is one of the 12 threads
   fully busy), from utime + stime in /proc/<pid>/stat, the scheduler's exact count for the
   whole process. Groups are recognised by their command line (GROUPS, group_of). A process
   counts only if its ROS_DOMAIN_ID and GZ_PARTITION equal this tool's, so another simulation
   on the same computer ends up in "rest of the machine". A process that exits between two
   samples loses the CPU time of its last interval.
2. **The machine's total**, from /proc/schedstat: the nanoseconds each CPU spent running tasks,
   also split into the P-cores and E-cores of a hybrid Intel CPU. /proc/stat would sample busy
   time on the timer tick, and Gazebo's 1 kHz loop beats against it
   (docs/research/PC_ASSESSMENT.md, section 3).
3. **Memory**: MemAvailable and the swap in use (/proc/meminfo), and the PSS of each group
   (/proc/<pid>/smaps_rollup: a shared library counts once, split between its users).
4. **Clocks and heat**: the mean clock of the P- and E-cores (cpufreq) and the package
   temperature (thermal zone x86_pkg_temp).
5. **Gazebo's statistics**, from the gz-transport topic /world/<world>/stats (through the
   Python bindings gz.transport13 and gz.msgs10, not ROS): sim time, real time and iterations.
   Their changes over each interval give the real-time factor (RTF = change of sim time /
   change of real time) and the physics step itself (change of sim time / change of
   iterations).

It also notices a suspend (the boot clock runs on while the computer sleeps, the monotonic
clock does not) and MemAvailable below 1.5 GiB; the run rules void a run with either.

Output: a line per sample (unless --quiet) and a summary table at the end. With --out DIR:
DIR/load.jsonl (one JSON object per sample), DIR/load_summary.json and DIR/load_summary.txt
(the table).

Exit codes: 0 the duration was sampled with Gazebo's statistics; 1 the measurement does not
count (no Gazebo statistics, a suspend, or MemAvailable below 1.5 GiB); 2 a bad argument or
--out cannot be written; 130 stopped early by Ctrl-C or SIGTERM (the summary covers the samples
taken).
"""

import argparse
from dataclasses import dataclass
import datetime
import json
import math
import os
import signal
import statistics
import sys
import threading
import time

from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node

DEFAULT_INTERVAL = 2.0           # s between samples, as in the PC assessment
STATS_WAIT = 5.0                 # s to wait for Gazebo's first statistics before sampling
RTF_WINDOW = 10.0                # s: the worst RTF over this long (plan section 3.7)
MIN_MEM_AVAILABLE = 1536.0       # MiB: below this, stop the run (the run rules)
SUSPEND_GAP = 1.0                # s the boot clock may run ahead of the monotonic clock
CLK_TCK = os.sysconf('SC_CLK_TCK')   # the unit of utime, stime and starttime in /proc

# The process groups, in table order. A process joins the first group whose rule in
# group_of() it matches; processes that match none are not part of the run.
GROUPS = (
    ('gazebo_server', 'Gazebo server'),
    ('gazebo_window', 'Gazebo window'),
    ('bridges', 'ros_gz bridges'),
    ('nav2_container', 'nav2_container'),
    ('filter_container', 'filter_container'),
    ('ekf', 'EKF'),
    ('robot_state_publisher', 'robot_state_publisher'),
    ('twist_mux', 'twist_mux'),
    ('localization', 'AMCL, map server, SLAM'),
    ('rviz', 'RViz'),
    ('tools', 'Python tools'),
    ('recorder', 'bag recorder'),
    ('launch', 'ros2 launch'),
    ('other_ros', 'other ROS processes'),
)
SELF = 'measure_load'            # this tool, reported apart from the run
LABELS = dict(GROUPS, **{SELF: 'measure_load (this tool)'})

# Plain processes of the localization (Phase 2); from S2a, AMCL runs in nav2_container.
LOCALIZATION = {'amcl', 'map_server', 'lifecycle_manager', 'async_slam_toolbox_node',
                'sync_slam_toolbox_node', 'localization_slam_toolbox_node'}
# Packages whose Python executables count as tools (drive_route, teleop_window, ...).
TOOL_PACKAGES = ('warehouse_', 'teleop_twist_keyboard')
# The environment variables that tell one simulation run from another.
RUN_VARIABLES = ('ROS_DOMAIN_ID', 'GZ_PARTITION')


# ----------------------------------------------------------------------------- processes


def group_of(words):
    """
    Return the group of a process from its command line, split into words.

    Returns a key of GROUPS, SELF, or None for a process that is not part of a simulation run.
    """
    if not words:
        return None              # a kernel thread
    # gz titles its process 'gz sim <arguments>'. With -s it is the server; without it, the
    # process forks 'gz sim server' and 'gz sim gui' and waits for them.
    gz = _arguments_of(words, 'gz')
    if gz is not None and gz[:1] == ['sim']:
        if gz[1:2] == ['gui'] or '-g' in gz:
            return 'gazebo_window'
        return 'gazebo_server'
    ros2 = _arguments_of(words, 'ros2')
    if ros2 is not None:
        if ros2[:2] == ['bag', 'record']:
            return 'recorder'
        if ros2[:1] == ['launch']:
            return 'launch'
        if ros2[:1] == ['run'] and len(ros2) >= 3:
            # The `ros2 run` process waits for the program it started: same group.
            if ros2[2] == 'measure_load':
                return SELF
            return 'tools' if ros2[1].startswith(TOOL_PACKAGES) else 'other_ros'
        return 'other_ros'       # ros2 topic, ros2 param, ...
    program = words[0]
    script = os.path.basename(program).startswith('python') and len(words) > 1
    if script:
        program = words[1]       # a Python program: the script names it
    name = os.path.basename(program)
    package = os.path.basename(os.path.dirname(program))
    if name == 'measure_load':
        return SELF
    if package in ('ros_gz_bridge', 'ros_gz_image'):
        return 'bridges'
    if name.startswith('component_container'):
        container = next((word.partition(':=')[2] for word in words
                          if word.startswith('__node:=')), '')
        return container if container in ('nav2_container', 'filter_container') else 'other_ros'
    if name == 'ekf_node':
        return 'ekf'
    if name in ('robot_state_publisher', 'twist_mux'):
        return name
    if name in LOCALIZATION:
        return 'localization'
    if name == 'rviz2':
        return 'rviz'
    if script and package.startswith(TOOL_PACKAGES):
        return 'tools'
    # Controller spawners, ros_gz_sim create, and the ros2 daemon that the first ros2 command
    # starts (python3 -c "from ros2cli.daemon.daemonize import main; ..." --name ros2-daemon).
    if '--ros-args' in words or 'ros2-daemon' in words:
        return 'other_ros'
    return None


def _arguments_of(words, program):
    """Return the words after ``program`` if it is the first or second word, else None."""
    for index, word in enumerate(words[:2]):
        if os.path.basename(word) == program:
            return words[index + 1:]
    return None


def read_stat(pid):
    """
    Return (comm, utime + stime, starttime) from /proc/<pid>/stat.

    Times are in clock ticks (CLK_TCK per second); starttime counts from boot. Returns None if
    the process has gone.
    """
    try:
        with open(f'/proc/{pid}/stat') as stream:
            text = stream.read()
    except OSError:
        return None
    # Field 2, the name, is in parentheses and may hold spaces or ')': split at the last ')'.
    head, _, tail = text.rpartition(')')
    fields = tail.split()
    # fields[0] is field 3 (state); utime is field 14, stime 15 and starttime 22 (proc(5)).
    return head.partition('(')[2], int(fields[11]) + int(fields[12]), int(fields[19])


def command_words(pid):
    """Return a process's command line as words ([] for a kernel thread or a gone process)."""
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as stream:
            data = stream.read()
    except OSError:
        return []
    # Arguments are separated by NUL bytes. A process that retitles itself (gz does) writes
    # one string with spaces, padded with NULs: split on both.
    return data.decode(errors='replace').replace('\0', ' ').split()


def environment_of(pid):
    """Return the environment a process started with as a dict, or None if it is unreadable."""
    try:
        with open(f'/proc/{pid}/environ', 'rb') as stream:
            data = stream.read()
    except OSError:
        return None              # another user's process, or gone
    variables = {}
    for entry in data.split(b'\0'):
        name, equals, value = entry.partition(b'=')
        if equals:
            variables[name.decode(errors='replace')] = value.decode(errors='replace')
    return variables


def pss_mib(pid):
    """Return a process's proportional set size (PSS) in MiB, 0 if it cannot be read."""
    try:
        with open(f'/proc/{pid}/smaps_rollup') as stream:
            for line in stream:
                if line.startswith('Pss:'):
                    return int(line.split()[1]) / 1024.0
    except OSError:
        pass
    return 0.0


# ----------------------------------------------------------------------------- the machine


def cpu_list(text):
    """Return the CPU numbers of a list such as '0-3,8' as a set."""
    cpus = set()
    for part in text.strip().split(','):
        if part:
            first, _, last = part.partition('-')
            cpus.update(range(int(first), int(last or first) + 1))
    return cpus


def core_types():
    """
    Return {cpu number: 'P' or 'E'} on a hybrid Intel CPU, or {} on any other.

    The kernel lists the performance cores in /sys/devices/cpu_core/cpus and the efficiency
    cores in /sys/devices/cpu_atom/cpus (this laptop: 0-3 and 4-11).
    """
    types = {}
    for kind, path in (('P', '/sys/devices/cpu_core/cpus'), ('E', '/sys/devices/cpu_atom/cpus')):
        try:
            with open(path) as stream:
                types.update(dict.fromkeys(cpu_list(stream.read()), kind))
        except OSError:
            return {}
    return types


def cpu_running_ns():
    """Return {cpu number: nanoseconds spent running tasks} from /proc/schedstat."""
    running = {}
    with open('/proc/schedstat') as stream:
        for line in stream:
            if line.startswith('cpu'):
                fields = line.split()
                # The 7th number after the name: the time tasks ran on this CPU (ns).
                running[int(fields[0][3:])] = int(fields[7])
    return running


def meminfo_mib():
    """Return (MemAvailable, swap in use) in MiB from /proc/meminfo."""
    values = {}
    with open('/proc/meminfo') as stream:
        for line in stream:
            name, _, rest = line.partition(':')
            values[name] = int(rest.split()[0]) / 1024.0       # kB
    return values['MemAvailable'], values['SwapTotal'] - values['SwapFree']


def read_text(path):
    """Return the stripped text of a small file, or None if it cannot be read."""
    try:
        with open(path) as stream:
            return stream.read().strip()
    except OSError:
        return None


def package_temperature_file():
    """Return the file holding the CPU package temperature (m°C), or None."""
    base = '/sys/class/thermal'
    for zone in sorted(os.listdir(base)) if os.path.isdir(base) else ():
        if read_text(os.path.join(base, zone, 'type')) == 'x86_pkg_temp':
            return os.path.join(base, zone, 'temp')
    return None


def power_state():
    """Return the power supply and the CPU's energy-performance preference, as text."""
    base = '/sys/class/power_supply'
    supplies = sorted(os.listdir(base)) if os.path.isdir(base) else []
    mains = [read_text(os.path.join(base, name, 'online')) for name in supplies
             if read_text(os.path.join(base, name, 'type')) == 'Mains']
    supply = 'unknown supply' if not mains else ('on mains' if '1' in mains else 'on battery')
    preference = read_text('/sys/devices/system/cpu/cpu0/cpufreq/energy_performance_preference')
    if preference is None:
        return supply
    return f'{supply}; energy-performance preference {preference}'


# ----------------------------------------------------------------------------- Gazebo


@dataclass(frozen=True)
class Stats:
    """One message of /world/<world>/stats: times in seconds."""

    sim_time: float
    real_time: float
    iterations: int
    paused: bool


def seconds(time_message):
    """Return a gz.msgs.Time as seconds."""
    return time_message.sec + time_message.nsec * 1e-9


class GazeboStatistics:
    """The newest message on Gazebo's /world/<world>/stats, received through gz-transport."""

    def __init__(self, world):
        self.topic = f'/world/{world}/stats'
        self._newest = None
        self._lock = threading.Lock()
        # Kept as a member: the subscription ends when the node is destroyed.
        self._node = Node()
        if not self._node.subscribe(WorldStatistics, self.topic, self._on_message):
            raise RuntimeError(f'cannot subscribe to the gz topic {self.topic}')

    def _on_message(self, message):
        # Runs in a gz-transport thread, hence the lock. Gazebo publishes about 5 per second.
        stats = Stats(seconds(message.sim_time), seconds(message.real_time),
                      message.iterations, message.paused)
        with self._lock:
            self._newest = stats

    def newest(self):
        """Return the newest Stats, or None before the first message."""
        with self._lock:
            return self._newest


# ----------------------------------------------------------------------------- sampling


@dataclass(frozen=True, eq=False)
class Snapshot:
    """The counters read at one moment; a sample is the difference of two snapshots."""

    monotonic: float             # s (CLOCK_MONOTONIC: stops while the computer is suspended)
    boottime: float              # s (CLOCK_BOOTTIME: runs on while it is suspended)
    wall: datetime.datetime
    cpu_ticks: dict              # (pid, starttime, comm) -> (group, utime + stime in ticks)
    counts: dict                 # group -> processes
    pss: dict                    # group -> PSS [MiB]
    running_ns: dict             # cpu number -> ns spent running tasks
    mem_available: float         # MiB
    swap_used: float             # MiB
    clocks: dict                 # 'P' / 'E' / 'all' -> mean clock [GHz]
    temperature: float | None    # CPU package [°C]
    stats: Stats | None          # Gazebo's newest statistics


class Sampler:
    """Reads Snapshots of this computer and of the processes of this run."""

    def __init__(self, gazebo):
        self.gazebo = gazebo
        self.own_pid = os.getpid()
        self.run = {name: os.environ.get(name) for name in RUN_VARIABLES}
        self.core_types = core_types()
        self.temperature_file = package_temperature_file()
        self._groups = {}        # (pid, starttime, comm) -> group or None, for live processes

    def snapshot(self):
        """Read every counter now; return a Snapshot."""
        monotonic = time.clock_gettime(time.CLOCK_MONOTONIC)
        boottime = time.clock_gettime(time.CLOCK_BOOTTIME)
        groups, cpu_ticks, counts, pss = {}, {}, {}, {}
        for entry in os.scandir('/proc'):
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            stat = read_stat(pid)
            if stat is None:
                continue         # it has just exited
            comm, ticks, start = stat
            # pid, start time and name: a reused pid or an exec gives a new key.
            key = (pid, start, comm)
            group = self._groups[key] if key in self._groups else self._classify(pid)
            groups[key] = group
            if group is None:
                continue
            cpu_ticks[key] = (group, ticks)
            counts[group] = counts.get(group, 0) + 1
            pss[group] = pss.get(group, 0.0) + pss_mib(pid)
        self._groups = groups    # forgets the processes that have gone
        mem_available, swap_used = meminfo_mib()
        temperature = read_text(self.temperature_file) if self.temperature_file else None
        return Snapshot(
            monotonic=monotonic, boottime=boottime, wall=datetime.datetime.now(),
            cpu_ticks=cpu_ticks, counts=counts, pss=pss, running_ns=cpu_running_ns(),
            mem_available=mem_available, swap_used=swap_used, clocks=self._clocks(),
            temperature=float(temperature) / 1000.0 if temperature else None,
            stats=self.gazebo.newest())

    def _classify(self, pid):
        """Return the group of a new process, or None if it is not part of this run."""
        if pid == self.own_pid:
            return SELF
        group = group_of(command_words(pid))
        if group is None:
            return None
        # Part of this run only with this tool's ROS_DOMAIN_ID and GZ_PARTITION: a simulation
        # in another domain is someone else's run.
        environment = environment_of(pid)
        if environment is None:
            return None
        ours = all(environment.get(name) == value for name, value in self.run.items())
        return group if ours else None

    def _clocks(self):
        """Return the mean current clock of each core type in GHz."""
        clocks = {}
        for cpu in range(os.cpu_count() or 0):
            khz = read_text(f'/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_cur_freq')
            if khz:
                clocks.setdefault(self.core_types.get(cpu, 'all'), []).append(int(khz) / 1e6)
        return {kind: statistics.fmean(values) for kind, values in clocks.items()}


def sample(start, previous, current, core_kinds):
    """
    Return the sample between two Snapshots as a dict (one line of load.jsonl).

    ``start`` is the monotonic time of the first snapshot; ``core_kinds`` maps cpu numbers to
    'P' or 'E' ({} on a CPU without core types).
    """
    interval = current.monotonic - previous.monotonic
    cores = {}
    for key, (group, ticks) in current.cpu_ticks.items():
        if key in previous.cpu_ticks:
            used = ticks - previous.cpu_ticks[key][1]
        elif key[1] / CLK_TCK > previous.boottime:
            used = ticks         # started during this interval: all its time is in it
        else:
            continue             # existed before but was not part of the run then
        cores[group] = cores.get(group, 0.0) + used / CLK_TCK / interval
    run = sum(value for group, value in cores.items() if group != SELF)
    busy = {cpu: current.running_ns[cpu] - previous.running_ns.get(cpu, current.running_ns[cpu])
            for cpu in current.running_ns}
    machine = sum(busy.values()) / 1e9 / interval
    record = {
        't': round(current.monotonic - start, 3),
        'time': current.wall.isoformat(timespec='seconds'),
        'interval': round(interval, 3),
        'cores': {group: round(value, 3) for group, value in cores.items()},
        'processes': dict(current.counts),
        'pss_mib': {group: round(value, 1) for group, value in current.pss.items()},
        'run_cores': round(run, 3),
        'self_cores': round(cores.get(SELF, 0.0), 3),
        'machine_cores': round(machine, 3),
        'rest_cores': round(machine - run - cores.get(SELF, 0.0), 3),
    }
    # The machine total split by core type: p_cores and e_cores (hybrid CPUs only).
    for kind in ('P', 'E'):
        if kind in core_kinds.values():
            record[f'{kind.lower()}_cores'] = round(sum(
                ns for cpu, ns in busy.items() if core_kinds.get(cpu) == kind) / 1e9 / interval, 3)
    record.update({
        'mem_available_mib': round(current.mem_available, 1),
        'swap_used_mib': round(current.swap_used, 1),
        'clock_ghz': {kind: round(value, 3) for kind, value in current.clocks.items()},
        'package_c': current.temperature,
        # The computer slept for this long during the interval.
        'suspended_s': round(max(0.0, (current.boottime - previous.boottime) - interval), 3),
    })
    record.update(_gazebo_fields(previous.stats, current.stats))
    return record


def _gazebo_fields(previous, current):
    """Return the Gazebo part of a sample: the statistics, RTF and step (None if unknown)."""
    fields = {'sim_time': None, 'real_time': None, 'iterations': None, 'paused': None,
              'sim_s': None, 'real_s': None, 'rtf': None, 'step': None}
    if current is None:
        return fields
    fields.update(sim_time=round(current.sim_time, 6), real_time=round(current.real_time, 6),
                  iterations=current.iterations, paused=current.paused)
    if previous is None:
        return fields
    simulated = current.sim_time - previous.sim_time
    real = current.real_time - previous.real_time
    iterations = current.iterations - previous.iterations
    # Paused (nothing moved), no new message, or Gazebo restarted: no RTF for this interval.
    if simulated > 0.0 and real > 0.0 and iterations > 0:
        fields.update(sim_s=round(simulated, 6), real_s=round(real, 6),
                      rtf=round(simulated / real, 4), step=round(simulated / iterations, 7))
    return fields


def sample_line(record):
    """Return one sample as a short line for the terminal."""
    if record['rtf'] is not None:
        gazebo = f'RTF {record["rtf"]:.3f}  step {record["step"] * 1000:.3f} ms'
    elif record['paused']:
        gazebo = 'Gazebo paused'
    else:
        gazebo = 'no Gazebo statistics'
    temperature = '' if record['package_c'] is None else f'  {record["package_c"]:.0f} °C'
    return (f'{record["t"]:6.0f} s  {gazebo}  Gazebo server '
            f'{record["cores"].get("gazebo_server", 0.0):.2f}  run {record["run_cores"]:.2f}  '
            f'machine {record["machine_cores"]:.2f} cores  MemAvailable '
            f'{record["mem_available_mib"] / 1024.0:.2f} GiB{temperature}')


# ----------------------------------------------------------------------------- summary


def _mean(records, value):
    """Return the mean of value(record) over the records, weighted by their intervals."""
    total = sum(record['interval'] for record in records)
    return sum(value(record) * record['interval'] for record in records) / total


def worst_rtf(records, window):
    """Return the lowest RTF over ``window`` seconds of consecutive samples (None if none)."""
    worst = None
    for first in range(len(records)):
        simulated = real = 0.0
        for record in records[first:]:
            if record['rtf'] is None:
                break
            simulated += record['sim_s']
            real += record['real_s']
            if real >= window - 1e-6:
                worst = simulated / real if worst is None else min(worst, simulated / real)
                break
    return worst


def summarize(records, info):
    """Return the summary of the samples as a dict (load_summary.json)."""
    summary = dict(info)
    summary['samples'] = len(records)
    summary['duration'] = round(sum(record['interval'] for record in records), 3)
    if records:
        summary.update(_summarize_samples(records))
    summary['warnings'] = warnings(summary)
    return summary


def _summarize_samples(records):
    """Return the summary values that need at least one sample."""
    summary = {}
    groups = {}
    for group, label in list(GROUPS) + [(SELF, LABELS[SELF])]:
        counts = [record['processes'].get(group, 0) for record in records]
        if any(counts):
            values = [record['cores'].get(group, 0.0) for record in records]
            groups[group] = {
                'label': label,
                'processes': max(counts),
                'cores_mean': round(_mean(records, lambda r: r['cores'].get(group, 0.0)), 3),
                'cores_max': round(max(values), 3),
                'pss_mib_max': round(max(r['pss_mib'].get(group, 0.0) for r in records), 1),
            }
    summary['groups'] = groups
    for name in ('run_cores', 'self_cores', 'machine_cores', 'rest_cores', 'p_cores',
                 'e_cores'):
        if name in records[0]:
            summary[f'{name}_mean'] = round(_mean(records, lambda r: r[name]), 3)
            summary[f'{name}_max'] = round(max(record[name] for record in records), 3)
    summary['run_pss_mib_max'] = round(max(
        sum(value for group, value in record['pss_mib'].items() if group != SELF)
        for record in records), 1)
    summary['mem_available_min_mib'] = min(record['mem_available_mib'] for record in records)
    summary['swap_used_max_mib'] = max(record['swap_used_mib'] for record in records)
    summary['clock_ghz_mean'] = {
        kind: round(statistics.fmean(r['clock_ghz'][kind] for r in records
                                     if kind in r['clock_ghz']), 2)
        for kind in records[0]['clock_ghz']}
    temperatures = [r['package_c'] for r in records if r['package_c'] is not None]
    summary['package_c_mean'] = round(statistics.fmean(temperatures), 1) if temperatures else None
    summary['package_c_max'] = max(temperatures) if temperatures else None
    summary['suspended_s'] = round(sum(record['suspended_s'] for record in records), 3)

    timed = [record for record in records if record['rtf'] is not None]
    summary['gazebo_samples'] = len(timed)
    summary['paused_samples'] = sum(1 for record in records if record['paused'])
    if timed:
        steps = [record['step'] for record in timed]
        summary.update(
            rtf_mean=round(sum(r['sim_s'] for r in timed) / sum(r['real_s'] for r in timed), 4),
            rtf_lowest=min(record['rtf'] for record in timed),
            rtf_worst_window=_round(worst_rtf(records, RTF_WINDOW), 4),
            rtf_window_s=RTF_WINDOW,
            step_median=statistics.median(steps), step_min=min(steps), step_max=max(steps))
    return summary


def _round(value, digits):
    return None if value is None else round(value, digits)


def warnings(summary):
    """Return what makes the measurement not count, one sentence each."""
    found = []
    if not summary.get('gazebo_samples'):
        found.append(f'no statistics from Gazebo on {summary["topic"]}: is Gazebo running '
                     f'world:={summary["world"]}, with this GZ_PARTITION '
                     f'({summary["gz_partition"] or "unset"})?')
    if summary.get('mem_available_min_mib', math.inf) < MIN_MEM_AVAILABLE:
        found.append(f'MemAvailable fell to {summary["mem_available_min_mib"]:.0f} MiB, below '
                     f'{MIN_MEM_AVAILABLE / 1024.0:g} GiB: the run rules stop such a run')
    if summary.get('suspended_s', 0.0) > SUSPEND_GAP:
        found.append(f'the computer was suspended for {summary["suspended_s"]:.0f} s during '
                     'the run: a run with a suspend does not count')
    return found


def summary_text(summary):
    """Return the summary as a table for people (load_summary.txt)."""
    started = summary['started'].replace('T', ' ')
    samples = summary['samples']
    lines = [f'measure_load: {summary["duration"]:.0f} s from {started}, {samples} '
             f'sample{"" if samples == 1 else "s"} every {summary["interval"]:g} s; ROS_DOMAIN_ID '
             f'{summary["ros_domain_id"] or "unset"}, GZ_PARTITION '
             f'{summary["gz_partition"] or "unset"}']
    if summary.get('gazebo_samples'):
        step = f'{summary["step_median"] * 1000:.3f} ms'
        if summary['step_max'] - summary['step_min'] > 1e-6:
            step += (f' (from {summary["step_min"] * 1000:.3f} to '
                     f'{summary["step_max"] * 1000:.3f} ms)')
        window = summary['rtf_worst_window']
        window = '-' if window is None else f'{window:.3f}'
        lines.append(f'Gazebo world {summary["world"]} ({summary["topic"]}): RTF '
                     f'{summary["rtf_mean"]:.3f} mean, {window} worst '
                     f'{summary["rtf_window_s"]:g} s, {summary["rtf_lowest"]:.3f} lowest '
                     f'{summary["interval"]:g} s; physics step {step} (change of sim time / '
                     'change of iterations)')
        if summary['paused_samples']:
            lines.append(f'  Gazebo was paused in {summary["paused_samples"]} samples')
    if not summary['samples']:
        lines.append('no samples')
        return '\n'.join(lines)
    lines += ['', f'{"group":28s} {"processes":>9s} {"cores mean":>11s} {"cores max":>10s} '
                  f'{"PSS MiB":>8s}']
    for group, values in summary['groups'].items():
        if group != SELF:
            lines.append(f'{values["label"]:28s} {values["processes"]:9d} '
                         f'{values["cores_mean"]:11.2f} {values["cores_max"]:10.2f} '
                         f'{values["pss_mib_max"]:8.0f}')
    lines.append(f'{"run total":28s} {"":9s} {summary["run_cores_mean"]:11.2f} '
                 f'{summary["run_cores_max"]:10.2f} {summary["run_pss_mib_max"]:8.0f}')
    if SELF in summary['groups']:
        values = summary['groups'][SELF]
        lines.append(f'{values["label"]:28s} {values["processes"]:9d} '
                     f'{values["cores_mean"]:11.2f} {values["cores_max"]:10.2f} '
                     f'{values["pss_mib_max"]:8.0f}')
    lines.append(f'{"rest of the machine":28s} {"":9s} {summary["rest_cores_mean"]:11.2f} '
                 f'{summary["rest_cores_max"]:10.2f}')
    machine = (f'{"machine total":28s} {"":9s} {summary["machine_cores_mean"]:11.2f} '
               f'{summary["machine_cores_max"]:10.2f}')
    if 'p_cores_mean' in summary:
        machine += (f'   (P-cores {summary["p_cores_mean"]:.2f}, E-cores '
                    f'{summary["e_cores_mean"]:.2f})')
    lines += [machine, '']
    lines.append(f'memory: MemAvailable lowest {summary["mem_available_min_mib"]:.0f} MiB; '
                 f'swap in use at most {summary["swap_used_max_mib"]:.0f} MiB')
    clocks = ', '.join(f'{"all cores" if kind == "all" else kind + "-cores"} {ghz:.1f} GHz'
                       for kind, ghz in summary['clock_ghz_mean'].items()) or 'unknown'
    heat = ('' if summary['package_c_mean'] is None else
            f'; package {summary["package_c_mean"]:.0f} °C mean, '
            f'{summary["package_c_max"]:.0f} °C max')
    lines.append(f'clocks: {clocks} (mean){heat}')
    lines.append(f'power: {summary["power"]}')
    lines += [f'WARNING: {warning}' for warning in summary['warnings']]
    return '\n'.join(lines)


# ----------------------------------------------------------------------------- the command


def _interrupt(signum, frame):
    """End the sampling on SIGTERM the same way as on Ctrl-C."""
    raise KeyboardInterrupt


def _positive(text):
    value = float(text)
    if not value > 0.0:
        raise argparse.ArgumentTypeError(f'{text} is not above 0')
    return value


def main(argv=None):
    """Run the measure_load command; return its exit code."""
    parser = argparse.ArgumentParser(
        prog='measure_load',
        description='Sample the CPU per process group, the machine total, memory, clocks and '
                    "heat, and Gazebo's real-time factor and physics step, every few seconds; "
                    'print a summary table at the end.')
    parser.add_argument('--world', default='warehouse',
                        help='the Gazebo world, for /world/<world>/stats (default %(default)s)')
    parser.add_argument('--duration', type=_positive, default=60.0, metavar='SECONDS',
                        help='how long to sample, in seconds (default %(default)g)')
    parser.add_argument('--interval', type=_positive, default=DEFAULT_INTERVAL,
                        metavar='SECONDS',
                        help='seconds between samples (default %(default)g)')
    parser.add_argument('--out', metavar='DIR',
                        help='write load.jsonl, load_summary.json and load_summary.txt there')
    parser.add_argument('--quiet', action='store_true',
                        help='print only the summary, not a line per sample')
    args = parser.parse_args(argv)
    if args.interval > args.duration:
        parser.error('--interval is longer than --duration')

    try:
        gazebo = GazeboStatistics(args.world)
    except RuntimeError as error:
        print(f'error: {error}', file=sys.stderr)
        return 2
    jsonl = None
    if args.out:
        try:
            os.makedirs(args.out, exist_ok=True)
            # 'x': never overwrite the samples of an earlier run.
            jsonl = open(os.path.join(args.out, 'load.jsonl'), 'x')
        except OSError as error:
            print(f'error: cannot write {args.out}/load.jsonl: {error.strerror} (choose another '
                  '--out for a new run)', file=sys.stderr)
            return 2
    signal.signal(signal.SIGTERM, _interrupt)

    sampler = Sampler(gazebo)
    info = {'world': args.world, 'topic': gazebo.topic, 'interval': args.interval,
            'ros_domain_id': sampler.run['ROS_DOMAIN_ID'],
            'gz_partition': sampler.run['GZ_PARTITION'], 'power': power_state()}
    print(f'measure_load: every {args.interval:g} s for {args.duration:g} s; Gazebo statistics '
          f'on {gazebo.topic}; processes with ROS_DOMAIN_ID {info["ros_domain_id"] or "unset"} '
          f'and GZ_PARTITION {info["gz_partition"] or "unset"}; {info["power"]}', flush=True)

    records, interrupted = [], False
    try:
        # Wait for Gazebo's first statistics (discovery takes a moment), so the first sample
        # already has an RTF.
        deadline = time.monotonic() + STATS_WAIT
        while gazebo.newest() is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if gazebo.newest() is None:
            print(f'warning: no message on {gazebo.topic} after {STATS_WAIT:g} s: is Gazebo '
                  f'running world:={args.world} with this GZ_PARTITION? Sampling the CPU only',
                  file=sys.stderr, flush=True)
        previous = sampler.snapshot()
        start = previous.monotonic
        info['started'] = previous.wall.isoformat(timespec='seconds')
        for number in range(1, int(args.duration / args.interval + 1e-9) + 1):
            time.sleep(max(0.0, start + number * args.interval
                           - time.clock_gettime(time.CLOCK_MONOTONIC)))
            current = sampler.snapshot()
            record = sample(start, previous, current, sampler.core_types)
            records.append(record)
            if jsonl:
                jsonl.write(json.dumps(record) + '\n')
                jsonl.flush()
            if not args.quiet:
                print(sample_line(record), flush=True)
            if record['mem_available_mib'] < MIN_MEM_AVAILABLE:
                print(f'warning: MemAvailable {record["mem_available_mib"]:.0f} MiB is below '
                      f'{MIN_MEM_AVAILABLE / 1024.0:g} GiB: stop the run', file=sys.stderr,
                      flush=True)
            previous = current
    except KeyboardInterrupt:
        interrupted = True
    finally:
        if jsonl:
            jsonl.close()
    info.setdefault('started', datetime.datetime.now().isoformat(timespec='seconds'))

    summary = summarize(records, info)
    summary['stopped_early'] = interrupted
    text = summary_text(summary)
    print(('\n' if not args.quiet else '') + text)
    if args.out:
        try:
            with open(os.path.join(args.out, 'load_summary.json'), 'w') as stream:
                json.dump(summary, stream, indent=2)
                stream.write('\n')
            with open(os.path.join(args.out, 'load_summary.txt'), 'w') as stream:
                stream.write(text + '\n')
        except OSError as error:
            print(f'error: cannot write the summary to {args.out}: {error.strerror}',
                  file=sys.stderr)
            return 2
        print(f'wrote {args.out}/load.jsonl, load_summary.json and load_summary.txt')
    if interrupted:
        print(f'stopped early after {summary["duration"]:.0f} s', file=sys.stderr)
        return 130
    return 1 if summary['warnings'] else 0


if __name__ == '__main__':
    sys.exit(main())
