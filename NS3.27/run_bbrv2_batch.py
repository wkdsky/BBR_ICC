#!/usr/bin/env python3
"""
BBRv2 Batch Runner - Advanced Configuration & Execution

This script provides advanced control over BBRv2 test scenarios:
- Dynamic parameter modification
- Parallel execution support
- Comprehensive result analysis
- Configuration file support
"""

import os
import sys
import subprocess
import argparse
import json
import re
import shutil
from pathlib import Path
from datetime import datetime
from collections import defaultdict

# Color codes
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'

class BBRv2BatchRunner:
    def __init__(self, ns3_dir):
        self.ns3_dir = Path(ns3_dir)
        self.scratch_dir = self.ns3_dir / "scratch"
        self.build_dir = self.ns3_dir / "build"
        self.traces_dir = self.ns3_dir / "traces"
        self.waf_header_dir = self.build_dir / "ns3"
        self.results = defaultdict(list)

    @staticmethod
    def resolve_probe_rtt(probe_rtt):
        value = str(probe_rtt).strip().lower()
        if value in {"on", "true", "1", "yes", "enable", "enabled"}:
            return True
        if value in {"off", "false", "0", "no", "disable", "disabled"}:
            return False
        raise ValueError(f"Invalid probeRTT value: {probe_rtt}")

    def print_header(self, title):
        """Print a formatted header"""
        border = "=" * 70
        print(f"\n{Colors.BLUE}{Colors.BOLD}{border}{Colors.END}")
        print(f"{Colors.BLUE}{Colors.BOLD}{title:^70}{Colors.END}")
        print(f"{Colors.BLUE}{Colors.BOLD}{border}{Colors.END}\n")

    def print_info(self, msg):
        print(f"{Colors.CYAN}ℹ {msg}{Colors.END}")

    def print_success(self, msg):
        print(f"{Colors.GREEN}✓ {msg}{Colors.END}")

    def print_error(self, msg):
        print(f"{Colors.RED}✗ {msg}{Colors.END}")

    def print_warning(self, msg):
        print(f"{Colors.YELLOW}⚠ {msg}{Colors.END}")

    def update_script_params(self, num_flows, sender_bw, bottle_bw, bottle_delay):
        """Update C++ constants in the script file"""
        script_file = self.scratch_dir / f"{num_flows}_bbrv2.cc"

        if not script_file.exists():
            self.print_error(f"Script not found: {script_file}")
            return False

        # Backup original
        backup_file = script_file.with_suffix('.cc.orig')
        if not backup_file.exists():
            script_file.rename(backup_file)
            script_file = backup_file

        # Read original content
        with open(backup_file, 'r') as f:
            content = f.read()

        # Update parameters (convert Mbps to bps)
        sender_bps = sender_bw * 1000000
        bottle_bps = bottle_bw * 1000000

        content = re.sub(
            r'const uint64_t TOPO_SENDER_BW\s*=\s*[^;]+;',
            f'const uint64_t TOPO_SENDER_BW       =   {sender_bps};    // in bps',
            content
        )

        content = re.sub(
            r'const uint64_t TOPO_BOTTLE_BW\s*=\s*[^;]+;',
            f'const uint64_t TOPO_BOTTLE_BW       =   {bottle_bps};    // in bps',
            content
        )

        content = re.sub(
            r'const uint64_t TOPO_BOTTLE_PDELAY\s*=\s*[^;]+;',
            f'const uint64_t TOPO_BOTTLE_PDELAY   =   {bottle_delay};    // in ms',
            content
        )

        # Write modified content
        with open(script_file.with_suffix('.cc'), 'w') as f:
            f.write(content)

        self.print_success(f"Updated {num_flows}_bbrv2.cc")
        self.print_info(f"  TOPO_SENDER_BW={sender_bw} Mbps")
        self.print_info(f"  TOPO_BOTTLE_BW={bottle_bw} Mbps")
        self.print_info(f"  TOPO_BOTTLE_PDELAY={bottle_delay} ms")

        return True

    def sync_waf_exported_headers(self):
        self.waf_header_dir.mkdir(parents=True, exist_ok=True)
        headers = [
            self.ns3_dir / "src/dqc/model/thirdparty/include/proto_types.h",
            self.ns3_dir / "src/dqc/model/thirdparty/congestion/quic_bbr2_sender.h",
        ]

        for src in headers:
            dst = self.waf_header_dir / src.name
            if not dst.exists() or src.read_bytes() != dst.read_bytes():
                if dst.exists():
                    dst.chmod(0o644)
                shutil.copy2(src, dst)
                dst.chmod(0o644)

    def build_script(self, num_flows, force=False):
        """Build a specific script"""
        script_name = f"{num_flows}_bbrv2"
        executable = self.build_dir / "scratch" / script_name

        if executable.exists() and not force:
            self.print_info(f"Executable already exists: {script_name}")
            return True

        self.print_info(f"Building {script_name}...")

        try:
            os.chdir(self.ns3_dir)
            self.sync_waf_exported_headers()
            result = subprocess.run(
                [f"./waf", "build", f"--targets={script_name}"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=300
            )

            if result.returncode != 0:
                self.print_error(f"Build failed: {result.stderr.decode()}")
                return False

            self.print_success(f"Built {script_name}")
            return True
        except Exception as e:
            self.print_error(f"Build error: {e}")
            return False

    def run_test(self, num_flows, sender_bw, bottle_bw, bottle_delay, sim_time, probe_rtt):
        """Run a single test"""
        cc_name = "bbrv2" if probe_rtt else "bbrv2_noprobe_rtt"
        probe_tag = "probeRTTon" if probe_rtt else "probeRTToff"
        test_name = f"{num_flows}flows_{sender_bw}mbps_{bottle_bw}mbps_{bottle_delay}ms_{probe_tag}"

        # Update script
        if not self.update_script_params(num_flows, sender_bw, bottle_bw, bottle_delay):
            return False

        # Build
        if not self.build_script(num_flows, force=True):
            return False

        # Run
        self.print_header(f"Running: {test_name} (SIM_TIME={sim_time}s, CC={cc_name})")

        try:
            os.chdir(self.ns3_dir)
            start_time = datetime.now()

            result = subprocess.run(
                ["./waf", "--run", f"{num_flows}_bbrv2 --sim_time={sim_time} --cc={cc_name}"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=600,
                text=True
            )

            duration = (datetime.now() - start_time).total_seconds()

            # Save log
            log_file = self.ns3_dir / f"test_{test_name}.log"
            with open(log_file, 'w') as f:
                f.write(result.stdout)

            if result.returncode == 0:
                self.print_success(f"Test completed in {duration:.1f}s")
                self.results[num_flows].append({
                    'config': f"{sender_bw}Mbps_{bottle_bw}Mbps_{bottle_delay}ms_{probe_tag}",
                    'duration': duration,
                    'status': 'success',
                    'log': str(log_file)
                })
                return True
            else:
                self.print_error(f"Test failed")
                self.results[num_flows].append({
                    'config': f"{sender_bw}Mbps_{bottle_bw}Mbps_{bottle_delay}ms_{probe_tag}",
                    'duration': duration,
                    'status': 'failed',
                    'log': str(log_file)
                })
                return False

        except subprocess.TimeoutExpired:
            self.print_error("Test timeout")
            return False
        except Exception as e:
            self.print_error(f"Test error: {e}")
            return False

    def print_summary(self):
        """Print test summary"""
        self.print_header("Test Summary")

        for num_flows in sorted(self.results.keys()):
            results = self.results[num_flows]
            success_count = sum(1 for r in results if r['status'] == 'success')
            total_count = len(results)

            print(f"{Colors.BOLD}{num_flows} flows:{Colors.END}")
            for result in results:
                status_icon = f"{Colors.GREEN}✓{Colors.END}" if result['status'] == 'success' else f"{Colors.RED}✗{Colors.END}"
                print(f"  {status_icon} {result['config']} ({result['duration']:.1f}s)")

            print(f"  Result: {success_count}/{total_count} passed\n")

    def list_trace_files(self, num_flows=None):
        """List generated trace files"""
        if not self.traces_dir.exists():
            return

        files = list(self.traces_dir.glob("*inflight*.txt"))

        if num_flows:
            print(f"\n{Colors.BOLD}Trace files for {num_flows} flows:{Colors.END}")
        else:
            print(f"\n{Colors.BOLD}Generated trace files:{Colors.END}")

        for f in sorted(files)[:20]:
            size_mb = f.stat().st_size / (1024 * 1024)
            print(f"  {f.name} ({size_mb:.2f} MB)")

        if len(files) > 20:
            print(f"  ... and {len(files) - 20} more files")


def main():
    parser = argparse.ArgumentParser(
        description="BBRv2 Batch Runner - Advanced Configuration & Execution",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run all flows with default parameters
  python3 run_bbrv2_batch.py

  # Run only 8 flows with custom parameters
  python3 run_bbrv2_batch.py --flows 8 --sender-bw 10 --bottle-bw 16 --sim-time 60

  # Run multiple configurations
  python3 run_bbrv2_batch.py --flows 4,8,16 --bottle-delay 50 --sim-time 120

  # Load from config file
  python3 run_bbrv2_batch.py --config config.json
        """
    )

    parser.add_argument('--flows', type=str, default='4,8,16,32',
                        help='Comma-separated flow counts (default: 4,8,16,32)')
    parser.add_argument('--sender-bw', type=int, default=10,
                        help='Edge link bandwidth in Mbps (default: 10)')
    parser.add_argument('--bottle-bw', type=int,
                        help='Bottleneck bandwidth in Mbps (auto if not set)')
    parser.add_argument('--bottle-delay', type=int, default=28,
                        help='Bottleneck delay in ms (default: 28)')
    parser.add_argument('--sim-time', type=int, default=30,
                        help='Simulation duration in seconds (default: 30)')
    parser.add_argument('--probe-rtt', type=str, default='on',
                        help='Enable ProbeRTT: on/off (default: on)')
    parser.add_argument('--config', type=str,
                        help='Load configuration from JSON file')

    args = parser.parse_args()

    # Determine NS3 directory
    ns3_dir = Path(__file__).parent
    if not (ns3_dir / "waf").exists():
        print(f"{Colors.RED}Error: Could not find NS-3 directory{Colors.END}")
        sys.exit(1)

    runner = BBRv2BatchRunner(ns3_dir)

    # Load config if provided
    if args.config:
        try:
            with open(args.config, 'r') as f:
                config = json.load(f)
                args.flows = config.get('flows', args.flows)
                args.sender_bw = config.get('sender_bw', args.sender_bw)
                args.bottle_bw = config.get('bottle_bw', args.bottle_bw)
                args.bottle_delay = config.get('bottle_delay', args.bottle_delay)
                args.sim_time = config.get('sim_time', args.sim_time)
                args.probe_rtt = config.get('probe_rtt', args.probe_rtt)
                runner.print_info(f"Loaded configuration from {args.config}")
        except Exception as e:
            print(f"{Colors.RED}Error loading config: {e}{Colors.END}")
            sys.exit(1)

    try:
        probe_rtt = runner.resolve_probe_rtt(args.probe_rtt)
    except ValueError as e:
        print(f"{Colors.RED}{e}{Colors.END}")
        sys.exit(1)

    cc_name = "bbrv2" if probe_rtt else "bbrv2_noprobe_rtt"

    # Default bottleneck bandwidth
    default_bottle_bw = {4: 8, 8: 16, 16: 32, 32: 64}

    runner.print_header("BBRv2 Batch Runner Configuration")

    flows = [int(f.strip()) for f in args.flows.split(',')]
    print(f"{Colors.BOLD}Configuration:{Colors.END}")
    print(f"  Flows: {args.flows}")
    print(f"  Sender BW: {args.sender_bw} Mbps")
    print(f"  Bottleneck Delay: {args.bottle_delay} ms")
    print(f"  Simulation Time: {args.sim_time} seconds")
    print(f"  ProbeRTT: {'on' if probe_rtt else 'off'} ({cc_name})\n")

    # Run tests
    total_tests = len(flows)
    success_count = 0

    for idx, num_flows in enumerate(flows, 1):
        bottle_bw = args.bottle_bw or default_bottle_bw.get(num_flows, num_flows * 2)

        print(f"{Colors.YELLOW}[{idx}/{total_tests}]{Colors.END} Testing {num_flows} flows...")

        if runner.run_test(num_flows, args.sender_bw, bottle_bw, args.bottle_delay, args.sim_time, probe_rtt):
            success_count += 1

    # Print summary
    runner.print_summary()

    # List trace files
    runner.list_trace_files()

    print(f"\n{Colors.BOLD}Overall Results:{Colors.END}")
    print(f"  Successful: {success_count}/{total_tests}")
    print(f"  Traces: {runner.traces_dir}")
    print()


if __name__ == '__main__':
    main()
