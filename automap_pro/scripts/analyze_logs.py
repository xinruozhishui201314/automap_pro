#!/usr/bin/env python3
"""
AutoMap Pro 日志分析工具

功能：
1. 统计日志级别分布
2. 搜索特定错误
3. 性能分析
4. Trace追踪
5. 时间范围过滤
"""

import json
import argparse
import re
from datetime import datetime
from collections import defaultdict, Counter
from pathlib import Path
import sys


class LogAnalyzer:
    def __init__(self, log_file):
        self.log_file = Path(log_file)
        self.entries = []
        self.load_logs()
    
    def load_logs(self):
        """加载日志文件"""
        print(f"Loading logs from {self.log_file}...")
        count = 0
        with open(self.log_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                
                try:
                    entry = json.loads(line)
                    self.entries.append(entry)
                    count += 1
                except json.JSONDecodeError:
                    # 跳过非JSON行（可能是启动消息）
                    continue
                
                if count % 10000 == 0:
                    print(f"  Loaded {count} entries...")
        
        print(f"Total entries loaded: {len(self.entries)}")
    
    def get_level_distribution(self):
        """统计日志级别分布"""
        levels = Counter()
        for entry in self.entries:
            level = entry.get('level', 'UNKNOWN')
            levels[level] += 1
        
        return levels
    
    def get_error_summary(self):
        """统计错误摘要"""
        errors = []
        for entry in self.entries:
            if entry.get('level') in ['ERROR', 'FATAL']:
                errors.append(entry)
        
        # 按错误消息分组
        error_groups = defaultdict(list)
        for error in errors:
            msg = error.get('message', 'Unknown')
            error_groups[msg].append(error)
        
        return error_groups
    
    def search_by_pattern(self, pattern, field='message'):
        """按模式搜索"""
        regex = re.compile(pattern, re.IGNORECASE)
        results = []
        
        for entry in self.entries:
            value = entry.get(field, '')
            if regex.search(str(value)):
                results.append(entry)
        
        return results
    
    def get_performance_stats(self):
        """获取性能统计"""
        perf_entries = []
        for entry in self.entries:
            context = entry.get('context', {})
            if isinstance(context, dict) and 'duration_ms' in context:
                perf_entries.append(entry)
        
        stats = defaultdict(list)
        for entry in perf_entries:
            ctx = entry.get('context', {})
            operation = ctx.get('operation', 'Unknown')
            duration = float(ctx.get('duration_ms', 0))
            stats[operation].append(duration)
        
        # 计算统计量
        result = {}
        for op, durations in stats.items():
            durations.sort()
            n = len(durations)
            result[op] = {
                'count': n,
                'min': durations[0],
                'max': durations[-1],
                'mean': sum(durations) / n,
                'median': durations[n//2],
                'p95': durations[int(n*0.95)],
                'p99': durations[int(n*0.99)]
            }
        
        return result
    
    def trace_by_id(self, trace_id):
        """按trace ID追踪"""
        traces = []
        for entry in self.entries:
            ctx = entry.get('context', {})
            if isinstance(ctx, dict) and ctx.get('trace_id') == trace_id:
                traces.append(entry)
        
        return traces
    
    def filter_by_time(self, start_time=None, end_time=None):
        """按时间范围过滤"""
        results = []
        
        if start_time:
            start_ts = datetime.fromisoformat(start_time).timestamp()
        else:
            start_ts = 0
        
        if end_time:
            end_ts = datetime.fromisoformat(end_time).timestamp()
        else:
            end_ts = float('inf')
        
        for entry in self.entries:
            ts_str = entry.get('timestamp', '')
            try:
                ts = datetime.fromisoformat(ts_str).timestamp()
                if start_ts <= ts <= end_ts:
                    results.append(entry)
            except:
                continue
        
        return results
    
    def print_summary(self):
        """打印摘要"""
        print("\n" + "="*80)
        print("LOG SUMMARY")
        print("="*80)
        
        # 时间范围
        timestamps = [e.get('timestamp', '') for e in self.entries if e.get('timestamp')]
        if timestamps:
            print(f"Time range: {timestamps[0]} to {timestamps[-1]}")
        
        # 级别分布
        levels = self.get_level_distribution()
        print(f"\nLog Level Distribution:")
        for level, count in sorted(levels.items()):
            bar = '█' * min(50, int(count * 50 / len(self.entries)))
            print(f"  {level:8s}: {count:6d} ({count/len(self.entries)*100:5.1f}%) {bar}")
        
        # 错误摘要
        error_groups = self.get_error_summary()
        if error_groups:
            print(f"\nError Summary ({len(error_groups)} unique errors):")
            for msg, errors in sorted(error_groups.items(), 
                                    key=lambda x: len(x[1]), reverse=True)[:10]:
                print(f"  [{len(errors)}] {msg[:80]}...")
        
        # 性能统计
        perf_stats = self.get_performance_stats()
        if perf_stats:
            print(f"\nPerformance Statistics:")
            for op, stats in sorted(perf_stats.items(), 
                                   key=lambda x: x[1]['mean'], reverse=True):
                print(f"  {op:40s}: "
                      f"count={stats['count']:5d}, "
                      f"mean={stats['mean']:7.2f}ms, "
                      f"p95={stats['p95']:7.2f}ms, "
                      f"p99={stats['p99']:7.2f}ms")


def main():
    parser = argparse.ArgumentParser(description='AutoMap Pro Log Analyzer')
    parser.add_argument('log_file', help='Path to log file')
    parser.add_argument('--summary', action='store_true', help='Print summary')
    parser.add_argument('--errors', action='store_true', help='Show all errors')
    parser.add_argument('--search', type=str, help='Search pattern')
    parser.add_argument('--field', type=str, default='message', 
                       help='Field to search in (default: message)')
    parser.add_argument('--performance', action='store_true', 
                       help='Show performance statistics')
    parser.add_argument('--trace', type=str, help='Trace by trace ID')
    parser.add_argument('--start-time', type=str, help='Start time (ISO format)')
    parser.add_argument('--end-time', type=str, help='End time (ISO format)')
    
    args = parser.parse_args()
    
    if not Path(args.log_file).exists():
        print(f"Error: Log file not found: {args.log_file}", file=sys.stderr)
        sys.exit(1)
    
    analyzer = LogAnalyzer(args.log_file)
    
    if args.summary:
        analyzer.print_summary()
    
    if args.errors:
        error_groups = analyzer.get_error_summary()
        print(f"\nAll Errors ({len(error_groups)} unique errors):")
        print("="*80)
        for msg, errors in sorted(error_groups.items(),
                                 key=lambda x: len(x[1]), reverse=True):
            print(f"\n[{len(errors)} occurrences] {msg}")
            for error in errors[:3]:  # Show first 3
                print(f"  at {error.get('timestamp', '')}")
                if 'context' in error:
                    ctx = error['context']
                    if isinstance(ctx, dict):
                        if 'source_file' in ctx:
                            print(f"    location: {ctx['source_file']}:{ctx.get('source_line', '?')}")
    
    if args.search:
        results = analyzer.search_by_pattern(args.search, args.field)
        print(f"\nSearch results for '{args.search}' in '{args.field}': {len(results)} matches")
        print("="*80)
        for result in results[:20]:  # Show first 20
            print(f"[{result.get('level', '?')}] {result.get('message', '')}")
    
    if args.performance:
        perf_stats = analyzer.get_performance_stats()
        print(f"\nPerformance Statistics:")
        print("="*80)
        for op, stats in sorted(perf_stats.items(),
                               key=lambda x: x[1]['mean'], reverse=True):
            print(f"\n{op}:")
            print(f"  count:     {stats['count']}")
            print(f"  min:       {stats['min']:.2f} ms")
            print(f"  max:       {stats['max']:.2f} ms")
            print(f"  mean:      {stats['mean']:.2f} ms")
            print(f"  median:    {stats['median']:.2f} ms")
            print(f"  p95:       {stats['p95']:.2f} ms")
            print(f"  p99:       {stats['p99']:.2f} ms")
    
    if args.trace:
        traces = analyzer.trace_by_id(args.trace)
        print(f"\nTrace for ID: {args.trace}")
        print("="*80)
        for entry in sorted(traces, key=lambda x: x.get('timestamp', '')):
            print(f"[{entry.get('timestamp', '')}] "
                  f"[{entry.get('level', '?')}] "
                  f"{entry.get('message', '')}")


if __name__ == '__main__':
    main()
