#!/usr/bin/env python3
"""
LiDAR 回环检测性能基准测试
评估不同数据集与配置下的性能指标
"""

import re
import sys
from pathlib import Path
from collections import defaultdict

def parse_log_file(log_path):
    """解析日志文件，提取性能指标"""
    metrics = {
        'descriptor_times': [],
        'retrieve_times': [],
        'total_times': [],
        'candidates_per_query': [],
        'queries_with_candidates': 0,
        'total_queries': 0,
    }
    
    with open(log_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            # 提取描述子计算耗时
            match = re.search(r'\[PERF\].*Descriptor.*?(\d+\.\d+)ms', line)
            if match:
                metrics['descriptor_times'].append(float(match.group(1)))
            
            # 提取检索耗时
            match = re.search(r'retrieve_time=(\d+\.\d+)ms', line)
            if match:
                metrics['retrieve_times'].append(float(match.group(1)))
            
            # 提取总耗时
            match = re.search(r'total_time=(\d+\.\d+)ms', line)
            if match:
                metrics['total_times'].append(float(match.group(1)))
            
            # 统计候选数量
            match = re.search(r'candidates=(\d+)', line)
            if match:
                cands = int(match.group(1))
                if cands > 0:
                    metrics['queries_with_candidates'] += 1
                metrics['total_queries'] += 1
                metrics['candidates_per_query'].append(cands)
    
    return metrics

def compute_stats(values):
    """计算统计信息"""
    if not values:
        return None
    
    sorted_vals = sorted(values)
    n = len(values)
    
    return {
        'min': min(values),
        'max': max(values),
        'mean': sum(values) / n,
        'median': sorted_vals[n // 2],
        'p95': sorted_vals[int(n * 0.95)],
        'p99': sorted_vals[int(n * 0.99)],
    }

def print_report(metrics, hz=20):
    """打印性能报告"""
    target_ms = 1000 / hz
    
    print("\n" + "="*70)
    print(f"LiDAR 回环检测性能报告 (目标频率: {hz}Hz = {target_ms:.1f}ms)")
    print("="*70)
    
    print(f"\n📊 总体统计:")
    print(f"  - 总查询数: {metrics['total_queries']}")
    print(f"  - 有候选的查询: {metrics['queries_with_candidates']} ({100*metrics['queries_with_candidates']/max(1, metrics['total_queries']):.1f}%)")
    
    # 描述子计算耗时
    print(f"\n⏱️  描述子计算耗时:")
    if metrics['descriptor_times']:
        stats = compute_stats(metrics['descriptor_times'])
        print(f"  - 最小: {stats['min']:.2f}ms")
        print(f"  - 平均: {stats['mean']:.2f}ms")
        print(f"  - 中位数: {stats['median']:.2f}ms")
        print(f"  - P95: {stats['p95']:.2f}ms")
        print(f"  - P99: {stats['p99']:.2f}ms")
        print(f"  - 最大: {stats['max']:.2f}ms")
        if stats['mean'] > target_ms * 0.7:
            print(f"  ⚠️  警告: 平均耗时 ({stats['mean']:.2f}ms) 接近目标 ({target_ms:.1f}ms)")
    
    # 检索耗时
    print(f"\n🔍 检索候选耗时:")
    if metrics['retrieve_times']:
        stats = compute_stats(metrics['retrieve_times'])
        print(f"  - 最小: {stats['min']:.2f}ms")
        print(f"  - 平均: {stats['mean']:.2f}ms")
        print(f"  - 中位数: {stats['median']:.2f}ms")
        print(f"  - P95: {stats['p95']:.2f}ms")
        print(f"  - P99: {stats['p99']:.2f}ms")
        print(f"  - 最大: {stats['max']:.2f}ms")
    
    # 总耗时
    print(f"\n🎯 总耗时（Stage 1）:")
    if metrics['total_times']:
        stats = compute_stats(metrics['total_times'])
        print(f"  - 最小: {stats['min']:.2f}ms")
        print(f"  - 平均: {stats['mean']:.2f}ms (占目标 {100*stats['mean']/target_ms:.0f}%)")
        print(f"  - 中位数: {stats['median']:.2f}ms")
        print(f"  - P95: {stats['p95']:.2f}ms")
        print(f"  - P99: {stats['p99']:.2f}ms")
        print(f"  - 最大: {stats['max']:.2f}ms")
        
        # 性能评判
        if stats['mean'] <= target_ms * 0.5:
            print(f"  ✅ 优秀：平均耗时 {stats['mean']:.2f}ms ({100*stats['mean']/target_ms:.0f}%)")
        elif stats['mean'] <= target_ms * 0.8:
            print(f"  ✅ 良好：平均耗时 {stats['mean']:.2f}ms ({100*stats['mean']/target_ms:.0f}%)")
        elif stats['mean'] <= target_ms:
            print(f"  ⚠️  可接受：平均耗时 {stats['mean']:.2f}ms ({100*stats['mean']/target_ms:.0f}%)")
        else:
            print(f"  ❌ 过慢：平均耗时 {stats['mean']:.2f}ms ({100*stats['mean']/target_ms:.0f}%) 超过目标")
    
    # 候选分布
    print(f"\n📈 每查询候选数分布:")
    if metrics['candidates_per_query']:
        stats = compute_stats(metrics['candidates_per_query'])
        print(f"  - 最小: {stats['min']:.0f}")
        print(f"  - 平均: {stats['mean']:.1f}")
        print(f"  - 中位数: {stats['median']:.0f}")
        print(f"  - 最大: {stats['max']:.0f}")
    
    print("\n" + "="*70)

def main():
    if len(sys.argv) < 2:
        print("用法: python3 benchmark_loop_closure.py <log_file> [hz]")
        print("示例: python3 benchmark_loop_closure.py test.log 20")
        sys.exit(1)
    
    log_path = sys.argv[1]
    hz = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    
    if not Path(log_path).exists():
        print(f"❌ 日志文件不存在: {log_path}")
        sys.exit(1)
    
    print(f"📖 解析日志文件: {log_path}")
    metrics = parse_log_file(log_path)
    
    if not metrics['total_queries']:
        print("❌ 未找到任何查询数据")
        sys.exit(1)
    
    print_report(metrics, hz)

if __name__ == '__main__':
    main()
