#!/usr/bin/env bash
# 供 launch 的 prefix 使用：用 GDB 包装可执行文件，崩溃时自动打印 bt full
exec gdb -batch -ex run -ex "bt full" -ex quit --args "$@"
