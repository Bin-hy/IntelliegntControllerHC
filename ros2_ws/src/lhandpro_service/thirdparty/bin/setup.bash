#!/bin/bash

# 获取当前脚本所在的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 定义 lib 目录为脚本所在目录的上一级目录中的 lib 文件夹
LIB_DIR="$SCRIPT_DIR/../lib"

# 将 lib 目录加入 LD_LIBRARY_PATH（如果尚未包含）
if [[ ":$LD_LIBRARY_PATH:" != *":$LIB_DIR:"* ]]; then
    export LD_LIBRARY_PATH="$LIB_DIR:$LD_LIBRARY_PATH"
    echo "✅ 已将 $LIB_DIR 添加到 LD_LIBRARY_PATH"
else
    echo "ℹ️  $LIB_DIR 已存在于 LD_LIBRARY_PATH 中"
fi

# 打印当前环境变量
echo "📌 当前 LD_LIBRARY_PATH = $LD_LIBRARY_PATH"