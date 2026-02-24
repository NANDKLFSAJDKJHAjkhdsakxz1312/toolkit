#!/bin/bash
# 文件名: ethercat_launcher.sh

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BUILD_DIR="$SCRIPT_DIR/build"  # 设置build目录路径
EXEC_FILE="ws_ZDL_EC_MoveCapture"

# 显示状态栏
show_status() {
    clear
    echo "=========================================="
    echo "    EtherCAT 控制系统"
    echo "=========================================="
    echo "  服务状态: $(systemctl is-active ethercat 2>/dev/null || echo '停止')"
    echo "  工作目录: $BUILD_DIR"
    echo "  目标程序: $EXEC_FILE"
    echo "=========================================="
    echo ""
}

# 主程序
show_status

echo "1. 重启EtherCAT服务"
sudo systemctl restart ethercat
sleep 2
show_status

echo "2. 启动实时监控"
echo "   (监控将在新终端或后台运行)"
echo ""


    # 后台运行监控
echo "将在本终端后台运行监控，程序输出会混杂"
echo "按 Ctrl+C 停止监控"
(while true; do clear; echo "=== 实时监控 ==="; date; ethercat slaves; sleep 1; done) &
MON_PID=$!
sleep 2

echo "3. 启动测试程序"
echo "-----------------------------------"
cd "$BUILD_DIR" && ./"$EXEC_FILE"

echo ""
echo "程序执行完毕"
[ ! -z "$MON_PID" ] && kill $MON_PID 2>/dev/null
