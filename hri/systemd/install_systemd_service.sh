#!/usr/bin/env bash
# install_systemd_service.sh —— 把 hri.service 装到 systemd 用户会话目录
# 用法:
#   sudo loginctl enable-linger $(whoami)
#   bash hri/systemd/install_systemd_service.sh [--enable] [--start]
#
# --enable : systemctl --user enable hri.service（开机自启）
# --start  : 立即启动一次
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${HERE}/hri.service"
DEST_DIR="${HOME}/.config/systemd/user"
DEST="${DEST_DIR}/hri.service"
ENABLE=0
START=0
for arg in "$@"; do
    case "$arg" in
        --enable) ENABLE=1 ;;
        --start)  START=1 ;;
        *) echo "未知参数: $arg" >&2; exit 2 ;;
    esac
done

mkdir -p "${DEST_DIR}"
cp "${SRC}" "${DEST}"
echo "[install] 已复制: ${SRC} -> ${DEST}"

systemctl --user daemon-reload
echo "[install] systemd --user daemon-reload OK"

# 允许开机自启的前置条件（需要 linger 保持用户会话）
if ! loginctl show-user "$(id -u)" -p Linger | grep -q "Linger=yes"; then
    echo "[install] 注意：未启用 linger，用户注销后服务会停止。请管理员执行："
    echo "          sudo loginctl enable-linger $(whoami)"
fi

[ "${ENABLE}" -eq 1 ] && systemctl --user enable hri.service && echo "[install] enable OK"
[ "${START}"  -eq 1 ] && systemctl --user restart hri.service && echo "[install] start OK"

echo
echo "常用命令："
echo "  查看状态: systemctl --user status hri.service"
echo "  看日志  : journalctl --user -u hri.service -f --since today"
echo "  停止    : systemctl --user stop hri.service"
echo "  重启    : systemctl --user restart hri.service"
echo "  禁用自启: systemctl --user disable hri.service"
