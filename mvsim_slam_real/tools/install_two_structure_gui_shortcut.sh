#!/usr/bin/env bash
# Add a "두 구조물 주차 테스트" launcher to the app menu and desktop (current user only).
set -e
TASK_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
chmod +x "$TASK_WORKSPACE/tools/run_two_structure_gui.sh"
entry="$HOME/.local/share/applications/two-structure-parking-test.desktop"
mkdir -p "$(dirname "$entry")"
cat > "$entry" <<DESKTOP
[Desktop Entry]
Type=Application
Name=두 구조물 주차 테스트
Comment=테스트 모드 선택, 실행/정지, 인식한 주차 공간 보기
Exec=bash -c '"$TASK_WORKSPACE/tools/run_two_structure_gui.sh"'
Path=$TASK_WORKSPACE
Icon=applications-engineering
Terminal=false
Categories=Development;
DESKTOP
chmod +x "$entry"
desktop="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
if [ -d "$desktop" ]; then
  cp "$entry" "$desktop/"
  chmod +x "$desktop/$(basename "$entry")"
  gio set "$desktop/$(basename "$entry")" metadata::trusted true 2>/dev/null || true
fi
echo "Installed: $entry"
