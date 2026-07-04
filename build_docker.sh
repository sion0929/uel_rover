#!/bin/bash
# ─────────────────────────────────────────────────────────────────
#  Docker 이미지 빌드 스크립트
# ─────────────────────────────────────────────────────────────────

ARCH=$(uname -m)

# docker-compose.yml 과 동일하게 uel_rover:jazzy 로 태그 통일
if [ "$ARCH" = "x86_64" ]; then
    echo "[빌드] x86_64 개발용 이미지"
elif [ "$ARCH" = "aarch64" ]; then
    echo "[빌드] ARM64 (Jetson Orin) 이미지 — 네이티브 빌드"
else
    echo "[오류] 지원하지 않는 아키텍처: $ARCH"
    exit 1
fi

docker build -t uel_rover:jazzy .

echo "[완료] 빌드 성공 (uel_rover:jazzy)"
