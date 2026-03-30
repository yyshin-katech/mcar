#!/bin/bash
# sync_memory.sh - Claude Code 메모리 파일 동기화 스크립트
# 사용법:
#   ./sync_memory.sh push   : 로컬 메모리 → 레포 (.claude_memory/) 복사
#   ./sync_memory.sh pull   : 레포 (.claude_memory/) → 로컬 메모리 (최신 파일만)
#   ./sync_memory.sh        : 기본값 = pull

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_MEMORY="$REPO_DIR/.claude_memory"
LOCAL_MEMORY="$HOME/.claude/projects/-home-katech-mcar/memory"

mkdir -p "$REPO_MEMORY" "$LOCAL_MEMORY"

sync_pull() {
    local updated=0
    for f in "$REPO_MEMORY"/*; do
        [ -f "$f" ] || continue
        fname="$(basename "$f")"
        local_f="$LOCAL_MEMORY/$fname"

        if [ ! -f "$local_f" ]; then
            cp "$f" "$local_f"
            echo "[PULL] 새 파일 복사: $fname"
            updated=$((updated + 1))
        elif [ "$f" -nt "$local_f" ]; then
            cp "$f" "$local_f"
            echo "[PULL] 업데이트: $fname (레포가 더 최신)"
            updated=$((updated + 1))
        else
            echo "[PULL] 스킵: $fname (로컬이 최신이거나 동일)"
        fi
    done

    # 레포에서 삭제된 파일은 로컬에서도 삭제
    for f in "$LOCAL_MEMORY"/*; do
        [ -f "$f" ] || continue
        fname="$(basename "$f")"
        if [ ! -f "$REPO_MEMORY/$fname" ]; then
            rm "$f"
            echo "[PULL] 삭제: $fname (레포에 없음)"
            updated=$((updated + 1))
        fi
    done

    echo "--- pull 완료: ${updated}개 파일 변경됨 ---"
}

sync_push() {
    local updated=0
    for f in "$LOCAL_MEMORY"/*; do
        [ -f "$f" ] || continue
        fname="$(basename "$f")"
        repo_f="$REPO_MEMORY/$fname"

        if [ ! -f "$repo_f" ]; then
            cp "$f" "$repo_f"
            echo "[PUSH] 새 파일 복사: $fname"
            updated=$((updated + 1))
        elif [ "$f" -nt "$repo_f" ]; then
            cp "$f" "$repo_f"
            echo "[PUSH] 업데이트: $fname (로컬이 더 최신)"
            updated=$((updated + 1))
        else
            echo "[PUSH] 스킵: $fname (레포가 최신이거나 동일)"
        fi
    done

    # 로컬에서 삭제된 파일은 레포에서도 삭제
    for f in "$REPO_MEMORY"/*; do
        [ -f "$f" ] || continue
        fname="$(basename "$f")"
        if [ ! -f "$LOCAL_MEMORY/$fname" ]; then
            rm "$f"
            echo "[PUSH] 삭제: $fname (로컬에 없음)"
            updated=$((updated + 1))
        fi
    done

    echo "--- push 완료: ${updated}개 파일 변경됨 ---"
}

case "${1:-pull}" in
    push) sync_push ;;
    pull) sync_pull ;;
    *)    echo "사용법: $0 [push|pull]"; exit 1 ;;
esac
