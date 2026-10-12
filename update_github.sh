#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly MAX_FILE_SIZE=$((50 * 1024 * 1024))

info() {
    printf '[INFO] %s\n' "$*"
}

warn() {
    printf '[WARN] %s\n' "$*" >&2
}

die() {
    printf '[ERROR] %s\n' "$*" >&2
    exit 1
}

usage() {
    cat <<'EOF'
用法：
  ./update_github.sh "提交说明"
  ./update_github.sh

不传提交说明时，脚本会交互式询问。
脚本会显示改动、暂存所有未被 .gitignore 排除的改动、二次确认，
然后提交并推送当前分支。
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

if [[ $# -gt 1 ]]; then
    usage >&2
    exit 2
fi

cd "$SCRIPT_DIR"

git rev-parse --is-inside-work-tree >/dev/null 2>&1 \
    || die "当前目录不是 Git 仓库：$SCRIPT_DIR"

branch="$(git branch --show-current)"
[[ -n "$branch" ]] || die "当前处于 detached HEAD，不能自动推送"

remote_url="$(git remote get-url origin 2>/dev/null || true)"
[[ -n "$remote_url" ]] || die "没有配置 origin 远程仓库"

info "仓库：$SCRIPT_DIR"
info "分支：$branch"
info "远程：$remote_url"

# 父仓库只能记录 submodule 的提交指针，无法替 submodule 提交代码。
dirty_submodules="$(
    git submodule foreach --quiet \
        'if [ -n "$(git status --porcelain)" ]; then printf "%s\n" "$sm_path"; fi' \
        2>/dev/null || true
)"
if [[ -n "$dirty_submodules" ]]; then
    warn "以下 submodule 内有未提交改动："
    printf '%s\n' "$dirty_submodules" >&2
    die "请先在 submodule 中单独提交并推送，再运行本脚本"
fi

info "获取远端分支状态……"
git fetch origin "$branch"

if git show-ref --verify --quiet "refs/remotes/origin/$branch"; then
    local_sha="$(git rev-parse HEAD)"
    remote_sha="$(git rev-parse "origin/$branch")"
    merge_base="$(git merge-base HEAD "origin/$branch")"

    if [[ "$local_sha" == "$remote_sha" ]]; then
        :
    elif [[ "$local_sha" == "$merge_base" ]]; then
        die "远端 $branch 有新提交，请先运行：git pull --rebase origin $branch"
    elif [[ "$remote_sha" != "$merge_base" ]]; then
        die "本地与远端历史已分叉，请先人工处理后再推送"
    fi
fi

if [[ -z "$(git status --porcelain)" ]]; then
    info "没有需要提交的改动"
    exit 0
fi

printf '\n当前改动：\n'
git status --short

message="${1:-}"
if [[ -z "$message" ]]; then
    printf '\n请输入本次提交说明：'
    IFS= read -r message
fi
[[ -n "${message//[[:space:]]/}" ]] || die "提交说明不能为空"

git add -A

if git diff --cached --quiet; then
    info "没有可提交的改动（其余文件可能已被 .gitignore 排除）"
    exit 0
fi

unsafe=0
while IFS= read -r -d '' file; do
    lower_file="${file,,}"
    case "$lower_file" in
        .env|*/.env|.env.*|*/.env.*|*.pem|*.key|*/id_rsa|*/id_ed25519)
            warn "疑似敏感文件：$file"
            unsafe=1
            ;;
    esac

    if [[ -f "$file" ]]; then
        file_size="$(stat -c '%s' -- "$file")"
        if ((file_size > MAX_FILE_SIZE)); then
            warn "文件超过 50 MiB：$file"
            unsafe=1
        fi
    fi
done < <(git diff --cached --name-only --diff-filter=ACMR -z)

((unsafe == 0)) || die "发现敏感文件名或大文件；请检查并从暂存区移除后重试"

printf '\n即将提交：\n'
git diff --cached --stat
printf '\n提交说明：%s\n' "$message"
printf '确认提交并推送到 origin/%s？[y/N] ' "$branch"
IFS= read -r confirm

case "$confirm" in
    y|Y|yes|YES|Yes)
        ;;
    *)
        info "已取消；文件仍保留在暂存区，未提交、未推送"
        exit 0
        ;;
esac

git commit -m "$message"
git push -u origin "$branch"

info "更新完成：$(git rev-parse --short HEAD)"
