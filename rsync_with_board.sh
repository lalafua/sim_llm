#!/bin/bash

local_dir="$HOME/Workspace/sim_llm/"
remote_usr_name="debian"
remote_ip="10.42.0.11"
remote_dir_path="Workspace/sim_llm/"
remote_target="$remote_usr_name@$remote_ip:$remote_dir_path"

RSYNC_OPTS=("-av" "--progress")

EXCLUDE_OPTS=(
  --exclude '.git/' \
  --exclude 'build/' \
  --exclude '*.o' \
  --exclude '*.a' \
  --exclude '*.so' \
  --exclude '*.elf' \
  --exclude '.vscode/' \
  --exclude '.idea/' \
  --exclude '*.swp' \
)

if [ -z "$1" ]; then
  echo "error: need arguments"
  echo "usage: <push|pull> [--dry-run|-n]"
  exit 1
fi

ACTION=$1
DRY_RUN_OPT=""

if [[ "$2" == "--dry-run" || "$2" == "-n" ]]; then
  DRY_RUN_OPT="-n"
  echo "dry mode..."
fi

declare -a FINAL_OPTS=()           
FINAL_OPTS+=( "${RSYNC_OPTS[@]}" )  

if [[ -n "$DRY_RUN_FLAG" ]]; then
  FINAL_OPTS+=( "$DRY_RUN_FLAG" ) 
fi

FINAL_OPTS+=( "${EXCLUDE_OPTS[@]}" )

case "$ACTION" in
  push)
    echo "pushing to board..."
    rsync "${FINAL_OPTS[@]}" "$local_dir" "$remote_target"
    ;;

  pull)
    echo "pulling from board..."
    rsync "${FINAL_OPTS[@]}" "$local_dir" "$remote_source" 
    ;;
  *)
    echo "error: invailed operation '$ACTION'."
    echo "usage: $0 <push|pull> [--dry-run|-n]"
    exit 1
    ;;
esac

if [ $? -eq 0 ]; then
  echo "operation '$ACTION' success!"
  exit 0
else
  echo "operation '$ACTION' failure!"
  exit 1
fi
