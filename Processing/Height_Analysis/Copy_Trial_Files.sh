#!/usr/bin/env bash
#
# copy_trial_files.sh
#
# Recursively finds every *.csv and trial_info.txt file under SRC_DIR
# (no matter how the intermediate subfolder names are spelled) and
# copies them to DEST_DIR. .db3 files are never touched.
#
# By default the relative folder structure is preserved under DEST_DIR,
# which avoids collisions since "trial_info.txt" is the same filename
# in every trial folder. Pass --flatten to instead dump everything into
# a single flat DEST_DIR, with each filename prefixed by its parent
# folder name to keep them unique.
#
# Usage:
#   ./copy_trial_files.sh SRC_DIR DEST_DIR [--flatten] [--dry-run] [--move]
#
# Examples:
#   ./copy_trial_files.sh /run/media/joab/Seagate/roughness/Joab_Characterisation/Flat_Error ~/collected_trials
#   ./copy_trial_files.sh /run/media/joab/Seagate/roughness/Joab_Characterisation/Flat_Error ~/collected_trials --flatten
#   ./copy_trial_files.sh /run/media/joab/Seagate/roughness/Joab_Characterisation/Flat_Error ~/collected_trials --dry-run

set -euo pipefail

SRC_DIR=""
DEST_DIR=""
FLATTEN=0
DRY_RUN=0
DO_MOVE=0

# --- parse args ---
for arg in "$@"; do
    case "$arg" in
        --flatten) FLATTEN=1 ;;
        --dry-run) DRY_RUN=1 ;;
        --move)    DO_MOVE=1 ;;
        -h|--help)
            grep '^#' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            if [[ -z "$SRC_DIR" ]]; then
                SRC_DIR="$arg"
            elif [[ -z "$DEST_DIR" ]]; then
                DEST_DIR="$arg"
            else
                echo "Unexpected argument: $arg" >&2
                exit 1
            fi
            ;;
    esac
done

if [[ -z "$SRC_DIR" || -z "$DEST_DIR" ]]; then
    echo "Usage: $0 SRC_DIR DEST_DIR [--flatten] [--dry-run] [--move]" >&2
    exit 1
fi

if [[ ! -d "$SRC_DIR" ]]; then
    echo "Source directory does not exist: $SRC_DIR" >&2
    exit 1
fi

# Normalize (strip trailing slash) so relative-path math below is clean
SRC_DIR="${SRC_DIR%/}"

mkdir -p "$DEST_DIR"

CMD="cp"
[[ "$DO_MOVE" -eq 1 ]] && CMD="mv"

count=0

# -iname "trial_info.txt"  -> case-insensitive exact match on that filename
# -iname "*.csv"           -> any csv, case-insensitive, anywhere under SRC_DIR
# We explicitly do NOT match *.db3, so it's simply never selected.
while IFS= read -r -d '' file; do
    rel_path="${file#"$SRC_DIR"/}"          # path relative to SRC_DIR
    parent_name="$(basename "$(dirname "$file")")"
    base_name="$(basename "$file")"

    if [[ "$FLATTEN" -eq 1 ]]; then
        target="$DEST_DIR/${parent_name}__${base_name}"
    else
        target="$DEST_DIR/$rel_path"
        mkdir -p "$(dirname "$target")"
    fi

    if [[ "$DRY_RUN" -eq 1 ]]; then
        echo "[dry-run] $CMD '$file' -> '$target'"
    else
        $CMD "$file" "$target"
        echo "$CMD '$file' -> '$target'"
    fi
    count=$((count + 1))
done < <(find "$SRC_DIR" -type f \( -iname "*.csv" -o -iname "trial_info.txt" \) -print0)

echo "Done. $count file(s) processed."
