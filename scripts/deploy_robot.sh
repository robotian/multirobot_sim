#!/bin/bash
# Deploy colcon_ws/src to a real robot and build it there, as the overlay on top of the robot's own workspaces.
#
#   scripts/deploy_robot.sh a300_00036                 # dry run, ask if anything would be deleted, sync, build
#   scripts/deploy_robot.sh a300_00036 --dry-run       # only list what would change on the robot
#   scripts/deploy_robot.sh j100_0921 --host 192.168.50.70 --yes -- --packages-select mtu32_bringup
#   scripts/deploy_robot.sh a300_00036 --pull          # copy edits made on the robot back into colcon_ws/src
#
# Options: --host H (default cpr-<id with - for _>.local, the robot's mDNS name), --user U (default robot),
# --dry-run, --yes (don't ask before deleting), --allow-dirty (deploy uncommitted changes), --no-build,
# --pull / --force (files edited on the robot since the last deploy, below); arguments after `--` go to colcon build.
#
# Edits on the robot: each deploy saves a checksum list of what it put in ~/colcon_ws/src
# (~/colcon_ws/.deployed_manifest). A deploy first compares the robot's files with it and stops if any were
# changed, added or removed there. --pull copies those back into colcon_ws/src (a file also changed here since
# the deploy is saved next to it as <file>.robot instead, a removal is applied only if unchanged here) and
# deploys nothing; review with git diff (commit in the submodule first), then deploy. --force overwrites them.
#
# Layout on the robot (see scripts/CLAUDE.md, "Deploying to a real robot"):
#   ~/robot_ws    the robot's own packages (drivers, arm, cameras: whatever differs per robot), built by hand;
#                 a COLCON_IGNORE in each package this repo provides
#   ~/colcon_ws   exactly this repo's colcon_ws/src (rsync --delete: nothing else belongs there), built here
# Both are listed in the robot's /etc/clearpath/robot.yaml system.ros2.workspaces, robot_ws first; the build
# sources every workspace listed before ~/colcon_ws as its underlay. ~/colcon_ws/DEPLOYED records the commit.
# Nothing is restarted: restart clearpath-robot / your launch afterwards.
set -euo pipefail
cd "$(dirname "$0")/.."

usage() { sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//' >&2; exit 1; }

id="" host="" user=robot dry=0 yes=0 dirty_ok=0 build=1 pull=0 force=0
while [ $# -gt 0 ]; do
    case "$1" in
        --host) host=$2; shift 2 ;;
        --user) user=$2; shift 2 ;;
        --dry-run) dry=1; shift ;;
        --yes) yes=1; shift ;;
        --allow-dirty) dirty_ok=1; shift ;;
        --no-build) build=0; shift ;;
        --pull) pull=1; shift ;;
        --force) force=1; shift ;;
        --) shift; break ;;
        -h|--help) usage ;;
        -*) echo "unknown option $1" >&2; usage ;;
        *) [ -z "$id" ] || usage; id=$1; shift ;;
    esac
done
[[ "$id" =~ ^[a-z0-9]+_[0-9]+$ ]] || usage
host=${host:-cpr-${id//_/-}.local}
target="$user@$host"
ssh_() { ssh -o BatchMode=yes -o ConnectTimeout=10 "$target" "$@"; }
[ $((pull + force)) -le 1 ] || { echo "--pull or --force, not both" >&2; exit 1; }

# "<sha1>  ./<path>" for every file under src, the same on both sides (.git and __pycache__ skipped like rsync's)
MANIFEST='find . \( -name .git -o -name __pycache__ \) -prune -o -type f -print0 | LC_ALL=C sort -z | xargs -0r sha1sum'
SAVED=.deployed_manifest

ssh_ true || { echo "cannot ssh to $target (key login: ssh-copy-id $target)" >&2; exit 1; }
echo "== $id: $target"

# files edited on the robot since the last deploy: "M|A|D <path>" (changed, added, removed there)
drift=""
if [ $force = 0 ]; then
    if ssh_ "test -f ~/colcon_ws/$SAVED"; then
        drift=$(awk 'NF { p = substr($0, 43); sub(/^\.\//, "", p); if (NR == FNR) s[p] = $1; else c[p] = $1 }
                     END { for (p in s) if (!(p in c)) print "D " p; else if (s[p] != c[p]) print "M " p
                           for (p in c) if (!(p in s)) print "A " p }' \
                    <(ssh_ "cat ~/colcon_ws/$SAVED") <(ssh_ "cd ~/colcon_ws/src && $MANIFEST") | sort -k2)
    elif [ $pull = 1 ]; then
        echo "no ~/colcon_ws/$SAVED on the robot (deployed before the edit check): compare by hand with" >&2
        echo "  rsync -rlnci --exclude .git --exclude __pycache__ $target:colcon_ws/src/ colcon_ws/src/" >&2
        exit 1
    fi
fi

if [ $pull = 1 ]; then
    [ -n "$drift" ] || { echo "nothing was edited on the robot since the last deploy"; exit 0; }
    declare -A saved
    while read -r h p; do saved[${p#./}]=$h; done < <(ssh_ "cat ~/colcon_ws/$SAVED")
    here() { [ -f "colcon_ws/src/$1" ] && sha1sum "colcon_ws/src/$1" | cut -c1-40 || echo none; }
    tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
    grep -v '^D ' <<<"$drift" | cut -c3- > "$tmp/list" || true
    [ ! -s "$tmp/list" ] || rsync -lpt --files-from="$tmp/list" -e "ssh -o BatchMode=yes" "$target:colcon_ws/src/" "$tmp/src/"
    conflicts=0
    while read -r kind p; do
        unchanged_here=0; [ "$(here "$p")" = "${saved[$p]:-none}" ] && unchanged_here=1
        if [ "$kind" = D ]; then
            if [ $unchanged_here = 1 ]; then rm -f "colcon_ws/src/$p"; echo "  removed  $p"
            else echo "  kept     $p (removed on the robot, changed here)"; conflicts=1; fi
        elif mkdir -p "colcon_ws/src/$(dirname "$p")" && [ $unchanged_here = 1 ]; then
            cp -p "$tmp/src/$p" "colcon_ws/src/$p"; echo "  pulled   $p"
        else
            cp -p "$tmp/src/$p" "colcon_ws/src/$p.robot"; echo "  conflict $p (changed here too: the robot's is $p.robot)"; conflicts=1
        fi
    done <<<"$drift"
    # the robot's files are now accounted for here: they are the new baseline for the edit check
    ssh_ "cd ~/colcon_ws/src && $MANIFEST > ~/colcon_ws/$SAVED"
    echo "review with git status / git diff (submodules: commit there first), commit, then deploy"
    [ $conflicts = 0 ] || echo "merge each <file>.robot into <file> by hand and delete it"
    exit 0
fi

if [ -n "$drift" ]; then
    echo "edited on the robot since the last deploy (M changed, A added, D removed):"
    sed 's/^/  /' <<<"$drift" | head -50
    if [ $dry = 0 ]; then
        echo "--pull copies them into colcon_ws/src (deploys nothing), --force overwrites them" >&2
        exit 1
    fi
fi

# what gets deployed: the checked-out tree, including each submodule's checked-out commit
if [ -n "$(git status --porcelain --ignore-submodules=none -- colcon_ws/src)" ]; then
    git status --short --ignore-submodules=none -- colcon_ws/src >&2
    if [ $dirty_ok = 0 ]; then
        echo "colcon_ws/src has uncommitted changes (above): commit them (submodules first), or --allow-dirty" >&2
        exit 1
    fi
    echo "warning: deploying uncommitted changes" >&2
fi
echo "multirobot_sim $(git rev-parse --short HEAD)"
ssh_ 'mkdir -p ~/colcon_ws/src'

RSYNC=(rsync -rlptc --delete --exclude .git --exclude __pycache__ -e "ssh -o BatchMode=yes")
changes=$("${RSYNC[@]}" -n --itemize-changes colcon_ws/src/ "$target:colcon_ws/src/")
deletes=$(grep '^\*deleting' <<<"$changes" || true)
echo "$(grep -v '^\*deleting' <<<"$changes" | grep -c . || true) file(s) to update, $(grep -c . <<<"$deletes" || true) to delete"
if [ -n "$deletes" ]; then
    echo "$deletes" | sed 's/^\*deleting */  delete /' | head -50
    [ "$(grep -c . <<<"$deletes")" -le 50 ] || echo "  ..."
fi
[ $dry = 0 ] || { grep -v '^\*deleting' <<<"$changes" | sed 's/^[^ ]* /  update /' | head -50; exit 0; }
if [ -n "$deletes" ] && [ $yes = 0 ]; then
    read -r -p "delete these from $host:~/colcon_ws/src? [y/N] " a < /dev/tty
    [[ "$a" =~ ^[yY] ]] || { echo "aborted, nothing changed"; exit 1; }
fi
"${RSYNC[@]}" colcon_ws/src/ "$target:colcon_ws/src/"
{
    echo "multirobot_sim $(git rev-parse HEAD) ($(git log -1 --format='%cs %s'))"
    [ -z "$(git status --porcelain --ignore-submodules=none -- colcon_ws/src)" ] || echo "with uncommitted changes"
    echo "deployed $(date -Is) by $(whoami)@$(hostname)"
    git submodule status -- colcon_ws/src
} | ssh_ 'cat > ~/colcon_ws/DEPLOYED'
ssh_ "cd ~/colcon_ws/src && $MANIFEST > ~/colcon_ws/$SAVED"
echo "synced"
[ $build = 1 ] || exit 0

# build on the robot, on top of the workspaces robot.yaml lists before ~/colcon_ws
# (one { } block: bash reads all of it before running anything, so nothing in it can eat the rest from stdin)
ssh_ bash -s -- "$@" <<'EOF'
{
source /opt/ros/jazzy/setup.bash
set -eo pipefail
mapfile -t workspaces < <(python3 -c '
import yaml
ws = (yaml.safe_load(open("/etc/clearpath/robot.yaml")).get("system", {}).get("ros2", {}) or {}).get("workspaces") or []
print("\n".join(ws))' 2>/dev/null || true)
own=$HOME/colcon_ws/install/setup.bash
listed=0
for w in "${workspaces[@]}"; do
    [ "$w" = "$own" ] && { listed=1; break; }
    if [ -f "$w" ]; then echo "underlay: $w"; source "$w"; else echo "warning: $w (robot.yaml) does not exist" >&2; fi
done
[ $listed = 1 ] || echo "warning: /etc/clearpath/robot.yaml system.ros2.workspaces doesn't list $own: the robot won't use this build" >&2
cd ~/colcon_ws
# packages an underlay workspace (not /opt/ros) already has: colcon_ws's copy wins at run time
override=()
for p in $(colcon list -n --base-paths src); do
    IFS=: read -ra prefixes <<<"${AMENT_PREFIX_PATH:-}"
    for pre in "${prefixes[@]}"; do
        [[ "$pre" == /opt/ros/* ]] && continue
        [ -f "$pre/share/$p/package.xml" ] && { override+=("$p"); echo "overrides $p in $pre"; break; }
    done
done
rosdep check --from-paths src --ignore-src 2>/dev/null </dev/null | grep -v '^All system' || true
[ ${#override[@]} = 0 ] || set -- --allow-overriding "${override[@]}" "$@"
colcon build --symlink-install "$@" --cmake-args -DCMAKE_BUILD_TYPE=Release </dev/null
echo "built; restart clearpath-robot (sudo systemctl restart clearpath-robot) or your launch to use it"
}
EOF
