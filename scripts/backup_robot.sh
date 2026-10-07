#!/bin/bash
# Back up a real robot's system configuration and its own code (everything scripts/deploy_robot.sh doesn't put
# there) to robot_data/<id>/backups/<date>_<time>/ on this workstation. Changes nothing on the robot.
#
#   scripts/backup_robot.sh a300_00036            # asks for the robot's sudo password once (root-only files in /etc)
#   scripts/backup_robot.sh a200_0284 --no-sudo   # no password: root-only files (netplan's, ...) are skipped, listed
#
# Options: --host H (default: tools/sim_ui/real_robots.json's host, else cpr-<id with - for _>.local),
# --user U (default robot), --no-sudo.
#
# A snapshot:
#   root/        /etc (without shadow/gshadow), systemd units and udev rules no package installed, ZED
#                calibration, crontabs; root.tar.gz is the same with owners and modes, for restoring
#   home/        dotfiles, ~/.ssh authorized_keys/known_hosts/config, ~/*_config/ (zenoh, cyclonedds),
#                diagnostic_captures/, every other file in ~ up to 20 MB (robot.yaml.*.bak, rtk.txt, ...)
#   ws/<ws>/     every ~/<ws> with a src/ except ~/colcon_ws (this repo's), .git included; no build/install/log,
#                .venv (pip freeze instead)
#   system/      packages, pip freezes, enabled services, network, git_state.tsv, git/*.diff (uncommitted
#                edits), not_in_git.txt, unowned_files.txt, modified_conffiles.txt
#   SUMMARY.md   code that exists only on the robot, what was skipped, what changed since the last snapshot
#   RESTORE.md   where things go back
# Unchanged files are hard links into the previous snapshot (rsync --link-dest): a repeat run transfers and
# stores only what changed. Snapshots hold secrets (WiFi passwords in netplan, NTRIP): mode 700, gitignored.
# Not backed up: rosbags, ~/.cache, editor servers, SDK installers, ~/colcon_ws (deploy_robot.sh's).
set -euo pipefail
umask 077
cd "$(dirname "$0")/.."
repo=$PWD

usage() { sed -n '2,24p' "$0" | sed 's/^# \{0,1\}//' >&2; exit 1; }

id="" host="" user=robot sudo=1
while [ $# -gt 0 ]; do
    case "$1" in
        --host) host=$2; shift 2 ;;
        --user) user=$2; shift 2 ;;
        --no-sudo) sudo=0; shift ;;
        -h|--help) usage ;;
        -*) echo "unknown option $1" >&2; usage ;;
        *) [ -z "$id" ] || usage; id=$1; shift ;;
    esac
done
[[ "$id" =~ ^[a-z0-9]+_[0-9]+$ ]] || usage
host=${host:-$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1])).get(sys.argv[2], {}).get("host", ""))' \
    tools/sim_ui/real_robots.json "$id" 2>/dev/null || true)}
host=${host:-cpr-${id//_/-}.local}
target="$user@$host"
ssh_() { ssh -o BatchMode=yes -o ConnectTimeout=10 "$target" "$@"; }
RSYNC=(rsync -a -e "ssh -o BatchMode=yes -o ConnectTimeout=10")

ssh_ true || { echo "cannot ssh to $target (key login: ssh-copy-id $target)" >&2; exit 1; }
if [ $sudo = 1 ] && [ ! -t 0 ]; then
    echo "no terminal to type the sudo password in: run from a terminal, or --no-sudo" >&2; exit 1
fi

base=$repo/robot_data/$id/backups
mkdir -p "$base"; chmod 700 "$base"
prev=""; [ -L "$base/latest" ] && [ -d "$base/latest/" ] && prev=$(readlink -f "$base/latest")
ts=$(date +%Y-%m-%d_%H%M%S)
snap=$base/$ts.partial
mkdir -p "$snap"/{system,home,ws}
link() { [ -n "$prev" ] && [ -d "$prev/$1" ] && echo "--link-dest=$prev/$1"; true; }
echo "== $id: $target -> ${snap#$repo/}${prev:+ (unchanged files linked to ${prev##*/})}"

# --- root pass: /etc and other system files, as root (sudo) or, with --no-sudo, whatever the user can read
read -r -d '' ROOT_SCRIPT <<'EOF' || true
set -u
u=$1 out=$2
cd "$out" && mkdir -p report
# files no package installed (dpkg records /lib/... for files now under /usr/lib)
cat /var/lib/dpkg/info/*.list | sed -e p -e 's#^/lib/#/usr/lib/#' | LC_ALL=C sort -u > owned
find /etc /usr/lib/systemd/system /usr/lib/udev/rules.d /usr/local/bin /usr/local/sbin -xdev \( -type f -o -type l \) \
    2>/dev/null | LC_ALL=C sort > all
LC_ALL=C comm -23 all owned > report/unowned_files.txt
# conffiles edited since their package installed them (or unreadable without sudo)
dpkg-query -W -f='${Conffiles}\n' | awk 'NF >= 2 && $3 != "obsolete" { print $2 "  " $1 }' \
    | md5sum -c 2>/dev/null | grep -v ': OK$' > report/modified_conffiles.txt
find /etc -xdev ! -readable 2>/dev/null | grep -vE '^/etc/g?shadow-?$' > report/unreadable.txt
extra=$( { grep -E '^/usr/lib/(systemd/system|udev/rules\.d)/' report/unowned_files.txt
           ls -d /usr/local/zed/settings /var/spool/cron/crontabs 2>/dev/null; } | sed 's#^/##')
tar -czf root.tar.gz --ignore-failed-read --exclude='etc/shadow*' --exclude='etc/gshadow*' \
    -C / etc $extra -C "$out" report 2> tar_warnings.txt
rm -rf owned all report
chown "$u:" root.tar.gz tar_warnings.txt
EOF
rdir=$(ssh_ mktemp -d /tmp/robot_backup.XXXXXX)
ssh_ "cat > $rdir/root.sh" <<<"$ROOT_SCRIPT"
if [ $sudo = 1 ]; then
    echo "-- root-only files: sudo on $host"
    ssh -t -o BatchMode=yes -o ConnectTimeout=10 "$target" \
        "sudo -p '[sudo] password for %u on %H: ' bash $rdir/root.sh $user $rdir" \
        || { ssh_ "rm -rf $rdir"; echo "sudo failed (nothing kept): rerun, or --no-sudo" >&2; rm -rf "$snap"; exit 1; }
else
    echo "-- /etc without sudo: root-only files are skipped"
    ssh_ "bash $rdir/root.sh $user $rdir"
fi
"${RSYNC[@]}" "$target:$rdir/root.tar.gz" "$snap/"
"${RSYNC[@]}" "$target:$rdir/tar_warnings.txt" "$snap/system/"
mkdir "$snap/root"
tar -xzf "$snap/root.tar.gz" -C "$snap/root" --no-same-owner 2>/dev/null || true
chmod -R u+rwX "$snap/root"
mv "$snap/root/report/"* "$snap/system/" && rmdir "$snap/root/report"

# --- reports, as the user: versions, packages, services, network, git state of the robot's own code
read -r -d '' REPORT_SCRIPT <<'EOF' || true
set -u
exec 3>&1 1>&2
d=$(mktemp -d); trap 'rm -rf "$d"' EXIT; cd "$d"; mkdir git
{
    echo "hostname: $(hostname)"
    echo "os: $(lsb_release -ds 2>/dev/null)"
    echo "kernel: $(uname -r)"
    echo "gpu: $(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null)"
    echo "cuda: $(ls -d /usr/local/cuda-* 2>/dev/null | xargs -rn1 basename | tr '\n' ' ')"
    echo "zed sdk: $(sed -n 's/.*PACKAGE_VERSION "\(.*\)".*/\1/p' /usr/local/zed/zed-config-version.cmake 2>/dev/null)"
    echo "zed calibration: $(ls /usr/local/zed/settings 2>/dev/null | tr '\n' ' ')"
    echo "uptime: $(uptime -p)"
} > os.txt
dpkg-query -W -f='${Package}\t${Version}\n' > packages.tsv
apt-mark showmanual > apt_manual.txt 2>/dev/null
python3 -m pip list --user --format=freeze > pip_user.txt 2>/dev/null
systemctl list-unit-files --state=enabled --no-legend --no-pager > services_enabled.txt
systemctl list-units --type=service --state=running --no-legend --no-pager > services_running.txt
{ ip -br addr; echo; ip route; echo; nmcli -t -f NAME,TYPE,DEVICE,AUTOCONNECT connection show 2>/dev/null; } > network.txt
crontab -l > crontab.txt 2>&1
cat ~/colcon_ws/DEPLOYED > colcon_ws_DEPLOYED.txt 2>/dev/null
# workspaces: every ~/<dir> with a src/, except ~/colcon_ws (scripts/deploy_robot.sh's copy of multirobot_sim)
for w in ~/*/; do w=${w%/}; [ -d "$w/src" ] && [ "$w" != "$HOME/colcon_ws" ] && echo "${w#$HOME/}"; done > workspaces.txt
srcfind() {  # find in every workspace's src, skipping build output and (unless VENVS=1) venvs
    local v=-name\ .venv; [ "${VENVS:-0}" = 0 ] || v=-false
    while read -r w; do
        find "$HOME/$w/src" -maxdepth 4 \( -name build -o -name install -o -name log -o $v \) -prune -o "$@"
    done < workspaces.txt 2>/dev/null
}
# git repos: ~/<dir> itself, or in a workspace's src
repos=$( { for w in ~/*/; do [ -e "$w/.git" ] && echo "${w%/}"; done; srcfind -name .git -printf '%h\n'; } | LC_ALL=C sort -u)
printf 'path\tremote\tbranch\tcommit\tupstream\tahead\tbehind\tchanged\n' > git_state.tsv
while read -r r; do
    [ -n "$r" ] || continue
    rel=${r#$HOME/}
    up=$(git -C "$r" rev-parse --abbrev-ref --symbolic-full-name '@{u}' 2>/dev/null) || up=-
    ahead=- behind=-
    [ "$up" = - ] || read -r behind ahead < <(git -C "$r" rev-list --left-right --count "$up...HEAD" 2>/dev/null)
    n=$(git -C "$r" status --porcelain 2>/dev/null | grep -cvE '^\?\? (.*/)?COLCON_IGNORE$')
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$rel" "$(git -C "$r" remote get-url origin 2>/dev/null || echo -)" \
        "$(git -C "$r" rev-parse --abbrev-ref HEAD 2>/dev/null)" "$(git -C "$r" rev-parse --short HEAD 2>/dev/null)" \
        "$up" "$ahead" "$behind" "$n" >> git_state.tsv
    if [ "$n" != 0 ] || { [ "$ahead" != - ] && [ "$ahead" != 0 ]; }; then
        {
            [ "$ahead" = - ] || [ "$ahead" = 0 ] || { echo "# commits not on $up"; git -C "$r" log --oneline "$up..HEAD"; echo; }
            echo "# git status"; git -C "$r" status --short; echo
            echo "# git diff HEAD (untracked files: in ws/)"; git -C "$r" diff HEAD
        } > "git/${rel//\//__}.diff" 2>&1
    fi
done <<<"$repos"
# packages whose package.xml no git repo tracks
srcfind -name package.xml -printf '%h\n' | LC_ALL=C sort -u | while read -r p; do
    git -C "$p" ls-files --error-unmatch package.xml >/dev/null 2>&1 || echo "${p#$HOME/}"
done > not_in_git.txt
srcfind -name COLCON_IGNORE -printf '%h\n' | sed "s#^$HOME/##" | LC_ALL=C sort > colcon_ignore.txt
VENVS=1 srcfind -name pyvenv.cfg -printf '%h\n' | while read -r v; do
    rel=${v#$HOME/}; "$v/bin/python" -m pip freeze > "venv__${rel//\//__}.txt" 2>/dev/null
done
tar -cz -C "$d" . >&3
EOF
echo "-- reports"
ssh_ "cat > $rdir/report.sh" <<<"$REPORT_SCRIPT"
ssh_ "bash $rdir/report.sh </dev/null" | tar -xz -C "$snap/system"
ssh_ "rm -rf $rdir"

# --- home: config and small files, not caches, editor servers, installers or rosbags
echo "-- home"
"${RSYNC[@]}" $(link home) --max-size=20M --log-file="$snap/system/rsync_home.log" --log-file-format='%i %n' \
    --include=/.ssh/ --include=/.ssh/authorized_keys --include=/.ssh/known_hosts --include=/.ssh/config --exclude='/.ssh/*' \
    --include='/*_config/***' --include='/diagnostic_captures/***' \
    --exclude=/.viminfo --exclude=/.lesshst --exclude=/.wget-hsts --exclude=/.Xauthority --exclude=/.claude.json \
    --exclude=/.python_history --exclude=/.sudo_as_admin_successful \
    --exclude='/*/' --include='/*' \
    "$target:" "$snap/home/"

# --- workspaces, with .git; build output and venvs are rebuilt from the sources and the pip freezes
while read -r w; do
    [ -n "$w" ] || continue
    echo "-- ws $w"
    "${RSYNC[@]}" $(link "ws/$w") --info=progress2 --log-file="$snap/system/rsync_ws_$w.log" --log-file-format='%i %n' \
        --exclude=/build/ --exclude=/install/ --exclude=/log/ \
        --exclude=/src/build/ --exclude=/src/install/ --exclude=/src/log/ \
        --exclude=.venv/ --exclude=__pycache__/ --exclude='*.pyc' \
        "$target:$w/" "$snap/ws/$w/"
done < "$snap/system/workspaces.txt"

# --- SUMMARY.md
S=$snap/system
changed() { grep -c ' >f' "$1" 2>/dev/null || true; }
pname=${prev##*/}; pname=${pname:-none}
{
    echo "# $id backup $ts"
    echo
    echo "From $target by $(whoami)@$(hostname), $( [ $sudo = 1 ] && echo "with sudo" || echo "without sudo (--no-sudo)" )."
    echo "Previous snapshot: $pname."
    sed 's/^/    /' "$S/os.txt"
    echo
    echo "## Code that exists only on the robot"
    echo
    if [ -s "$S/not_in_git.txt" ]; then
        echo "Packages no git repo tracks (copied in ws/):"; sed 's/^/- /' "$S/not_in_git.txt"; echo
    fi
    awk -F'\t' 'NR > 1 && $8 > 0 { f = $1; gsub("/", "__", f); printf "- %s: %d changed/untracked file(s) (system/git/%s.diff)\n", $1, $8, f }' \
        "$S/git_state.tsv" | { read -r l && { echo "Uncommitted edits:"; echo "$l"; cat; echo; }; } || true
    awk -F'\t' 'NR > 1 && $6 != "-" && $6 > 0 { printf "- %s: %d commit(s) not on %s\n", $1, $6, $5 }' \
        "$S/git_state.tsv" | { read -r l && { echo "Unpushed commits:"; echo "$l"; cat; echo; }; } || true
    awk -F'\t' 'NR > 1 && $5 == "-" { printf "- %s (%s, %s)\n", $1, $3, $2 }' \
        "$S/git_state.tsv" | { read -r l && { echo "No upstream branch (can't tell what is pushed):"; echo "$l"; cat; echo; }; } || true
    echo "## Skipped"
    echo
    n=$(grep -c . "$S/unreadable.txt" || true)
    if [ "$n" != 0 ]; then
        echo "$n file(s) in /etc only root can read, not backed up (system/unreadable.txt; rerun without --no-sudo), e.g.:"
        { grep -E '^/etc/(netplan|NetworkManager|iptables|network|sudoers|clearpath|wpa_supplicant)' "$S/unreadable.txt"; cat "$S/unreadable.txt"; } \
            | awk '!seen[$0]++' | head -8 | sed 's/^/- /'
        echo
    fi
    grep -v -e 'Permission denied' -e '^$' "$S/tar_warnings.txt" | head -5 | sed 's/^/- /' || true
    echo "- not backed up: ~/rosbags, ~/.cache, ~/.local, editor servers, files over 20 MB in ~, ~/colcon_ws (scripts/deploy_robot.sh)"
    echo
    echo "## Changed since $pname"
    echo
    if [ -n "$prev" ]; then
        for d in root home; do
            diff -rq --no-dereference "$prev/$d" "$snap/$d" 2>&1 | sed -e "s#$prev/$d/##g" -e "s#$snap/$d/##g" -e "s#$prev/##g" -e "s#$snap/##g" \
                | sed "s#^#- $d: #" | head -40 || true
        done
        diff <(tail -n +2 "$prev/system/git_state.tsv" | cut -f1,3,4,6,8) <(tail -n +2 "$S/git_state.tsv" | cut -f1,3,4,6,8) \
            | grep '^[<>]' | sed -e 's/^</- git before:/' -e 's/^>/- git now:   /' | tr '\t' ' ' || true
        diff <(cut -f1,2 "$prev/system/packages.tsv") <(cut -f1,2 "$S/packages.tsv") \
            | grep '^[<>]' | sed -e 's/^</- package was:/' -e 's/^>/- package now:/' | tr '\t' ' ' | head -40 || true
        while read -r w; do echo "- ws $w: $(changed "$S/rsync_ws_$w.log") file(s) new or changed"; done < "$S/workspaces.txt"
    else
        echo "- everything"
    fi
} > "$snap/SUMMARY.md"

cat > "$snap/RESTORE.md" <<EOF
# Restoring $id from this snapshot

Nothing is restored automatically. root.tar.gz keeps owners and modes; root/ is the same unpacked, for reading.

- One file, as root on the robot: \`tar -xzf root.tar.gz -C / etc/netplan/60-wifi.yaml\` (copy root.tar.gz there
  first), then \`sudo netplan apply\` for netplan, \`sudo udevadm control --reload && sudo udevadm trigger\` for
  udev rules, \`sudo sysctl --system\` for sysctl.d, \`sudo systemctl daemon-reload\` for units.
- /etc/clearpath/robot.yaml: saving it restarts every Clearpath service about 15 s later; make sure the
  workspaces it lists are built first.
- Units no package installed (system/unowned_files.txt, e.g. rtk-corrections.service): copy from
  root/usr/lib/systemd/system, \`sudo systemctl enable --now <unit>\`; system/services_enabled.txt is what was on.
- Workspaces: rsync ws/<ws>/ to ~/<ws>/ (git repos come with .git and their uncommitted edits), recreate
  each venv from system/venv__*.txt, then colcon build (--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release).
  ~/colcon_ws: scripts/deploy_robot.sh $id.
- Home: rsync home/ to ~/ (zenoh/cyclonedds configs, robot.yaml backups, dotfiles, authorized_keys).
- Packages: system/apt_manual.txt (installed by hand), system/packages.tsv (all, with versions),
  system/pip_user.txt; GPU/CUDA/ZED versions in system/os.txt.
- ZED calibration: root/usr/local/zed/settings/SN*.conf to /usr/local/zed/settings/.
EOF

mv "$snap" "$base/$ts"
ln -sfn "$ts" "$base/latest"
echo "== done: ${base#$repo/}/$ts ($(du -sh "$base/$ts" | cut -f1)${prev:+, new: $(du -sh "$prev" "$base/$ts" | tail -1 | cut -f1)})"
sed -n '/^## Code that exists only/,$p' "$base/$ts/SUMMARY.md"
