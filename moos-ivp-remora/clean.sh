#!/bin/bash
# Remove build output and mission leftovers in the moos-ivp-remora tree.
cd "$(dirname "$0")"

find build lib bin -mindepth 1 ! -name .gitignore -exec rm -rf {} + 2>/dev/null
rm -f .DS_Store
for m in missions/*/; do
    [ -x "$m/clean.sh" ] && "$m/clean.sh"
done

find . -name '.DS_Store'  -print -exec rm -rf {} \;
find . -name '*~'         -print -exec rm -rf {} \;
find . -name '#*'         -print -exec rm -rf {} \;
find . -name '*.moos++'   -print -exec rm -rf {} \;
find . -name 'MOOSLog*'   -print -exec rm -rf {} \;
