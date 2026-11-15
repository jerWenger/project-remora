# ansible — MOOS-IvP on the boat computer

One playbook installs the autonomy stack on the Jetson (or any Ubuntu host):

| Role | Installs | Where |
|---|---|---|
| `moos_ivp` | apt build deps, clone + build [moos-ivp](https://github.com/moos-ivp/moos-ivp) (`build-ivp.sh --nogui`), PATH block | `/home/boat/moos-ivp` |
| `mavsdk` | MAVSDK C++ library v3.17.2 for `iArduRoverBridge`. Release `.deb` on x86_64 Ubuntu; **built from source on the Jetson** (no arm64 Ubuntu package upstream) | `/usr/local` |
| `remora_moos` | clone this repo if missing, `moos-ivp-remora/build.sh`, PATH + `IVP_BEHAVIOR_DIRS` block | `/home/boat/project-remora/moos-ivp-remora` |

It does **not** touch the systemd services, udev rules or mavlink-router config. Those stay
with [`deploy/install.sh`](../deploy/install.sh), and `deploy/update.sh` still owns `git pull`
on the checkout: the playbook never pulls an existing repo.

## Run it

From the laptop, on the Tailscale network:

```sh
brew install ansible                      # once
cd ansible
ansible all -m ping                       # reachability
ansible-playbook site.yml -K              # -K asks for boat's sudo password
```

The first run builds MOOS-IvP (~30 min) and MAVSDK (~45 min) on the Orin Nano. The long
steps run with `async`, so a dropped Tailscale/DERP session does not kill the compile; re-run
the playbook and it picks up where the artefacts are. Re-runs with nothing to do take about a
minute.

Useful variants:

```sh
ansible-playbook site.yml -K --tags remora              # just rebuild moos-ivp-remora
ansible-playbook site.yml -K -e moos_ivp_update=true    # pull + rebuild moos-ivp
ansible-playbook site.yml -K -e mavsdk_force_rebuild=true
ansible-playbook site.yml -K -l my-linux-laptop         # another host from inventory.yml
ansible-playbook site.yml --check --diff -K             # dry run (builds are skipped)
```

Afterwards, on the Jetson, open a new shell and check:

```sh
which MOOSDB pHelmIvP iArduRoverBridge
echo $IVP_BEHAVIOR_DIRS
```

## Settings

[`group_vars/all.yml`](group_vars/all.yml) holds every knob with a comment: versions
(`moos_ivp_version`, `mavsdk_version`), directories, build flags (`moos_ivp_build_flags`,
default `--nogui`), parallel jobs. Per-host overrides go in [`inventory.yml`](inventory.yml);
one-offs on the command line with `-e`.

Pin `moos_ivp_version` to a commit sha once the boat is tuned, so a later re-run cannot change
the helm underneath you.

## Files

```
ansible.cfg            ssh keepalives for the relayed Tailscale link, yaml output
inventory.yml          highfieldboat (boat group); empty workstations group as an example
site.yml               the playbook: pre-checks, three roles, next-steps message
group_vars/all.yml     settings
roles/*/tasks/main.yml one task file per role
```

## Why Ansible and not install.sh

`install.sh` copies a handful of files and is run on the Jetson itself. The MOOS stack is a
long compile with system packages and a library install, is driven from the laptop, and must
be safe to re-run. That is what Ansible is for. Both stay small and do one job each.
