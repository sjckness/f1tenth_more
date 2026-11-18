# tools/SUDO_TODO.md: cio' che il tooling non puo' fare da solo (sudo o permessi da admin)

Niente qui e' stato eseguito. Ogni voce dice dove serve, il comando e cosa sblocca.

## Orin (`ssh fabiocar@jetson-orin`)

1. **`/etc/f1tenth/` scrivibile da fabiocar** (deployed.json nel percorso chiesto):
   `sudo install -d -o fabiocar -g fabiocar -m 755 /etc/f1tenth`.
   Finche' manca, `f1t deploy` scrive `~/.config/f1tenth/deployed.json` e lo segnala con un WARN.
2. **Buffer UDP per l'HIL** (precheck: `rmem_max orin` WARN, 212992 oggi):
   `echo 'net.core.rmem_max=16777216' | sudo tee /etc/sysctl.d/90-f1tenth-hil.conf && sudo sysctl --system`
   (anche `net.core.wmem_max`). Il setup B lo chiedeva gia' (output/hil/setup.md su hil-isaac).
3. **Collegamento HIL 10.42.60.0/24 permanente**: `eno1` ha 10.42.60.2 solo fino al riavvio
   (handoff 2026-11-11). `sudo nmcli con add type ethernet ifname eno1 ip4 10.42.60.2/24` senza
   toccare il profilo del lidar (`hokuyo-static`, 192.168.0.15): da fare con l'utente davanti.

## linus (o la macchina work)

4. **Indirizzo HIL sul cavo verso l'Orin** (precheck: `interfaccia, IP, MTU` FAIL; 10.42.60.1 non
   risponde ad ARP dal 2026-11-11): `sudo nmcli con add type ethernet ifname <iface> ip4 10.42.60.1/24`
   sull'interfaccia del cavo (n/d: `ip -o link`), poi `HOST_LINK_IFACE` resta `auto`.
5. **Registry Docker locale** (facoltativo: oggi l'immagine nasce sull'Orin e non viaggia):
   `docker run -d --restart always -p 5000:5000 --name registry registry:2` su linus e, sull'Orin,
   `"insecure-registries": ["linus:5000"]` in `/etc/docker/daemon.json` + restart di dockerd. Attenzione:
   il daemon.json dell'Orin ha `bridge: none` / `iptables: false` (CLAUDE.md "Orin Docker"): una
   modifica sbagliata lascia dockerd giu' al boot. Serve solo se un giorno la build passa su linus.
6. **`asio_cmake_module` per costruire vesc_driver su linus**: `sudo apt install ros-jazzy-asio-cmake-module`.
   Senza, `vesc_driver` non si configura e restano fuori anche vesc, f1tenth_hardware, f1tenth_control,
   mpc_controller, f1tenth_planners, f1tenth_bringup, f1tenth_more, vesc_tuning (2026-11-18). Poi:
   `colcon build --symlink-install --packages-select vesc_driver vesc f1tenth_hardware f1tenth_control mpc_controller f1tenth_planners f1tenth_bringup f1tenth_more vesc_tuning`.
7. **Spazio disco su linus** (risolto in parte 2026-11-18: 35 GB liberi dopo la pulizia dei worktree): il precheck vuole 25 GB liberi su / (oggi 12). Non serve sudo ma va
   deciso cosa cancellare (immagini Docker, bag).

## GitHub (admin del repo sjckness/f1tenth_more)

8. **Branch protection di main** (merge bloccato se la CI e' rossa): Settings -> Branches -> Add rule
   `main`: "Require status checks to pass" con `tools-core / core` e `ros-packages / build-test`,
   "Require branches to be up to date". Oppure:
   `gh api -X PUT repos/sjckness/f1tenth_more/branches/main/protection --input protection.json`.
   Prima di attivarla: far girare `ros-packages` una volta e smistare i test gia' rossi su main
   (CLAUDE.md elenca quelli noti), altrimenti blocca tutto.

## Non sudo, ma da fare una volta

- `tools/f1t setup` su ogni macchina (hook pre-commit, `~/.local/bin/f1t`, completamento).
- Isaac per il setup B: `docker tag isaac_ws_devcontainer-isaac-sim:latest f1tenth_isaac:5.1.0`
  (docs/analysis/simulation/2026-11-10_hil-isaac-orin/report.md) o build di sim/isaac/docker su hil-isaac;
  mondo office: `scripts/sim/sim_isaac_hil world --convert` nel worktree hil-isaac.
- VRAM: durante Isaac `systemctl --user stop freeparty-llm.service` (il precheck lo indica).
