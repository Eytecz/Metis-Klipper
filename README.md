# Archetype-Klipper
**Archetype-Klipper - The Klipper engine of the Archetype project**

Archetype-Klipper provides the firmware-level integration for the [Archetype](https://github.com/Eytecz/Archetype) project — a unified framework for modular 3D printer toolchanging and adaptive multi-material systems.  
This package extends Klipper with macros, synchronization logic, and configuration templates to support complex toolchanger setups, including multi-extruder and automated filament handling systems.

Designed to be used alongside:  
- [klipper-toolchanger](https://github.com/viesturz/klipper-toolchanger)  
- [AFC-Klipper-Add-On](https://github.com/ArmoredTurtle/AFC-Klipper-Add-On)

---

## Installation

To install Archetype-Klipper, run the installation script over SSH.  
This script will clone the repository to your Raspberry Pi home directory and symlink the necessary files into Klipper’s `klippy/extras` folder.

```bash
wget -O - https://raw.githubusercontent.com/Eytecz/Archetype-Klipper/main/install.sh | bash
```

Add the following section to your moonraker.conf to enable update management for Archetype-Klipper:

```bash
[update_manager Archetype-Klipper]
type: git_repo
channel: dev
path: /home/pi/archetype-klipper
origin: https://github.com/Eytecz/Archetype-Klipper.git
managed_services: klipper
primary_branch: main
install_script: install.sh
```

## Notes
If an update includes new Klipper extension files, you’ll need to manually reinstall them by running:

```bash
bash ~/archetype-klipper/install.sh
```