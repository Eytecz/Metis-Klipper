#!/bin/bash
set -eu
export LC_ALL=C

KLIPPER_PATH="${HOME}/klipper"
INSTALL_PATH="${1:-${HOME}/metis-klipper}"
REPO_URL="https://github.com/Eytecz/Metis-Klipper.git"
REPO_NAME="Metis-Klipper"

MOONRAKER_CONF="${HOME}/printer_data/config/moonraker.conf"

function preflight_checks {
    if [ "$EUID" -eq 0 ]; then
        echo "[PRE-CHECK] This script must not be run as root!"
        exit 1
    fi

    if systemctl list-units --full -all -t service --no-legend | grep -qF 'klipper.service'; then
        printf "[PRE-CHECK] Klipper service found! Continuing...\n\n"
    else
        echo "[ERROR] Klipper service not found, please install Klipper first!"
        exit 1
    fi
}

function check_download {
    local installdirname installbasename
    installdirname="$(dirname "${INSTALL_PATH}")"
    installbasename="$(basename "${INSTALL_PATH}")"

    mkdir -p "$installdirname"

    if [ ! -d "${INSTALL_PATH}" ]; then
        echo "[DOWNLOAD] Downloading repository..."
        if git -C "$installdirname" clone "$REPO_URL" "$installbasename"; then
            chmod +x "${INSTALL_PATH}/install.sh"
            printf "[DOWNLOAD] Download complete!\n\n"
        else
            echo "[ERROR] Download of git repository failed!"
            exit 1
        fi
    else
        printf "[DOWNLOAD] Repository already found locally. Continuing...\n\n"
    fi
}

function link_extension {
    echo "[INSTALL] Linking extension to Klipper..."
    for file in "${INSTALL_PATH}"/klippy/extras/*.py; do
        ln -sfn "${file}" "${KLIPPER_PATH}/klippy/extras/"
    done
}

function register_update_manager {
    echo "[MOONRAKER] Adding update manager entry if not present..."
    if [ ! -f "$MOONRAKER_CONF" ]; then
        echo "[WARNING] Moonraker config not found at $MOONRAKER_CONF — skipping."
        return
    fi

    if grep -q "^\[update_manager ${REPO_NAME}\]" "$MOONRAKER_CONF"; then
        echo "[MOONRAKER] Entry already exists — skipping."
    else
        cat <<EOL >> "$MOONRAKER_CONF"

[update_manager ${REPO_NAME}]
type: git_repo
channel: dev
path: ${INSTALL_PATH}
origin: ${REPO_URL}
managed_services: klipper
primary_branch: main
install_script: install.sh
EOL
        echo "[MOONRAKER] Entry added to $MOONRAKER_CONF"
    fi
}

function restart_klipper {
    echo "[POST-INSTALL] Restarting Klipper..."
    sudo systemctl restart klipper
}

printf "\n======================================\n"
echo "- Klipper ${REPO_NAME} install script -"
printf "======================================\n\n"

preflight_checks
check_download
link_extension
register_update_manager
restart_klipper
