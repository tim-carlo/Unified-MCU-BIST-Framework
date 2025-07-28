#!/bin/bash

TOOLS="./tools"
TARGETS="./targets"

clean_target() {
    t="$1"
    case $t in
        msp430|nrf52)
            make -C "$TARGETS/$t" clean
            ;;
        *)
            echo "Unknown target: $t"
            exit 1
            ;;
    esac
}

build_target() {
    t="$1"
    case $t in
        msp430)
            make -C "$TARGETS/$t" docker-build
            ;;
        nrf52)
            make -C "$TARGETS/$t"
            ;;
        *)
            echo "Unknown target: $t"
            exit 1
            ;;
    esac
}

flash_target() {
    t="$1"
    case $t in
        msp430)
            b="$TARGETS/msp430/build/build.hex"
            [ -f "$b" ] || {
                echo "Binary not found! Please run build first."
                exit 1
            }
            "$TOOLS/flash_launchpad.sh" "$b"
            ;;
        nrf52)
            make -C "$TARGETS/nrf52" flash
            ;;
        *)
            echo "Unknown target: $t"
            exit 1
            ;;
    esac
}

build_flash_target() {
    t="$1"
    case $t in
        msp430)
            build_target "$t"
            flash_target "$t"
            ;;
        nrf52)
            make -C "$TARGETS/nrf52" flash
            ;;
    esac
}

case "$1" in
    clean)
        [ -z "$2" ] && {
            echo "Target required: msp430 or nrf52"
            exit 1
        }
        clean_target "$2"
        ;;
    build)
        [ -z "$2" ] && {
            echo "Target required: msp430 or nrf52"
            exit 1
        }
        build_target "$2"
        ;;
    flash)
        [ -z "$2" ] && {
            echo "Target required: msp430 or nrf52"
            exit 1
        }
        flash_target "$2"
        ;;
    all)
        [ -z "$2" ] && {
            echo "Target required: msp430 or nrf52"
            exit 1
        }
        build_flash_target "$2"
        ;;
    *)
        echo "Usage: $0 {clean|build|flash|all} [target]"
        echo "Targets: msp430, nrf52"
        echo "Examples:"
        echo "  $0 build msp430"
        echo "  $0 flash nrf52"
        echo "  $0 all msp430"
        exit 1
        ;;
esac