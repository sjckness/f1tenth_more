# shellcheck shell=bash
# One warning line in every interactive Orin shell while the car's Jazzy
# container runs (plan P6.5): native ROS (Humble) started from a shell then
# would be a second stack on the same devices, ports and domain.
#
# Sourced from ~/.bashrc; docker/orin/host/install.sh adds the line
#   [ -f ~/.local/share/f1tenth_car/bashrc_banner.sh ] && . ~/.local/share/f1tenth_car/bashrc_banner.sh  # f1tenth_car banner
# and uninstall.sh removes it. Prints nothing otherwise, never fails, never
# exits the shell, leaves no variable behind, and waits at most 2 s for Docker.
case $- in
  *i*)
    if command -v docker > /dev/null 2>&1 \
       && [ "$(timeout 2 docker inspect --type container -f '{{.State.Running}}' f1tenth_car 2> /dev/null)" = true ]; then
      printf '\033[1;33m%s\033[0m\n' \
        "f1tenth: the Jazzy container f1tenth_car is RUNNING. Do not start native ROS (Humble) here. Mode: stack_mode status; back to native: stack_mode native; a shell in it: docker_orin shell" >&2
    fi
    ;;
esac
true
