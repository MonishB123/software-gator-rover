#!/bin/sh
    
devcontainer up --workspace-folder . \
    --config .devcontainer/linux/devcontainer.json \
    --mount 'type=bind,source=/home/alexrivera/.config/nvim,target=/root/.config/nvim' \
    --additional-features '{"ghcr.io/duduribeiro/devcontainer-features/neovim:1": { "version": "stable" }, "ghcr.io/devcontainers/features/sshd:1": {} }'


