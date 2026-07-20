# Copyright (c) 2026 Nuvoton Technology Corporation
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_NPCX_SHM npcx_shm.c)
