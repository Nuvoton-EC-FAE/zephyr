# Copyright (c) 2021 Google Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_BBRAM_NPCX bbram_npcx.c)
zephyr_library_sources_ifdef(CONFIG_BBRAM_NPCX_EMUL bbram_npcx_emul.c)
zephyr_library_include_directories_ifdef(CONFIG_BBRAM_NPCX .)
zephyr_library_sources_ifdef(CONFIG_BBRAM_IT8XXX2 bbram_it8xxx2.c)
zephyr_library_sources_ifdef(CONFIG_BBRAM_EMUL bbram_emul.c)
zephyr_library_sources_ifdef(CONFIG_BBRAM_XEC bbram_xec.c)
