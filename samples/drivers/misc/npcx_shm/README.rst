.. zephyr:code-sample:: npcx_shm
   :name: NPCX SHM
   :relevant-api: npcx_shm_interface

   Demonstrates the Nuvoton NPCX shared memory driver APIs.

Overview
********

This sample exercises the NPCX SHM driver APIs for:

- SHM initialization and core configuration
- Semaphore interrupt and host semaphore configuration
- Window base/size and protection configuration
- Core/host offset configuration and related interrupts
- Semaphore read/write

.. note::

   This sample requires eSPI to be enabled and configured. The NPCX SHM
   peripheral is powered through the eSPI module; without an active eSPI
   link the host cannot access the shared memory windows. The sample
   initializes eSPI at startup (``CONFIG_ESPI=y`` in ``prj.conf``) before
   configuring any SHM window.

Devicetree model:

- SHM controller node ``shm`` owns the shared MMIO regions.
- Four child window nodes (``shm_win1`` to ``shm_win4``) represent
   window-specific driver instances.
- Boards enable one or more window nodes in overlays via ``status = "okay"``.

Requirements
************

- A Nuvoton NPCX4 evaluation board (e.g. ``npcx4m8f_evb``).
- A host connected via eSPI. Without an eSPI host the shared memory windows
  are accessible from the EC core side only; host-triggered interrupts and
  SMI/IRQ signalling will not fire.

Building and Running
********************

Build and flash the sample on NPCX4M8F EVB:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/misc/npcx_shm
   :board: npcx4m8f_evb
   :goals: flash
   :compact:
