.. zephyr:code-sample:: pwm-npcx-heartbeat
   :name: NPCX PWM Heartbeat
   :relevant-api: pwm_interface

   Configure the Nuvoton NPCX PWM heartbeat helper driver with two profiles.

Overview
********

This sample demonstrates how to use the NPCX heartbeat-specific PWM API in
:zephyr_file:`include/zephyr/drivers/pwm/pwm_hb_npcx.h`.

The application:

1. Gets a devicetree heartbeat device node with compatible
   ``nuvoton,npcx-pwm-heartbeat``.
2. Applies a standard heartbeat profile.
3. Prints the achieved heartbeat frequency returned by
   ``pwm_npcx_hb_configure()``.

Building and Running
********************

Build the sample for ``npcx4m8f_evb``:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/pwm/npcx_heartbeat
   :board: npcx4m8f_evb
   :goals: build flash
   :compact:

Sample output
*************

.. code-block:: console

   NPCX PWM heartbeat sample start
   STANDARD heartbeat profile applied, frequency: <hz> Hz
   VISUAL heartbeat profile applied, frequency: <hz> Hz
