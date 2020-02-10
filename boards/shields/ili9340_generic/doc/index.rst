.. _ili9340_generic:

Generic ILI9340 TFT Shield
#################################

Overview
********

This is a generic shield for display shields based on ILI9340 display
controller. More information about the controller can be found in
`ILI9340 Datasheet`_.

Pins Assignment of the Generic TFT Shield
========================================================

+-----------------------+---------------------------------------------+
| Shield Connector Pin  | Function                                    |
+=======================+=============================================+
| D4                    | MicroSD SPI CSn                             |
+-----------------------+---------------------------------------------+
| D8                    | STMPE610 SPI CSn (Resistive Touch Version)  |
+-----------------------+---------------------------------------------+
| D9                    | ILI9341 DC       (Data/Command)             |
+-----------------------+---------------------------------------------+
| D10                   | ILI9341 SPI CSn                             |
+-----------------------+---------------------------------------------+
| D11                   | SPI MOSI         (Serial Data Input)        |
+-----------------------+---------------------------------------------+
| D12                   | SPI MISO         (Serial Data Out)          |
+-----------------------+---------------------------------------------+
| D13                   | SPI SCK          (Serial Clock Input)       |
+-----------------------+---------------------------------------------+
| SDA                   | FT6206 SDA       (Capacitive Touch Version) |
+-----------------------+---------------------------------------------+
| SCL                   | FT6206 SCL       (Capacitive Touch Version) |
+-----------------------+---------------------------------------------+

Current supported displays
==========================

+----------------------+------------------------------+
| Display              | Shield Designation           |
|                      |                              |
+======================+==============================+
| Adafruit 320x240     | ili9340_adafruit_2_8_tftv2   |
| 2.8" Shield V2.0     |                              |
+----------------------+------------------------------+
| SУEED 320x240        | ili9340_seeed_2_8_tftv2      |
| 2.8" Shield V2.0     |                              |
+----------------------+------------------------------+

Requirements
************

This shield can only be used with a board which provides a configuration
for Arduino connectors and defines node aliases for SPI and GPIO interfaces
(see :ref:`shields` for more details).

Programming
***********

Set ``-DSHIELD=adafruit_2_8_tft_touch_v2`` when you invoke ``west build``. For example:

.. zephyr-app-commands::
   :zephyr-app: samples/gui/lvgl
   :board: nrf52840_pca10056
   :shield: adafruit_2_8_tft_touch_v2
   :goals: build

References
**********

.. target-notes::

.. _Adafruit 2.8" TFT Touch Shield v2 website:
   https://learn.adafruit.com/adafruit-2-8-tft-touch-shield-v2

.. Seed 2.8" TFT Touch Shield v2 website:
   https://github.com/Seeed-Studio/TFT_Touch_Shield_V2
