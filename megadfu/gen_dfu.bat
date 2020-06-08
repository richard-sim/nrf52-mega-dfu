@echo off
REM Enable notifications for characteristic AF7
REM Send 0201 to AF6
REM AF7 will receive 0201XXYY...
REM Device ID is 0xYYXX (convert to dec for nrfutil)
REM device revisions/types:
REM 	603 = ID130C
REM 	652 = ID132Color HR
REM Send 0101 to AF6 to enter DFU mode

REM c:\Python27\Scripts\nrfutil.exe dfu genpkg --dev-revision 603 --dev-type 603 --bootloader C:\dev\proxxiband-fw\proxxiband-fw-nrf52\.pioenvs\proxxihalo\bootloader\bootloader.hex --softdevice C:\dev\proxxiband-fw\.pio-packages\framework-nordic-sdk\sdk\components\softdevice\s132\hex\s132_nrf52_6.1.1_softdevice.hex c:\dev\proxxi-halo\dfu\test-fw-603-ns-blsd.zip

REM c:\Python27\Scripts\nrfutil.exe dfu genpkg --dev-revision 603 --dev-type 603 --application C:\dev\proxxiband-fw\proxxiband-fw-nrf52\.pioenvs\proxxihalo\firmware.hex --bootloader C:\dev\proxxiband-fw\proxxiband-fw-nrf52\.pioenvs\proxxihalo\bootloader\bootloader.hex --softdevice C:\dev\proxxiband-fw\.pio-packages\framework-nordic-sdk\sdk\components\softdevice\s132\hex\s132_nrf52_6.1.1_softdevice.hex c:\dev\proxxi-halo\dfu\test-fw-603-ns-all.zip

REM c:\Python27\Scripts\nrfutil.exe dfu genpkg --dev-revision 603 --dev-type 603 --application C:\dev\proxxiband-fw\proxxiband-fw-nrf52\.pioenvs\proxxihalo\firmware.hex c:\dev\proxxi-halo\dfu\test-fw-603-ns-app.zip

REM c:\Python27\Scripts\nrfutil.exe dfu genpkg --dev-revision 603 --dev-type 603 --application C:\dev\nRF5_SDK_11.0.0_89a8197\examples\ble_peripheral\ble_app_hrs\pca10040\s132_with_dfu\armgcc\_build\s132_pca10040.hex c:\dev\proxxi-halo\dfu\test-ble_app_hrs-603-ns-app.zip

c:\Python27\Scripts\nrfutil.exe dfu genpkg --dev-revision 652 --dev-type 652 --application C:\dev\nRF5_SDK_11.0.0_89a8197\examples\ble_peripheral\ble_app_hrs\pca10040\s132_with_dfu\armgcc\_build\s132_pca10040.hex c:\dev\proxxi-halo\dfu\test-ble_app_hrs-652-ns-app.zip

