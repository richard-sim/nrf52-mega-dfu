@echo off
set PREFIX=arm-none-eabi-

REM %PREFIX%objcopy -I binary -O elf32-littlearm -B arm %1 %2
%PREFIX%objcopy -I ihex -O elf32-littlearm -B arm %1 %2
