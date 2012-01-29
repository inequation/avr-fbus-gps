@ECHO OFF
"C:\Program Files (x86)\Atmel\AVR Tools\AvrAssembler2\avrasm2.exe" -S "C:\projects\smiw\gps\labels.tmp" -fI -W+ie -o "C:\projects\smiw\gps\gps.hex" -d "C:\projects\smiw\gps\gps.obj" -e "C:\projects\smiw\gps\gps.eep" -m "C:\projects\smiw\gps\gps.map" "C:\projects\smiw\gps\gps.asm"
