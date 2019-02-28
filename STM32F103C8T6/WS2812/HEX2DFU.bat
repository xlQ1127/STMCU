set ex_path=E:\winapp-dfuse-master\Bin\
set file_path=E:\STM32Project\STM32F103C8T6\WS2812\MDK-ARM\WS2812\
%ex_path%DfuSeCommand.exe -t %file_path%WS2812.hex %file_path%WS2812.dfu -c --de 0 -d --fn %file_path%WS2812.dfu
exit

