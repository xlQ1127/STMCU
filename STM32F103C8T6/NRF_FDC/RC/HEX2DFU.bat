set ex_path=E:\winapp-dfuse-master\Bin\
set file_path=E:\STM32Project\STM32F103C8T6\NRF_FDC\RC\MDK-ARM\RC\
%ex_path%DfuSeCommand.exe -t %file_path%RC.hex %file_path%RC.dfu -c --de 0 -d --fn %file_path%RC.dfu
exit

