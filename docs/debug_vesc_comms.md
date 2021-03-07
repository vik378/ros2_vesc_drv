# Debugging VESC Communications

The following setup is used for comms debug:

* [vesc_tool](https://github.com/vedderb/vesc_tool) to drive and monitor VESC operation.
* `strace -f -p <vesc_tool pid> -s 512 -o <experiment>.strace` to intercept the serial comms of the `vesc_tool` and store it in a text file
* inspect the strace by hand (`cat ...` and / or `grep ...`) to confirm that what you are after is there.
* `cat <experiment>.strace | python3 ./test/utils/strace_cleaner.py > <experiment>.py`  .py file for replaying against software / jupyter
  
