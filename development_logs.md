

### Commands: 


## Running the code 


Initial test and installations:

```
 1107  arm-none-eabi-gcc --version
 1108  which arm-n
 1109  brew search gcc-arm-none-eabi
 1110  brew search gcc-r-arm-none-eabi
 1111  brew search gcc-arm-none-eabi



 1112  brew install --cask gcc-arm-embedded


 1113  arm-none-eabi-gcc --version

 1114  make
```



Final run: 
```
cd src/compile
make clean
make
```




Running the board on server: 

list the devices
```
ls /dev/tty.usb*
```


```

ls -l /dev/tty.usb*


openocd -f interface/stlink.cfg -f target/stm32f4x.cfg


```






Flashing the program: 

```

arm-none-eabi-gdb src/compile/build/final.elf
or,
arm-none-eabi-gdb build/final.elf



target remote localhost:3333

target extended-remote :3333

Warn : Prefer GDB command "target extended-remote :3333" instead of "target remote :3333"


load



```





# Terminal 1: Start OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

# Terminal 2: Start GDB
arm-none-eabi-gdb your_program.elf

# Inside GDB:
(gdb) target remote localhost:3333    # Connect to OpenOCD
(gdb) monitor reset halt             # Reset and halt the CPU
(gdb) load                          # Load your program
(gdb) break main                    # Set breakpoint at main
(gdb) continue                      # Start program execution



Create diff: 
```
diff -ruN --exclude=".git" --exclude="development_logs.md" --exclude="draft.txt" --exclude=".gitignore" duos24_original duos24 > changes.patch


diff -ruN duos24_original duos24 > changes.patch


diff -ruN --exclude=".git" --exclude="development_logs.md" --exclude="draft.txt" --exclude=".gitignore" duos24_original duos24 > changes.patch
```



## Debugging

Initializing QEMU: 
```
qemu-system-arm -M mps2-an386 -kernel src/compile/build/final.elf -S -gdb tcp::1234
```

QEMU supported processors: 
```
qemu-system-arm -machine help
```


Running the debugger: 

```
arm-none-eabi-gdb src/compile/build/final.elf
```




## Chain of commands 


```



lubaina123@DESKTOP-IE1HBB2:/mnt/c/Users/Lubaina/Downloads/new$ history    


    1  lsusb
    Lists all USB devices connected to the system. Useful for checking if your development board or programmer (e.g., ST-Link) is connected.
    MacOS Equivalent: Use system_profiler SPUSBDataType or ioreg -p IOUSB.

    2  sudo apt install lsusb       

    3  sudo apt install usbutils
    Purpose: Installs usbutils, a collection of tools including lsusb for managing and displaying information about USB devices.

    4  lsusb    
    5  cd src/compile
    6  sudo apt install openocd   
    Purpose: Installs OpenOCD (Open On-Chip Debugger) for programming and debugging embedded systems via interfaces like ST-Link.
    MacOS Equivalent: brew install openocd.

    7  sudo apt update
    Purpose: Updates the package index to ensure the latest versions of packages are installed.
    MacOS Equivalent: brew update.
    
    9  sudo apt install gcc-r-arm-none-eabi
    Purpose: Installs a specific version of the ARM GCC toolchain used for cross-compiling code for ARM-based microcontrollers.
    MacOS Equivalent: brew install arm-none-eabi-gcc.

   10  sudo apt install gcc-arm-none-eabi   
    Purpose: Installs the standard arm-none-eabi-gcc cross-compiler for embedded ARM development.

   11  make all
    Purpose: Runs the Makefile in the current directory to compile all source files and produce the output binaries (e.g., .elf or .bin files).
    Explanation: The Makefile defines how to build the project by specifying rules for compiling and linking files.

   12  sudo apt install make   
    Purpose: Installs make, a build automation tool used to compile projects based on Makefile instructions.
    MacOS Equivalent: Comes pre-installed or can be installed with xcode-select --install
    
   14  make load   
    Purpose: A custom Makefile target for flashing the compiled code onto the microcontroller. 
    It may invoke tools like openocd or STM32CubeProgrammer to handle this process.
   
   18  sudo apt install minicom   
    Purpose: Installs minicom, a serial communication tool for monitoring UART output from embedded systems.
    MacOS Equivalent: Use screen or install minicom with brew install minicom.

   19  minicom -s
    Purpose: Opens minicom in setup mode to configure serial port settings (e.g., baud rate, device path).
   
   23  sudo make clean
     Purpose: Cleans up compiled object files, binaries, and other build artifacts from the project directory, preparing for a fresh build.

   24  diff -ruN folder_original folder_modified > changes.patch   
   24  diff -ruN folder_original folder_modified > changes.patch   
    Purpose: Compares the contents of two directories (folder_original and folder_modified) recursively and generates a patch file 
    (changes.patch) containing the differences. Useful for tracking changes or creating updates.
    
   26  history




   

lsusb
lsusb 
usbutils 
openocd 

gcc-r-arm-none-eabi
gcc-arm-none-eabi  

make 
minicom


```