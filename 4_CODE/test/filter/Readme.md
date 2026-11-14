### 1. Configure CMake
```bash
cmake -B _build -DCMAKE_TOOLCHAIN_FILE=Cmake/arm-none-eabi.cmake -G "MinGW Makefiles"
```



### 6. Build All Targets
```bash
cmake --build _build -j4
```

```bash
# Flash Bootloader to 0x08000000 via J-Link
cmake --build _build --target flash_boot

# Flash Application to 0x08010000 via J-Link
cmake --build _build --target flash_app

# Erase Entire Flash via J-Link
cmake --build _build --target erase_flash
```
