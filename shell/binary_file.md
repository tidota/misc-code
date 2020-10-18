# Binary file inspection

# objdump

show assembly code without instruction bytes
```
objdump --no-show-raw-insn -d <binary file>
```

# readelf

does the same thing as objdump but elf-specific
```
readelf -a <binary file>
```

