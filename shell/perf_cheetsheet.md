# perf cheetsheet

## Installation
```
sudo apt update && sudo apt install linux-tools-common
sudo apt install linux-tools-<kernel version>-generic
```

For example,
```
sudo apt update && sudo apt install linux-tools-common
sudo apt install linux-tools-4.15.0-112-generic
```

While running a process and check the PID (let's say it is 7243)

```
sudo perf record -F 3000 --call-graph dwarf -p 7243 sleep 10
```

## Using Firegraph

```
git clone https://github.com/brendangregg/FlameGraph.git
cd FlameGraph
sudo perf record -F 3000 --call-graph dwarf -p 7243 sleep 10
sudo perf script | ./stackcollapse-perf.pl > out.perf-folded
cat out.perf-folded | ./flamegraph.pl > perf-kernel.svg
```

Open the svg file by the web browser.
