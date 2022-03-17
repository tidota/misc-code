# Morse code

Some notes about commands to practice Morse code..

## morse

Morse code training

```
sudo apt install morse
```

```
morse -r -w 15 -a -v 0.08 -n 1
```
```
morse -r -a -w 15 -n 3 -N 3
```

## cw

`cw` plays Morse code based on the given text

Playing Morse code repeatedly
```
watch -n 1 'date "+%H %M %S" | cw -v 1 -w 20 -t 700'
```
