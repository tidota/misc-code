<h1 class="title">(Incomplete) Unix/Linux Command List</h1>

Here is my notes about Unix commands for some purposes. Some commands can do more than the descriptions below.

**Some commands cause things which cannot be undone. Please be careful and do at your own risk.**

# Exploring Directories
## ***pwd***
shows the current directory

## ***ls***
lists files and directories

- `ls`

  just shows usual files and folders

- `ls -la`

  lists all items including hidden files/directories with additional information

## ***cd***
moves to the specified directory

- `cd ./foo`

  goes to the directory named "foo"

- `cd ..`

  goes to the parent directory

- `cd ~` or `cd`

  goes to your home directory

# Checking Usage of Commands
## ***man***

shows manual of command

- `man ls`
- `man gcc`

## ***whereis***
shows the location of the executable of a command

- `whereis ls`
- `whereis gcc`

# Manipulating Files and Directories
## ***touch***
updates the access time / creates an empty file (if the file does not exist)
- `touch file`

## ***mkdir***
creates an empty directory
- `mkdir foo`

## ***mv***
moves a file and a directory
- `mv file ./foo`

  moves the file "file" to the directory "foot"

- `mv ./foo/file ./foo/renamed`

  renames the file to "renamed" (if a directory named as "renamed" does not exist)

## ***cp***
copies a file and a directory

- `cp file ./foo`

  copies the file "file" to the directory "foo"

- `cp -r foo bar`

  copies the directory "foo" to the directory "bar"

## ***rm***
deletes a file

_note: once you delete a file by rm command, you cannot recover it. If you are dealing with very important files, you should have backups._

## ***rmdir***
deletes an EMPTY directory

You need to clean inside the directory before removing it.

rm command with -r option will delete all contents including the directory.


As of April 2017, UH Unix (SunOS) only removes files but keeps directories unchanged for some reasons.
This problem may be solved after they switch their OS to Redhat Linux this Fall.

At this point, these commands remove a directory with files and subdirectories under it. (Make sure that you are deleting the right one and have backups.)

- `find [directory to remove] -exec rm {} \;`

  This removes all files under the directory.

- `find [directory to remove] -depth -exec rmdir {} \;`

  This removes all directories (including the top one) from the bottom to the top.

## ***chmod***
changes access rights

An octal represents three bits corresponding reading (r), wrighting (w), and execution rights (x).

4: read only, 5: read/run, 6:read/wright, 7: full access

- `chmod 644 file`

  6 for owner (u), 4 for those in the same group (g), 4 for others (o)

- `chmod u+x file`

  add execution right to the owner

# Passing Data
## ***cat***
concatenates and displays strings from files and standard input

- `cat /etc/bashrc`

  displays the contents in bashrc

- `cat /etc/hosts`

  displays the contents in hosts

- `cat /etc/bashrc /etc/hosts`

  displays the contents in bashrc and hosts

## ***head***
shows the first n lines

- `head -5 /etc/hosts`

## ***tail***
shows the last n lines

- `tail -5 /etc/hosts`

## ***more***
browses a file (one direction)

To view the next page, hit z.

When browsing, hit h to view help page of more.

- `more /etc/wgetrc`

## ***less***
browses a file (both directions)

To view the next page, hit z.

To view the previous page, hit w.

When browsing, hit h to view help page of more.

- `less /etc/wgetrc`

## ***echo***
displays a string

- `echo Hello World`

## ***grep***
displays lines containing a given word in files

- `grep -ns "localhost" /etc/*`

  searches lines containing "localhost" in files located in /etc. It also shows the line numbers (by option n) and surpresses errors (by option s).

## ***wc***
counts # of lines, words, and bytes

- `wc /etc/hosts`
- `wc -l /etc/hosts`

  only shows # of lines

## ***Redirection***
saves output to a file

- `ls -la > result.txt`

  stores the result of "ls -la" to the file result.txt (the file is overwritten)

- `echo foo bar baz >> result.txt`

  appends "foo bar baz" to the file result.text

You can also save the output of your program (output of printf) to a file by redirection.

## ***Piping***
passes output to another command

- `echo hello this is a message | wc`

- `ls -l ~ | wc -l`

  counts # of files and directories

You can also pass the output of your program (output of printf) to other commands by piping.

## ***sed***
stream editor

- `echo good morning | sed 's/morning/night/'`

  replaces "morning" with "night"

- `echo foobarbazfoobarbaz | sed 's/bar//'`

  removes "bar" (processes only on the first hit)

- `echo foobarbazfoobarbaz | sed 's/bar//g'`

  removes all "bar"

- `sed -i 's/foo/bar/g' file.txt`

  replace "foo" with "bar" in the file.

## ***awk***
filters strings

- `ls -la | awk '{print $3 "\t" $9}'`

  prints the 3rd (owner's name) and 9th (file name) columns separated by tab

## ***script***
records transactions<br>

To start, type "script" and hit enter.<br>To stop, type "exit" or hit Ctrl+D.<br>When a file name given, data will be written to the specified file.

# Searching Files
## ***locate***
searches files by using database<br>

- `locate "*.html"`

  searches all html files

## ***find***
searches files by looking through directories

- `find /usr/local/doc/gcc -name "*.html"`

  searches html files in /usr/local/doc/gcc

# Editors
## ***pico***
## ***vi***
## ***emacs***

# SSH
## ***ssh***
## ***scp***
## ***sftp***
## ***ssh-keygen***
## ***ssh-copy-id***

# Archiving files
## ***zip/unzip***
## ***tar***

# Misc
## ***ln***
## ***who***
## ***last***
## ***ps aux***
## ***top***
## ***dd***
## ***rsync***
