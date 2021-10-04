# Text-to-Speech

My personal notes for installation of the fllowing software to Ubuntu: Softalk and Open JTalk

# Softalk
https://talkstone.web.fc2.com/#ubuntu

1. create a drive in playonlinux

   - win32
	 - wine3.0
	 - dotnet4.0

1. Get Takao font file (ttf)

   https://launchpad.net/takao-fonts

   Copy the file to .PlayOnLinux/fonts

1. Add a new registry

   \HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Windows NT\CurrentVersion\FontSubstitutes

   - "MS Shell Dlg"="TakaoPGothic"
	 - "MS Shell Dlg 2"="TakaoPGothic"
	 - "MS UI Gothic"="TakaoPGothic"

1. Get the file of Softalk

   - Copy files of softalk to the drive
   - Add a shortcut to the exe file

1. In the miscellaneous setting

   `LANG="ja_JP.UTF8"``

# Open JTalk
https://kledgeb.blogspot.com/2014/05/ubuntu-open-jtalk-1-open-jtalkopen-jtalk.html

Install the packages.
```
sudo apt install open-jtalk
sudo apt install hts-voice-nitech-jp-atr503-m001 open-jtalk-mecab-naist-jdic
```

Generate a sample file.
```
echo 今日の天気は晴れです。 | open_jtalk -x /var/lib/mecab/dic/open-jtalk/naist-jdic -m /usr/share/hts-voice/nitech-jp-atr503-m001/nitech_jp_atr503_m001.htsvoice -ow ~/open_jtalk.wav
```


https://kledgeb.blogspot.com/2014/05/ubuntu-open-jtalk-3-mmdagentmeimei-mei.html

Download `MMDAgent_Example-1.8.zip` to `~/Downloads`.
Then, extract the files and copy them to `/usr/share/hts-voice`.
```
cd ~/Downloads
unzip MMDAgent_Example-1.8.zip
sudo cp -R MMDAgent_Example-1.8/Voice/mei /usr/share/hts-voice/
```

Generate a sample file.
```
echo 今日の天気は晴れです。 | open_jtalk -x /var/lib/mecab/dic/open-jtalk/naist-jdic -m /usr/share/hts-voice/mei/mei_normal.htsvoice  -ow ~/open_jtalk.wav
```


Script to do the same thing:
```
#!/bin/bash
if [ $# -eq 2 ]; then
    echo $1 | open_jtalk -x /var/lib/mecab/dic/open-jtalk/naist-jdic -m /usr/share/hts-voice/mei/mei_normal.htsvoice  -ow ~/$2
elif [ $# -eq 4 ]; then
    echo $1 | open_jtalk -a $3 -fm $4 -x /var/lib/mecab/dic/open-jtalk/naist-jdic -m /usr/share/hts-voice/mei/mei_normal.htsvoice  -ow ~/$2
else
    echo "Usage: sh $0 message ofile [voice tone] [pitch shift]"
fi
```
To use it (say the script name is `jtalk_gen.sh`),
```
sh jtalk_gen.sh 今日の天気は晴れです。 output.wav 0.45 10
```
