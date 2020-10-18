# GIF anime from images

This is a note to make a gif animation from still pictures.

# Resize image size
```
mogrify -resize 20% *.JPG
```

# Convert the resized images to GIF animation
```
convert -verbose -delay 10 -loop 0 *.JPG anime.gif
```

