# Notes for blender

This is miscellaneous notes about some techniques for each objective.

# Make a Collada file from point cloud

Given point cloud data, convert it to polygon data.

## at Meshlab
1. poison-disk sampling for downsampling
1. translate the points
1. export the data as ply

## at Blender
1. use the skinner script
1. remesh (deg: 12, Smooth)
1. make normals consistent (Ctrl + N)
1. reverse the normals of face

# Segment a model

The resulting data may be too large. This part is to divide it into small components.

Resource: https://www.youtube.com/watch?v=HnuOkJfT8Wg

1. Change the view to orthogonal
1. Hit B and select a part to export
1. Hit P and click "Selection"
1. Export the separated mesh


