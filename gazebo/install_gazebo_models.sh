#!/bin/sh

# Download all model archive files
git clone https://github.com/osrf/gazebo_models.git

# This is the folder into which wget downloads the model archives
cd gazebo_models

# Extract all model archives
for i in *
do
  tar -zvxf "$i/model.tar.gz"
done

# Copy extracted files to the local model folder
# cp -vfR * "$HOME/.gazebo/models/"
