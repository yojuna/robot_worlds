#!/bin/sh
mkdir -p "models"
for zip in *.zip
do
  echo `unzipping $zip`
  unzip ./"$zip" -d ./models/
  dirname=`echo $zip | sed 's/\.zip$//'`
  mv ./models/"$dirname"/* ./models/
#   if mkdir "$dirname"
#   then
#     if cd "$dirname"
#     then
#       unzip ../"$zip"
#       cd ..
#       # rm -f $zip # Uncomment to delete the original zip file
#     else
#       echo "Could not unpack $zip - cd failed"
#     fi
#   else
#     echo "Could not unpack $zip - mkdir failed"
#   fi
done
