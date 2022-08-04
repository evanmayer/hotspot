echo "Taking image $1...";
# pkill -f gphoto2;
sleep 2;
gphoto2 --capture-image-and-download --force-overwrite --filename ./verification/input/$1.jpg;
