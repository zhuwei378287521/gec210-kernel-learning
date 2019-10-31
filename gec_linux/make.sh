make -j4
mkimage -n 'linux-2.6.35.7' -A arm -O linux -T kernel -C none -a 0x30008000 -e 0x30008040  -d zImage uImage
#cp uImage  210/uImage-test
