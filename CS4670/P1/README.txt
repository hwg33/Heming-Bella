We implemented the second whistle (modifying the link costs so that the image is blurred before the link costs are calculated).
We didn't modify the UI; however, the blur amount can be adjusted in the InitNodeBuf method in iScissor.cpp.
The variable is called "blurAmt." This number indicates the size of the blur kernel used
For instance, if blurAmt = 3, the image will be cross-correlated with a 3x3 blur kernel.
