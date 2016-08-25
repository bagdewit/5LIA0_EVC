echo L300 R300 
./i2ctest 3DRI+300+300000 16
sleep 2
echo R900
./i2ctest 3DRI+000+900000 16
sleep 2
echo L900
./i2ctest 3DRI+900+000000 16
sleep 2
#./i2ctest 3DRI+300+300000 16
#./i2ctest 3DRI+300+300000 16
#./i2ctest 3DRI+300+300000 16
