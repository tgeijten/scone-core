del xml\*.* /Q
del ref\*.* /Q
"C:\Program Files\doxygen\bin\doxygen.exe" Doxyfile
"..\..\..\dokugen\bin\vc2019-x64\Release\dokugen.exe" xml ref -r classscone_1_1_ -r structscone_1_1_
copy ref\*_hfd.txt ref\*_hyfydy.txt
dir /b ref\*.txt > "..\..\resources\help\keywords.txt"

cd tools
python generate_sconepy.py
cd ..
