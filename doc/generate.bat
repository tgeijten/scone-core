del xml\*.* /Q
del ref\*.* /Q
"C:\Program Files\doxygen\bin\doxygen.exe" Doxyfile
"..\..\..\dokugen\bin\vc2019-x64\Release\dokugen.exe" xml ref -r classscone_1_1_ -r structscone_1_1_ -r s_c_o_n_e_h_f_d___n_a_m_e_s_p_a_c_e_1_1_
copy ref\*_hfd.txt ref\*_hyfydy.txt
dir /b ref\*.txt > "..\..\resources\help\keywords.txt"

cd tools
python sconepy_dokuwiki_generator.py
cd ..
