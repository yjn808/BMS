cd .

if "%1"=="" ("F:\MATLAB2018B\bin\win64\gmake"  -f foc_algorithm.mk all) else ("F:\MATLAB2018B\bin\win64\gmake"  -f foc_algorithm.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
An_error_occurred_during_the_call_to_make
