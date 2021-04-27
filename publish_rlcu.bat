@echo off
call "venv\scripts\activate.bat"
python setup.py sdist bdist_wheel
twine upload dist/*
pause