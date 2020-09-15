@echo off
set BinSpace=.\..\..\..\bin\x64\Release
set anchorName=anchor1
echo left standard create
%BinSpace%\sideStandardPictureCreate.exe .\left
echo;
echo right standard create
%BinSpace%\sideStandardPictureCreate.exe .\right
echo;

echo renamePicture and createConfigtxt
%BinSpace%\changeName.exe .\middle
%BinSpace%\changeName.exe .\left
%BinSpace%\changeName.exe .\right 
echo;

