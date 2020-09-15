@echo off
set BinSpace=.\..\..\..\bin\x64\Release
set anchorName=anchor2

echo createDB
%BinSpace%\SinglePictureCreateDB.exe .\middle %anchorName% 
echo;
%BinSpace%\SinglePictureCreateDB.exe .\left %anchorName% 
echo;
%BinSpace%\SinglePictureCreateDB.exe .\right %anchorName% 
echo;