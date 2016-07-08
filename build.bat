@ECHO OFF
@RD /S /Q "build"
@RD /S /Q  "dist"
del "*.spec"
pyinstaller --onefile --noconsole --noupx --noconfirm --clean --log-level=WARN mpu6050-qt.py