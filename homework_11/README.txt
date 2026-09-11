Перед першим запуском чекера треба ввімкнути підтримку віртуальних чіпів у ядрі
sudo modprobe gpio-sim
sudo ./checker_linux_x86_64 -uart /tmp/ttyB --start-line 24 --drop-line 23 1

Якщо при запуску чекера виникає помилка uart open: Permission denied, виконайтте
sudo socat -d -d pty,raw,echo=0,link=/tmp/ttyA,mode=0777 pty,raw,echo=0,link=/tmp/ttyB,mode=0777

sudo ./student -uart /tmp/ttyA --gpiochip gpiochip0 --start-line 24 --drop-line 23


