Багатопоточність — цілі, фізика та місія в окремих потоках
Cимуляція тепер іде в реальному часі — потоки сплять між кроками


dir build: command
cmake -DCMAKE_CXX_FLAGS="-fsanitize=thread -g" -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=thread" ..
make
./homework_10