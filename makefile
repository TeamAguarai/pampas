SRC = $(wildcard src/*.cpp)
HDR = $(wildcard include/*.h)

# Archivos combinados
COMBINED_SRC = build/pampas.cpp
COMBINED_HDR = build/pampas.h

# Objetos
OBJ = build/pampas.o

default: build combine compile install

combine:
	for file in $(HDR); do cat $$file; echo ""; done > $(COMBINED_HDR)
	for file in $(SRC); do cat $$file; echo ""; done > $(COMBINED_SRC)

compile:
	g++ -fPIC -Ibuild -c $(COMBINED_SRC) -o $(OBJ) -Iinclude -lwiringPi -DPAMPAS_DEV
	g++ -shared -o build/libpampas.so $(OBJ)

# si la libreria ya existe, se sobreescribe
install:
	mv -f build/libpampas.so /usr/lib/ 
	cp $(COMBINED_HDR) /usr/local/include/pampas.h

build:
	mkdir -p build

clean:
	rm -rf build
