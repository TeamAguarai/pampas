# Regla principal
all: 
	g++ -std=c++11 -I../include $(FILE).cpp ../src/*.cpp -o $(FILE) -lwiringPi -DPAMPAS_DEV
	sudo ./$(FILE)

# Limpieza de ejecutables
clean:
	rm -f *.exe *.o
