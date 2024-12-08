#include "../include/Writer.h"
#include <sstream>

Writer::Writer(std::string filename, std::string header, std::string delim) : is_open(false), delimiter(delim) {
    file.open(filename);
    if (file.is_open()) {
        is_open = true;
        file << header << "\n";
    } else {
        std::cerr << "Error: No se pudo abrir el archivo " << filename << "\n";
    }
}

Writer::~Writer() {
    if (is_open) file.close();
}

void Writer::write_row(const std::vector<std::string>& data) {
    if (!is_open) {
        std::cerr << "Error: El archivo no está abierto.\n";
        return;
    } 
    std::ostringstream oss;
    for (size_t i = 0; i < data.size(); ++i) {
        oss << data[i];
        if (i != data.size() - 1) {
            oss << delimiter;
        }
    }
    file << oss.str() << "\n";
}

void Writer::close() {
    if (is_open) {
        file.close();
        is_open = false;
    }
}


// int main() {
//     // Crear una instancia de la clase CSVWriter
//     Writter csv("datos.csv", "Ancho de Pulso (ms),Velocidad (m/s)");

//     // Verificar si el archivo se abrió correctamente
//     if (!csv.is_file_open()) {
//         return 1; // Salir si el archivo no se abrió
//     }

//     // Simular lecturas de datos y escribir en el archivo
//     for (int i = 0; i < 10000; ++i) {
//         int pulse_width = 1000 + i;           // Ejemplo de ancho de pulso
//         double speed = pulse_width * 0.001;   // Ejemplo de velocidad en m/s

//         csv.write_row(pulse_width, speed);    // Escribir datos en el archivo
//     }

//     // Cerrar el archivo explícitamente (opcional, el destructor también lo hace)
//     csv.close();

//     std::cout << "Datos guardados exitosamente en datos.csv\n";
//     return 0;
// }
