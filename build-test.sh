#!/bin/bash

if [ "$#" -lt 1 ]; then
    echo "Falta el nombre del archivo: $0 <NOMBRE_DEL_ARCHIVO>"
    exit 1
fi

FILE_TO_TEST=$1

INCLUDE_DIR="include"
SRC_DIR="src"
COMBINED_HDR="bundle/pampas.h"
COMBINED_SRC="bundle/pampas.cpp"

mkdir -p bundle

# --- Resolver el orden de los archivos .h ---
declare -A dependencies

# Extraer dependencias de cada archivo .h
for file in "$INCLUDE_DIR"/*.h; do
    filename=$(basename "$file")
    includes=$(grep -oP '#include\s+"(.*?)"' "$file" | cut -d'"' -f2)
    dependencies["$filename"]="$includes"
done

resolved=()        # Archivos ya ordenados
unresolved=()      # Archivos en proceso de resolución

resolve_dependencies() {
    local file=$1
    if [[ " ${resolved[@]} " =~ " $file " ]]; then
        return
    fi
    if [[ " ${unresolved[@]} " =~ " $file " ]]; then
        echo "Error: Dependencias circulares detectadas en $file."
        exit 1
    fi

    unresolved+=("$file")

    for dep in ${dependencies["$file"]}; do
        dep_file=$(basename "$dep")
        if [[ -f "$INCLUDE_DIR/$dep_file" ]]; then
            resolve_dependencies "$dep_file"
        fi
    done

    unresolved=("${unresolved[@]/$file}")
    resolved+=("$file")
}

for file in "${!dependencies[@]}"; do
    resolve_dependencies "$file"
done

# --- Crear los archivos combinados ---

# Crear el archivo combinado pampas.h
echo "Orden de archivos combinados en pampas.h:"
> "$COMBINED_HDR"
for file in "${resolved[@]}"; do
    echo "$file"
    cat "$INCLUDE_DIR/$file" >> "$COMBINED_HDR"
    echo "" >> "$COMBINED_HDR"
done

# Crear el archivo combinado pampas.cpp basado en el orden de pampas.h
echo "Orden de archivos combinados en pampas.cpp:"
> "$COMBINED_SRC"
echo '#include "pampas.h"' > "$COMBINED_SRC"
for file in "${resolved[@]}"; do
    cpp_file="${SRC_DIR}/${file%.*}.cpp"
    if [[ -f "$cpp_file" ]]; then
        echo "$(basename "$cpp_file")"
        cat "$cpp_file" >> "$COMBINED_SRC"
        echo "" >> "$COMBINED_SRC"
    else
        echo "Advertencia: No se encontró el archivo $cpp_file para $file"
    fi
done

# Copiar makefile y el archivo de prueba al bundle
cp test/"$FILE_TO_TEST".cpp bundle/

echo -e "all:\n\tg++ $FILE_TO_TEST.cpp pampas.cpp -o $FILE_TO_TEST -lwiringPi" > bundle/makefile

echo ""
echo "Makefile creado con éxito."