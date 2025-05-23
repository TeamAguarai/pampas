import os
import shutil
import subprocess
import sys

# --- Verificación de argumentos ---
if len(sys.argv) != 4:
    print("Uso: python generate_and_build.py archivo_prueba carpeta_destino nombre_ejecutable")
    sys.exit(1)

test_name = sys.argv[1]
target_dir = sys.argv[2]
output = sys.argv[3]
file_to_test = f"{test_name}"

if not os.path.isfile(file_to_test):
    print(f"[ERROR] El archivo '{file_to_test}' no existe.")
    sys.exit(1)

# --- Ejecutar merge.py ---
try:
    subprocess.run(["python", "scripts/merge.py", "include", ".hpp", "pampas.h"], check=True)
    print("[OK] merge.py ejecutado correctamente.")
except subprocess.CalledProcessError:
    print("[ERROR] No se pudo ejecutar merge.py.")
    sys.exit(1)

# --- Crear carpeta de destino si no existe ---
os.makedirs(target_dir, exist_ok=True)

# --- Crear makefile con el nombre del ejecutable dado ---
makefile_content = f"""all:
\tg++ {os.path.basename(file_to_test)} pampas.cpp -o {output} -lwiringPi
\tsudo ./{output}
"""

makefile_path = os.path.join(target_dir, "makefile")
with open(makefile_path, "w") as f:
    f.write(makefile_content)

# --- Copiar archivos necesarios ---
shutil.copy(file_to_test, os.path.join(target_dir, os.path.basename(file_to_test)))
shutil.move("pampas.h", os.path.join(target_dir, "pampas.h"))

print(f"[OK] Archivo '{file_to_test}' copiado a '{target_dir}'")
print(f"[OK] Archivo 'pampas.h' y 'makefile' movidos a '{target_dir}'")
print(f"[INFO] Ejecutable generado será: {output}")
