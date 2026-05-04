import os
import shutil
Import("env")

script_name = os.environ.get("SCRIPT", "")

if not script_name:
    print("\n[upload_script.py] ERRO: variável SCRIPT não definida.")
    print("  Uso: $env:SCRIPT=\"equip_3\"; pio run -t upload\n")
    env.Exit(1)

# Pastas onde procurar o script
project_dir = env["PROJECT_DIR"]
search_dirs = [
    os.path.join(project_dir, "util"),
    os.path.join(project_dir, "scripts", "equip"),
    os.path.join(project_dir, "scripts", "base"),
    os.path.join(project_dir, "scripts", "tdma"),
    os.path.join(project_dir, "scripts", "util"),
    os.path.join(project_dir, "scripts", "tdma", "equip"), 
    os.path.join(project_dir, "scripts", "tdma", "base"),  
]

src_file = None
for folder in search_dirs:
    candidate = os.path.join(folder, script_name + ".cpp")
    if os.path.isfile(candidate):
        src_file = candidate
        break

if not src_file:
    print(f"\n[upload_script.py] ERRO: '{script_name}.cpp' não encontrado em:")
    for d in search_dirs:
        print(f"  {d}")
    print()
    env.Exit(1)

dest = os.path.join(project_dir, "src", "main.cpp")
shutil.copyfile(src_file, dest)
print(f"\n[upload_script.py] '{script_name}.cpp' copiado para src/main.cpp\n")