# pyright: reportUndefinedVariable=false
import os
import shutil
Import("env")

script_name = os.environ.get("SCRIPT", "")

if not script_name:
    print("\n[upload_script.py] AVISO: variavel SCRIPT nao definida - build automatico do IDE, mantendo src/main.cpp como esta.\n")
else:
    project_dir = env["PROJECT_DIR"]
    search_dirs = [
        os.path.join(project_dir, "util"),
        os.path.join(project_dir, "scripts"),
        os.path.join(project_dir, "scripts", "equip"),
        os.path.join(project_dir, "scripts", "base"),
        os.path.join(project_dir, "scripts", "util"),
    ]

    src_file = None
    for folder in search_dirs:
        candidate = os.path.join(folder, script_name + ".cpp")
        if os.path.isfile(candidate):
            src_file = candidate
            break

    if not src_file:
        print(f"\n[upload_script.py] ERRO: '{script_name}.cpp' nao encontrado em:")
        for d in search_dirs:
            print(f"  {d}")
        print()
        env.Exit(1)
    else:
        dest = os.path.join(project_dir, "src", "main.cpp")
        shutil.copyfile(src_file, dest)
        print(f"\n[upload_script.py] '{script_name}.cpp' copiado para src/main.cpp\n")