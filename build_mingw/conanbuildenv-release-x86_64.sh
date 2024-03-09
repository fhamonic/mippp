script_folder="/home/plaiseek/Projects/mippp/build_mingw"
echo "echo Restoring environment" > "$script_folder/deactivate_conanbuildenv-release-x86_64.sh"
for v in LD CXX CC PATH
do
    is_defined="true"
    value=$(printenv $v) || is_defined="" || true
    if [ -n "$value" ] || [ -n "$is_defined" ]
    then
        echo export "$v='$value'" >> "$script_folder/deactivate_conanbuildenv-release-x86_64.sh"
    else
        echo unset $v >> "$script_folder/deactivate_conanbuildenv-release-x86_64.sh"
    fi
done


export LD="/home/plaiseek/Softwares/xpack-mingw-w64-gcc-12.2.0-1/bin/x86_64-w64-mingw32-ld"
export CXX="/home/plaiseek/Softwares/xpack-mingw-w64-gcc-12.2.0-1/bin/x86_64-w64-mingw32-g++"
export CC="/home/plaiseek/Softwares/xpack-mingw-w64-gcc-12.2.0-1/bin/x86_64-w64-mingw32-gcc"
export PATH="/home/plaiseek/.conan2/p/cmakecf6b18ccaa9f5/p/bin:$PATH"