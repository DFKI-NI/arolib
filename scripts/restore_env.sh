#!/bin/bash

#WARNING: this will clear arolib install paths from some environment variables! 
#         Problems will arrise if the user contains the words "arolib"


declare -a envVars=("PATH" "CMAKE_PREFIX_PATH" "CPATH" "LD_LIBRARY_PATH" "LIBRARY_PATH" "PKG_CONFIG_PATH")

for ev in "${envVars[@]}"
do

    paths=$(echo ${!ev} | tr ":" "\n")

    unset $ev

    for p in $paths
    do
        #echo "> [$p]"

        if [[ $p == *"arolib"* ]]; then
            echo "> > Removing '$p' from '$ev'"
        else
            sep=":"
            [ -z "${!ev}" ] && sep=""
            export $ev=${!ev}$sep$p
        fi

    done
done

 
