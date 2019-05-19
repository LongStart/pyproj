 export PYTHONPATH=$PYTHONPATH:$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
# echo $( cd "$(dirname "$1")" ; pwd -P )
# echo $(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)