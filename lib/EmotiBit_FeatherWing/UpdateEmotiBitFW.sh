f="EmotiBit_FeatherWing_depends.txt"
script_name=$(basename "$0")
echo "**** $script_name ****"
echo "Script to read dependency list from $f and git pull for all repos"
echo "ToDo: read/parse depends directly from library.properties"

eval "$(ssh-agent -s)"
ssh-add ~/.ssh/eval $(ssh-agent -s)
readarray -t repos < $f
echo “${repos[@]}”
for i in ${repos[@]}
do
  echo "-- $i --"
  c="cd ../$i"
  echo $c
  pwd
  eval "$c"
  git pull
done