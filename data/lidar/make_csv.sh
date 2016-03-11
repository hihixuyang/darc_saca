dir=$1

cd $dir
mkdir csv

for i in 0 1 2 3 4 5 6 7 8 9
do
  file="${dir}_${i}"
  mv $file/_slash_scan.csv csv/$file.csv
  rm -rf $file/
done

mkdir bags
mv *.bag bags/
cd ..


